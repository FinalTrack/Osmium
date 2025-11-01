#pragma once
#include <vector>
#include <array>
#include "math/Vec2.hpp"
#include "structures/AABB.hpp"
#include "Mesh.hpp"
#include "Body.hpp"
#include "structures/Quad.hpp"

//Lightweight container for Bodies, collision pairs, and the QuadGrid broadphase.
struct World 
{
    int allocated, activeCount;
    int colCnt;

    std::vector<int> freeList;
    std::vector<Body> bodies;

    std::vector<std::pair<int, int>> collisionPairs;
    std::vector<CollisionResult> collisionData;

    QuadGrid quad;

    World(int w, int h) : quad(std::max(w, h)) {}

    // Add new dynamic body, reuses id from freeList if available, otherwise appends to `bodies`.
    int addBody(const Vec2& pos, const Vec2& vel, int meshID, float mass, float MoI, float scale, float ang, float res) 
    {
        int id;
        if(!freeList.empty()) 
        {
            id = freeList.back();
            bodies[id] = Body(pos, vel, meshID, 1.0f / mass, 1.0f / MoI, scale, ang, res);
            freeList.pop_back();
        } 
        else 
        {
            id = allocated++;
            bodies.emplace_back(pos, vel, meshID, 1.0f / mass, 1.0f / MoI, scale, ang, res);
        }
        return id;
    }

    // Add new static body
    int addBody(const Vec2& pos, int meshID, float scale, float ang, float res) 
    {
        int id;
        if(!freeList.empty()) 
        {
            id = freeList.back();
            bodies[id] = Body(pos, Vec2(0, 0), meshID, 0.0f, 0.0f, scale, ang, res, 2);
            freeList.pop_back();
        } 
        else 
        {
            id = allocated++;
            bodies.emplace_back(pos, Vec2(0, 0), meshID, 0.0f, 0.0f, scale, ang, res, 2);
        }
        return id;
    }

    // Marks body inactive and pushes its id to freeList.
    // Does NOT immediately remove the id from quad.grid — grid init / reset handles that.
    void deleteBody(int id)
    {
        if(bodies[id].active == 0)
            return;
        bodies[id].active = 0;
        freeList.push_back(id);
    }

    // Recomputes body AABB and chooses quad level based on AABB size.
    // Computes grid coordinates and flattened index using QuadGrid helpers.
    void updateIndex(int id)
    {
        Body& body = bodies[id];
        body.calculateAABB();
        const AABB& aabb = body.aabb;    
        float len = std::max(aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y);
        body.level = quad.getLevel(len);
        
        int gx, gy;
        quad.gridCoord(gx, gy, body.level, aabb.min.x, aabb.min.y);
        body.ind = quad.getIndex(body.level, gx, gy);

        if(body.ind == -1)
            deleteBody(id);
    }

    // Rebuilds the quad.grid from scratch by iterating all active bodies
    void initGrid()
    {
        activeCount = 0;
        for(int id = 0; id < allocated; id++) 
        {
            if(bodies[id].active)
            {
                updateIndex(id);
                if(bodies[id].ind >= 0)
                {
                    quad.occ[bodies[id].level] = 1;
                    quad.grid[bodies[id].ind].push_back(id);
                    activeCount++;
                }
            }
        }
    }

    // Clears per-cell occupant lists referenced by active bodies; resets occupancy flags.
    void resetGrid()
    {
        for(int id = 0; id < allocated; id++) 
        {
            if(bodies[id].active) 
            {
                int ind = bodies[id].ind;
                if(!quad.grid[ind].empty())
                    quad.grid[ind].clear();
                //Might be worth clearing the memory here as well
            }
        }
        for(int& i: quad.occ)
            i = 0;
    }

    // Produces potential collision pairs for `id` by scanning 3x3 neighborhoods
    // from the body's level up to the coarsest level (level 0).
    // Performs simple dedup avoidance (when scanning the same level, emit only id < id2).
    // Final fast check: AABB overlap before adding to `local`.
    void getNeighbors(int id, std::vector<std::pair<int, int>>& local)
    {
        const Body& body = bodies[id];
        int mX = body.aabb.min.x;
        int mY = body.aabb.min.y;
        for(int i = body.level; i >= 0; i--)
        {
            if(!quad.occ[i])
                continue;
            int gx, gy;
            quad.gridCoord(gx, gy, i, mX, mY);
            for(int x = gx-1; x <= gx+1; x++)
            {
                for(int y = gy-1; y <= gy+1; y++)
                {
                    int ind = quad.getIndex(i, x, y);
                    if(ind != -1)
                    {
                        for(int id2: quad.grid[ind])
                        {
                            if(bodies[id].active == 2 && bodies[id2].active == 2)   
                                continue;
                            if(i == body.level && id >= id2)
                                continue;
                            if(body.aabb.overlaps(bodies[id2].aabb))
                                local.emplace_back(id, id2);
                        }
                    }
                }
            }
        }
    }

    void resetForces(const Vec2& g)
    {
        for(int id = 0; id < allocated; id++) 
            bodies[id].acceleration = g;
    }

    void applyCorrections()
    {
        for(int id = 0; id < allocated; id++) 
        {
            if(bodies[id].active == 1)
            {
                bodies[id].position += bodies[id].correction;
                bodies[id].correction = Vec2(0, 0);
            }
        }
    }

    void applyForce(int id, const Vec2& force)
    {
        bodies[id].acceleration += force * bodies[id].invMass;
    }

    void updateVelocities(float dt)
    {
        for(int id = 0; id < allocated; id++) 
        {
            if(bodies[id].active == 1)
                bodies[id].velocity += bodies[id].acceleration * dt;
        }
    }

    void updatePositions(float dt)
    {
        for(int id = 0; id < allocated; id++) 
        {
            if(bodies[id].active == 1)
            {
                bodies[id].position += bodies[id].velocity * dt;
                bodies[id].theta += bodies[id].omega * dt;
                bodies[id].cosTheta = std::cos(bodies[id].theta);
                bodies[id].sinTheta = std::sin(bodies[id].theta);
            }
        }
    }

    // Walks collisionData / collisionPairs and calls Body::resolve for actual impulse resolution.
    void resolveCollisions()
    {
        colCnt = 0;
        for (int i = 0; i < collisionData.size(); ++i)
        {
            auto [id1, id2] = collisionPairs[i];
            if (!collisionData[i].collide)
                continue;
            colCnt++;
            Body::resolve(bodies[id1], bodies[id2], collisionData[i]);
        }
    }
};