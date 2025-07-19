#include "World.hpp"
#include <algorithm>
#include <iostream>

using namespace std;
using ull = unsigned long long;
const int lim = 16;

World::World(int w, int h) 
{
    length = 1;
    while(length < max(w, h))
        length <<= 1;
    
    int cnt = 1;
    int ind = 0;
    int tmp = length;
    while(tmp >= lim)
    {
        levels.push_back(ind);
        occ.push_back(0);
        ind += cnt;
        cnt *= 4;
        tmp >>= 1;
    }
}

int World::getIndex(int lvl, int x, int y)
{
    int cnt = 1<<lvl;
    if(x < 0 || x >= cnt || y < 0 || y >= cnt)
        return -1;
    return levels[lvl] + y * (1<<lvl) + x;
}

void World::updateIndex(int id)
{
    Body& body = bodies[id];
    body.calculateAABB();
    const AABB& aabb = body.aabb;    
    float len = max(aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y);
    
    int curr = length;
    body.level = 0;
    while(body.level < levels.size() - 1 && (curr >> 1) >= len)
    {
        curr >>= 1;
        body.level++;
    }

    int gx = (int)floor(aabb.min.x / curr);
    int gy = (int)floor(aabb.min.y / curr);

    body.ind = getIndex(body.level, gx, gy);

    if(body.ind == -1)
        deleteObject(id);
}

void World::getNeighbors(int id, vector<pair<int, int>>& local)
{
    const Body& body = bodies[id];
    int mX = body.aabb.min.x;
    int mY = body.aabb.min.y;
    for(int i = body.level; i >= 0; i--)
    {
        if(!occ[i])
            continue;
        int curr = length >> i;
        int gx = (int)floor(mX / curr);
        int gy = (int)floor(mY / curr);
        for(int x = gx-1; x <= gx+1; x++)
        {
            for(int y = gy-1; y <= gy+1; y++)
            {
                int ind = getIndex(i, x, y);
                if(ind != -1)
                {
                    for(int id2: grid[ind])
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

// Adds an object, returns its ID
int World::addObject(const Vec2& pos, const Vec2& vel, int meshID, float mass, float MoI, float scale, float ang, float res) {
    int id;

    if (!freeList.empty()) {
        id = freeList.back();
        bodies[id] = Body(pos, vel, meshID, 1.0f / mass, 1.0f / MoI, scale, ang, res);
        freeList.pop_back();
    } else {
        id = allocated++;
        bodies.emplace_back(pos, vel, meshID, 1.0f / mass, 1.0f / MoI, scale, ang, res);
    }
    return id;
}

int World::addObject(const Vec2& pos, int meshID, float scale, float ang, float res) {
    int id;

    if (!freeList.empty()) {
        id = freeList.back();
        bodies[id] = Body(pos, Vec2(0, 0), meshID, 0.0f, 0.0f, scale, ang, res, 2);
        freeList.pop_back();
    } else {
        id = allocated++;
        bodies.emplace_back(pos, Vec2(0, 0), meshID, 0.0f, 0.0f, scale, ang, res, 2);
    }
    return id;
}

void World::deleteObject(int id)
{
    if(bodies[id].active == 0)
        return;
    bodies[id].active = 0;
    freeList.push_back(id);
}

void World::initGrid()
{
    activeCount = 0;
    for(int id = 0; id < allocated; id++) 
    {
        if(bodies[id].active)
        {
            updateIndex(id);
            if(bodies[id].ind >= 0)
            {
                occ[bodies[id].level] = 1;
                grid[bodies[id].ind].push_back(id);
                activeCount++;
            }
        }
    }
}

void World::resetGrid()
{
    for(int id = 0; id < allocated; id++) 
    {
        if(bodies[id].active) 
        {
            int ind = bodies[id].ind;
            if(!grid[ind].empty())
                grid[ind].clear();
        }
    }

    for(int& i: occ)
        i = 0;
}

void World::resetForces(const Vec2& g)
{
    for(int id = 0; id < allocated; id++) 
        bodies[id].acceleration = g;
}

void World::applyCorrections()
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

void World::applyForce(int id, const Vec2& force)
{
    bodies[id].acceleration += force * bodies[id].invMass;
}

void World::updateVelocities(float dt)
{
    for(int id = 0; id < allocated; id++) 
    {
        if(bodies[id].active == 1)
            bodies[id].velocity += bodies[id].acceleration * dt;
    }
}

void World::updatePositions(float dt)
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

void World::resolveCollisions()
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

void World::genCollisionPairs() 
{

}