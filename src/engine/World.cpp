#include "World.hpp"
#include <algorithm>

const int offset = 10;

World::World(float cellSize, AABB b, float r, float c)
    : cellSize(cellSize), entityCount(0), border(b), RES(r), CORR(c) {}

AABB World::calculateAABB(const Vec2& position, int meshID, float scale) const {
    if (meshID == 1000) { 
        const float radius = 10.0f * scale; 

        Vec2 minPos = position - Vec2(radius, radius);
        Vec2 maxPos = position + Vec2(radius, radius);

        return AABB(minPos, maxPos);
    }
    const Mesh& mesh = meshes[meshID];
    Vec2 minPos = position;
    Vec2 maxPos = position;

    for (const Vec2& point : mesh.points) {
        Vec2 tp = position + point*scale;
        minPos = Vec2::min(minPos, tp);
        maxPos = Vec2::max(maxPos, tp);
    }

    return AABB(minPos, maxPos);
}

// Adds an object, returns its ID
int World::addObject(const Vec2& pos, const Vec2& vel, int meshID, float mass, float scale) {
    int id;

    if (!freeList.empty()) {
        id = freeList.back();
        freeList.pop_back();
        positions[id] = pos;
        corrections[id] = Vec2(0, 0);
        velocities[id] = vel;
        invMass[id] = 1.0f / mass;
        scales[id] = scale;
        accelerations[id] = Vec2(0, 0);
        active[id] = 1;
    } else {
        id = entityCount++;
        positions.push_back(pos);
        corrections.push_back(Vec2(0, 0));
        velocities.push_back(vel);
        invMass.push_back(1.0f / mass);
        scales.push_back(scale);
        accelerations.push_back(Vec2(0, 0));
        aabbs.push_back(AABB());
        meshIDs.push_back(meshID);
        prev.push_back({1, 1, 0, 0}); 
        active.push_back(1);
    }

    updateObject(id, calculateAABB(pos, meshID, scale)); 
    return id;
}

int World::addObject(const Vec2& pos, int meshID) {
    return -1;
}

void World::resetForces(const Vec2& g)
{
    for(int id = 0; id < entityCount; id++) {
        accelerations[id] = g;
    }
}

void World::applyCorrections()
{
    for(int id = 0; id < entityCount; id++) {
        positions[id] += corrections[id];
        corrections[id] = Vec2(0, 0);
    }
}

void World::applyForce(int id, const Vec2& f)
{
    accelerations[id] += f * invMass[id];
}

void World::updateVelocities(float dt)
{
    for(int id = 0; id < entityCount; id++) {
        if(active[id] == 1)
            velocities[id] += accelerations[id] * dt;
    }
}

void World::updatePositions(float dt)
{
    for(int id = 0; id < entityCount; id++) {
        if(active[id] == 1)
        {
            positions[id] += velocities[id] * dt;
            updateObject(id, calculateAABB(positions[id], meshIDs[id], scales[id]));
        }
    }
}

void World::resolveCollisions()
{
    for(auto [id1, id2]: collisionPairs)
    {
        CollisionResult res = checkCollision(positions[id1], positions[id2], meshIDs[id1], meshIDs[id2], scales[id1], scales[id2]);
        if(!res.isColliding)
            continue;
        resolve(positions[id1], positions[id2], corrections[id1], corrections[id2], velocities[id1], velocities[id2], invMass[id1], invMass[id2], res.normal, res.penetrationDepth, RES, CORR);
    }
}

void World::resolveBorder() {

    for(int id = 0; id < entityCount; id++)
    {
        if(active[id] != 1)
            continue;

        AABB& aabb = aabbs[id];
        Vec2& correction = corrections[id];
        Vec2& velocity = velocities[id];

        if (aabb.min.x < border.min.x) {
            correction.x += (border.min.x - aabb.min.x) * CORR; 
            velocity.x *= -RES;
        }

        if (aabb.max.x > border.max.x) {
            correction.x -= (aabb.max.x - border.max.x) * CORR;
            velocity.x *= -RES;
        }

        if (aabb.min.y < border.min.y) {
            correction.y += (border.min.y - aabb.min.y) * CORR;
            velocity.y *= -RES;
        }

        if (aabb.max.y > border.max.y) {
            correction.y -= (aabb.max.y - border.max.y) * CORR;
            velocity.y *= -RES;
        }
    }
}

void World::deleteObject(int id) {
    if (id >= entityCount || id < 0) return;


    remove(id, prev[id]);
    
    freeList.push_back(id);
    active[id] = 0;
}

// Computes hash index from 2D coordinates
int World::getCellIndex(int x, int y) const {
    return x * 73856093 ^ y * 19349663;
}

// Inserts an ID into all relevant cells
void World::insert(int id, std::array<int, 4> coord) {
    for (int x = coord[0]; x <= coord[2]; ++x) {
        for (int y = coord[1]; y <= coord[3]; ++y) {
            int cellIndex = getCellIndex(x, y);
            grid[cellIndex].push_back(id);
        }
    }
}

// Removes an ID from all relevant cells
void World::remove(int id, std::array<int, 4> coord) {
    for (int x = coord[0]; x <= coord[2]; ++x) {
        for (int y = coord[1]; y <= coord[3]; ++y) {
            int cellIndex = getCellIndex(x, y);
            auto& cellVector = grid[cellIndex];

            auto it = std::find(cellVector.begin(), cellVector.end(), id);
            if (it != cellVector.end()) {
                std::swap(*it, cellVector.back());
                cellVector.pop_back();
                if(cellVector.empty())
                    grid.erase(cellIndex);
            }
        }
    }
}

// Updates an object's position in the grid
void World::updateObject(int id, const AABB& newAABB) {
    int minX = static_cast<int>(newAABB.min.x / cellSize + offset);
    int minY = static_cast<int>(newAABB.min.y / cellSize + offset);
    int maxX = static_cast<int>(newAABB.max.x / cellSize + offset);
    int maxY = static_cast<int>(newAABB.max.y / cellSize + offset);

    std::array<int, 4> next = {minX, minY, maxX, maxY};

    if (next != prev[id]) {
        if (prev[id][2] >= prev[id][0]) {
            remove(id, prev[id]);
        }
        insert(id, next);

        prev[id] = next;
    }

    aabbs[id] = newAABB;
}

void World::genCollisionPairs() {
    collisionPairs.clear();

    for (const auto& [cellIndex, cellObjects] : grid) {
        int size = cellObjects.size();

        for (int i = 0; i < size; ++i) {
            int id1 = cellObjects[i];
            const AABB& aabb1 = aabbs[id1];

            for (int j = i + 1; j < size; ++j) {
                int id2 = cellObjects[j];
                const AABB& aabb2 = aabbs[id2];

                if (aabb1.overlaps(aabb2)) { 
                    collisionPairs.emplace_back(std::min(id1, id2), std::max(id1, id2));
                }
            }
        }
    }

    std::sort(collisionPairs.begin(), collisionPairs.end());

    auto last = std::unique(collisionPairs.begin(), collisionPairs.end());
    collisionPairs.erase(last, collisionPairs.end());
}