#pragma once
#include <vector>
#include <array>
#include <unordered_map>
#include "math/Vec2.hpp"
#include "structures/AABB.hpp"
#include "Mesh.hpp"
#include "Collision.hpp"

struct World {

    float cellSize;
    AABB border;
    int entityCount;
    float RES, CORR;

    std::vector<int> freeList;

    std::vector<Vec2> positions;
    std::vector<Vec2> corrections;
    std::vector<Vec2> velocities;
    std::vector<Vec2> accelerations;
    std::vector<float> invMass;
    std::vector<float> scales;
    std::vector<AABB> aabbs;
    std::vector<int> meshIDs;
    std::vector<int> active;
    std::vector<std::array<int, 4>> prev; 

    std::unordered_map<int, std::vector<int>> grid;
    std::vector<std::pair<int, int>> collisionPairs;

    World(float cellSize, AABB b, float r, float c);
    int addObject(const Vec2& pos, int meshID);
    int addObject(const Vec2& pos, const Vec2& vel, int meshID, float mass, float scale);
    void deleteObject(int id);
    void insert(int id, std::array<int, 4> coord);
    void remove(int id, std::array<int, 4> coord);
    void updateObject(int id, const AABB& newAABB);
    int getCellIndex(int x, int y) const;
    void genCollisionPairs();
    AABB calculateAABB(const Vec2& position, int meshID, float scale) const;
    void resetForces(const Vec2& g);
    void applyForce(int id, const Vec2& f);
    void updateVelocities(float dt);
    void updatePositions(float dt);
    void resolveBorder();
    void resolveCollisions();
    void applyCorrections();;
};
