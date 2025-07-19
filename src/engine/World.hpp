#pragma once
#include <vector>
#include <array>
#include "math/Vec2.hpp"
#include "structures/AABB.hpp"
#include "Mesh.hpp"
#include "Body.hpp"

inline const int MAXSZ = 1<<18;

struct World 
{
    int length;
    int allocated, activeCount;
    int colCnt;

    std::vector<int> freeList;
    std::vector<Body> bodies;

    std::array<std::vector<int>, MAXSZ> grid; 
    std::vector<int> levels;
    std::vector<int> occ;
    std::vector<std::pair<int, int>> collisionPairs;
    std::vector<CollisionResult> collisionData;

    World(int w, int h);
    int addObject(const Vec2& pos, const Vec2& vel, int meshID, float mass, float MoI, float scale, float ang, float res);
    int addObject(const Vec2& pos, int meshID, float scale, float ang, float res);
    void deleteObject(int id);
    int getIndex(int lvl, int x, int y);
    void getNeighbors(int id, std::vector<std::pair<int, int>>& local);
    void initGrid();
    void resetGrid();
    void updateIndex(int id);
    void genCollisionPairs();
    void resetForces(const Vec2& g);
    void applyForce(int id, const Vec2& f);
    void updateVelocities(float dt);
    void updatePositions(float dt);
    void resolveCollisions();
    void applyCorrections();;
};