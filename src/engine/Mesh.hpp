#pragma once
#include <vector>
#include <cmath>
#include <cassert>
#include "math/Vec2.hpp"

// Represents a simple 2D convex polygon mesh.
// Points must be in counter-clockwise order.
// Automatically recenters points around the centroid.
// Computes outward-facing edge normals.
struct Mesh {
    std::vector<Vec2> points;
    std::vector<Vec2> normals;      

    Mesh(const std::vector<Vec2> pnts)
    {
        points = pnts;
        int n = points.size();
        Vec2 avg(0, 0);
        for(Vec2 p: points)
            avg += p;
        avg = avg / (float)n;
        for(Vec2& p: points)
            p -= avg;
        calculateNormals();
    }

    void calculateNormals() {
        normals.clear();
        int n = points.size();

        for (int i = 0; i < n; ++i) {
            Vec2 p1 = points[i];
            Vec2 p2 = points[(i + 1) % n];

            Vec2 edge = p2 - p1;
            Vec2 normal = Vec2(edge.y, -edge.x).normalized(); 
            normals.push_back(normal);
        }
    }
}; 


// Global container for all meshes.
// Used as a registry for quick access.
namespace meshdata
{
    inline std::vector<Mesh> meshes;
    inline const float RADIUS = 10.0f;
    inline int addMesh(const std::vector<Vec2>& pts) {
        meshes.emplace_back(pts);
        return meshes.size() - 1;
    }
}
