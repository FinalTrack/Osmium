#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <cassert>
#include "math/Vec2.hpp"

struct Mesh {
    std::vector<Vec2> points;       // Points defining the polygon in counter-clockwise order
    std::vector<Vec2> normals;      

    Mesh(const std::vector<Vec2>& points) : points(points) {
        calculateNormals();
    }

    void calculateNormals() {
        normals.clear();
        size_t n = points.size();

        for (size_t i = 0; i < n; ++i) {
            Vec2 p1 = points[i];
            Vec2 p2 = points[(i + 1) % n];

            Vec2 edge = p2 - p1;
            Vec2 normal = Vec2(edge.y, -edge.x).normalized(); 
            normals.push_back(normal);
        }
    }
};

inline std::vector<Mesh> meshes; 
inline int addMesh(const std::vector<Vec2>& points) {
    meshes.emplace_back(points);
    return meshes.size() - 1;  
}
