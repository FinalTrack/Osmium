#include "math/Vec2.hpp"
#include "Mesh.hpp"
#include <limits>
#include <vector>
#include <cmath>

struct CollisionResult {
    bool isColliding;
    Vec2 normal;
    float penetrationDepth;
};

inline void projectOntoAxis(const std::vector<Vec2>& points, const Vec2& axis, float& min, float& max, const Vec2& position, float pScale) {
    min = std::numeric_limits<float>::infinity();
    max = -std::numeric_limits<float>::infinity();

    for (const Vec2& point : points) {
        float projection = Vec2::dot(position + point * pScale, axis);
        if (projection < min) min = projection;
        if (projection > max) max = projection;
    }
}

inline CollisionResult checkCircleCircle(const Vec2& pos1, const Vec2& pos2, float radius1, float radius2) {
    Vec2 distanceVec = pos2 - pos1;
    float distanceSq = Vec2::dot(distanceVec, distanceVec);
    float combinedRadius = radius1 + radius2;
    float combinedRadiusSq = combinedRadius * combinedRadius;

    if (distanceSq > combinedRadiusSq) {
        return {false, Vec2(0, 0), 0.0f}; 
    }

    float distance = std::sqrt(distanceSq);
    Vec2 normal = distanceVec.normalized();
    float penetrationDepth = combinedRadius - distance;

    return {true, normal, penetrationDepth};
}

inline CollisionResult checkCirclePolygon(const Vec2& circlePos, float radius, const Vec2& polyPos, const Mesh& polygon, float pScale) {
    float minOverlap = std::numeric_limits<float>::infinity();
    Vec2 collisionNormal;

    for (const Vec2& normal : polygon.normals) {
        float min1, max1, min2, max2;
        projectOntoAxis(polygon.points, normal, min1, max1, polyPos, pScale);
        float centerProjection = Vec2::dot(circlePos, normal);

        min2 = centerProjection - radius;
        max2 = centerProjection + radius;

        float overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap <= 0) {
            return {false, Vec2(0, 0), 0.0f}; 
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = normal;
        }
    }

    for (const Vec2& point : polygon.points) {
        float min1, max1, min2, max2;
        Vec2 vertexPos = polyPos + point * pScale;
        Vec2 normal = (circlePos - vertexPos).normalized();
        projectOntoAxis(polygon.points, normal, min1, max1, polyPos, pScale);
        float centerProjection = Vec2::dot(circlePos, normal);

        min2 = centerProjection - radius;
        max2 = centerProjection + radius;

        float overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap <= 0) {
            return {false, Vec2(0, 0), 0.0f}; 
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = normal;
        }
    }

    return {true, collisionNormal, minOverlap};
}

inline CollisionResult checkCollision(const Vec2& pos1, const Vec2& pos2, int meshID1, int meshID2, float scale1, float scale2) {

    if(meshID1 == 1000 && meshID2 == 1000)
        return checkCircleCircle(pos1, pos2, 10.0f * scale1, 10.0f * scale2);
    if(meshID2 == 1000)
    {
        const Mesh& polygon = meshes[meshID1];
        CollisionResult res = checkCirclePolygon(pos2, 10.0f * scale2, pos1, polygon, scale1);
        if (Vec2::dot(pos2 - pos1, res.normal) < 0) {
            res.normal = -res.normal;
        }
        return res;
    }
    if(meshID1 == 1000)
    {
        const Mesh& polygon = meshes[meshID2];
        CollisionResult res = checkCirclePolygon(pos1, 10.0f * scale1, pos2, polygon, scale2);
        if (Vec2::dot(pos2 - pos1, res.normal) < 0) {
            res.normal = -res.normal;
        }
        return res;
    }

    const Mesh& mesh1 = meshes[meshID1];
    const Mesh& mesh2 = meshes[meshID2];

    float minOverlap = std::numeric_limits<float>::infinity();
    Vec2 collisionNormal;

    for (const Vec2& normal : mesh1.normals) {
        float min1, max1, min2, max2;
        projectOntoAxis(mesh1.points, normal, min1, max1, pos1, scale1);
        projectOntoAxis(mesh2.points, normal, min2, max2, pos2, scale2);

        float overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap <= 0) {
            return {false, Vec2(0, 0), 0.0f};
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = normal;
        }
    }

    for (const Vec2& normal : mesh2.normals) {
        float min1, max1, min2, max2;
        projectOntoAxis(mesh1.points, normal, min1, max1, pos1, scale1);
        projectOntoAxis(mesh2.points, normal, min2, max2, pos2, scale2);

        float overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap <= 0) {
            return {false, Vec2(0, 0), 0.0f};
        }

        if (overlap < minOverlap) {
            minOverlap = overlap;
            collisionNormal = normal;
        }
    }

    if (Vec2::dot(pos2 - pos1, collisionNormal) < 0) {
        collisionNormal = -collisionNormal;
    }

    return {true, collisionNormal, minOverlap};
}

inline void resolve(Vec2& pos1, Vec2& pos2, Vec2& c1, Vec2& c2, Vec2& vel1, Vec2& vel2, float invMass1, float invMass2, const Vec2& normal, float penetrationDepth, float restitution = 0.8f, float correctionFactor = 0.2f) {
    
    Vec2 correction = normal * penetrationDepth * correctionFactor;
    c1 -= correction * invMass1;
    c2 += correction * invMass2;
    
    Vec2 relativeVelocity = vel2 - vel1;

    float velAlongNormal = Vec2::dot(relativeVelocity, normal);

    if (velAlongNormal > 0) return;

    float impulseMagnitude = -(1.0f + restitution) * velAlongNormal;
    impulseMagnitude /= (invMass1 + invMass2);

    Vec2 impulse = normal * impulseMagnitude;
    vel1 -= impulse * invMass1;
    vel2 += impulse * invMass2;
}

