#pragma once
#include "math/Vec2.hpp"
#include "structures/AABB.hpp"
#include "Mesh.hpp"
#include <limits>
#include <vector>
#include <cmath>
#include <array>


// collide: number of contact points (0 means no collision)
// normal: collision normal pointing from body A to B
// depth: penetration depth along normal
// contact: up to two contact points
struct CollisionResult 
{
    int collide;
    Vec2 normal;
    float depth;
    std::array<Vec2, 2> contact;
};

// Body: rigid-body state and collision helpers.
// meshID == 1000 is treated as a circle using meshdata::RADIUS (special case).
struct Body
{
    Vec2 position;
    Vec2 correction;
    Vec2 velocity;
    Vec2 acceleration;
    float invMass;
    float scale;
    float theta, omega, alpha;
    float cosTheta, sinTheta;
    float invMoI;
    AABB aabb; int ind, level;
    std::vector<Vec2> transformed;
    int meshID;

    float restitution;
    float sFriction, kFriction;
    int active;

    Body(const Vec2& pos, const Vec2& vel, int mid, float imass, float iMoI, float sc, float ang, float res, int act = 1)
    {
        position = pos;
        correction = Vec2(0, 0);
        velocity = vel;
        acceleration = Vec2(0, 0);
        invMass = imass;
        scale = sc;
        theta = ang;
        cosTheta = std::cos(ang), sinTheta = std::sin(ang);
        omega = alpha = 0;
        invMoI = iMoI;
        aabb = AABB();
        ind = -1;
        level = -1;
        meshID = mid;
        active = act;
        restitution = res;
        sFriction = 0.3;
        kFriction = 0.2;
    }

    // Fills transformed with mesh vertex positions rotated by theta, scaled and translated to position.
    void transform()
    {
        transformed.clear();
        std::vector<Vec2>& points = meshdata::meshes[meshID].points;
        for(Vec2& point: points)
            transformed.push_back(Vec2::rotate(point * scale, cosTheta, sinTheta) + position);
    }

    // Computes axis-aligned bounding box for the body, by calling transform.
    void calculateAABB()
    {
        if (meshID == 1000) 
        { 
            const float radius = meshdata::RADIUS * scale; 
    
            Vec2 minPos = position - Vec2(radius, radius);
            Vec2 maxPos = position + Vec2(radius, radius);
    
            aabb = AABB(minPos, maxPos);
            return;
        }
    
        transform();
        Vec2 minPos = position;
        Vec2 maxPos = position;
    
        for (const Vec2& tp : transformed) 
        {
            minPos = Vec2::min(minPos, tp);
            maxPos = Vec2::max(maxPos, tp);
        }
    
        aabb = AABB(minPos, maxPos);
    }

    //Projects the transformed polygon onto `axis` and returns min/max projection.
    void projectOntoAxis(const Vec2& axis, float& mn, float& mx) const {
        mn = std::numeric_limits<float>::infinity();
        mx = -std::numeric_limits<float>::infinity();
    
        for (const Vec2& tp : transformed) 
        {
            float project = Vec2::dot(tp, axis);
            mn = std::min(project, mn);
            mx = std::max(project, mx);
        }
    }

    // Point-in-polygon test for transformed polygon
    bool contains(Vec2 point) const
    {
        if(meshID == 1000)
        {
            Vec2 dist = point - position;
            float r = meshdata::RADIUS * scale;
            return Vec2::dot(dist, dist) <= r*r;
        }

        std::vector<Vec2>& norms = meshdata::meshes[meshID].normals;
        for(int i = 0; i < transformed.size(); i++)
        {
            Vec2 norm = Vec2::rotate(norms[i], cosTheta, sinTheta);
            if(Vec2::dot(point, norm) > Vec2::dot(transformed[i], norm))
                return false;
        }
        return true;
    }

    // circle circle SAT collision check
    static CollisionResult circleCircle(const Body& b1, const Body& b2)
    {
        Vec2 distVec = b2.position - b1.position;
        float dsqr = Vec2::dot(distVec, distVec);
        float rsum = meshdata::RADIUS * (b1.scale + b2.scale);
        float rsqr = rsum * rsum;

        if(dsqr > rsqr)
        {
            return {0}; 
        }

        float d = std::sqrt(dsqr);
        Vec2 normal = distVec.normalized();
        float depth = rsum - d;

        CollisionResult res = {1, normal, depth};
        res.contact[0] = b1.position + normal * meshdata::RADIUS * b1.scale;
        return res;
    } 
    
    // circle polygon SAT collision check
    static CollisionResult circlePoly(const Body& b1, const Body& b2)
    {
        float minOverlap = std::numeric_limits<float>::infinity();
        Vec2 normal;
        int poly;

        std::vector<Vec2>& normals = meshdata::meshes[b1.meshID].normals;

        for (const Vec2& norm : normals) {
            Vec2 rnorm = Vec2::rotate(norm, b1.cosTheta, b1.sinTheta);
            float min1, max1, min2, max2;
            b1.projectOntoAxis(rnorm, min1, max1);
            float center = Vec2::dot(b2.position, rnorm);

            min2 = center - meshdata::RADIUS * b2.scale;
            max2 = center + meshdata::RADIUS * b2.scale;

            float overlap = max1 - min2;

            if (overlap <= 0) {
                return {0}; 
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                normal = rnorm;
                poly = 1;
            }
        }

        for (const Vec2& tp : b1.transformed) {
            float min1, max1, min2, max2;
            Vec2 norm = (tp - b2.position).normalized();
            
            b1.projectOntoAxis(norm, min1, max1);
            float center = Vec2::dot(b2.position, norm);

            min2 = center - meshdata::RADIUS * b2.scale;
            max2 = center + meshdata::RADIUS * b2.scale;

            float overlap = max2 - min1;

            if (overlap <= 0) {
                return {0};
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                normal = norm;
                poly = 2;
            }
        }

        CollisionResult res = {1, normal, minOverlap};
        if(poly == 1)
            res.contact[0] = b2.position - normal * meshdata::RADIUS * b2.scale;
        else
            res.contact[0] = b2.position + normal * meshdata::RADIUS * b2.scale;
        
        return res;
    }

    // Sutherland–Hodgman style clipping for a segment against a half-space (n·x >= c).
    // `pts` should contain two points (segment endpoints). After call it may contain 0,1,2 points.
    static void clip(std::vector<Vec2>& pts, const Vec2& n, float c) {
        Vec2 A = pts[0];
        Vec2 B = pts[1];
    
        float dA = Vec2::dot(A, n) - c;
        float dB = Vec2::dot(B, n) - c;
    
        Vec2 out0, out1;
        int count = 0;
  
        if (dA >= 0.0f) {
            out0 = A;
            count = 1;
        }

        if (dA * dB < 0.0f) {
            float t = dA / (dA - dB);
            Vec2  I = A + (B - A) * t;
            if (count == 0) {
                out0 = I;
                count = 1;
            } else {
                out1 = I;
                count = 2;
            }
        }

        if (dB >= 0.0f) {
            if (count == 0) {
                out0 = B;
                count = 1;
            } else {
                out1 = B;
                count = 2;
            }
        }

        pts.clear();
        if (count >= 1) pts.push_back(out0);
        if (count == 2) pts.push_back(out1);
    }

    // polygon polygon SAT collision check
    static CollisionResult polyPoly(const Body& b1, const Body& b2)
    {
        float minOverlap = std::numeric_limits<float>::infinity();
        Vec2 normal;
        int poly, rid;

        int N1 = b1.transformed.size();
        int N2 = b2.transformed.size();

        int id = 0;
        for (const Vec2& norm : meshdata::meshes[b1.meshID].normals) {
            Vec2 rnorm = Vec2::rotate(norm, b1.cosTheta, b1.sinTheta);
            float min1, max1, min2, max2;

            b1.projectOntoAxis(rnorm, min1, max1);
            b2.projectOntoAxis(rnorm, min2, max2);
            float overlap = max1 - min2;
    
            if (overlap <= 0) {
                return {0};
            }
    
            if (overlap < minOverlap) {
                minOverlap = overlap;
                normal = rnorm;
                rid = id;
                poly = 1;
            }

            id++;
        }

        id = 0;
        for (const Vec2& norm : meshdata::meshes[b2.meshID].normals) {
            Vec2 rnorm = Vec2::rotate(norm, b2.cosTheta, b2.sinTheta);
            float min1, max1, min2, max2;

            b1.projectOntoAxis(rnorm, min1, max1);
            b2.projectOntoAxis(rnorm, min2, max2);
            float overlap = max2 - min1;
    
            if (overlap <= 0) {
                return {0};
            }
    
            if (overlap < minOverlap) {
                minOverlap = overlap;
                normal = rnorm;
                rid = id;
                poly = 2;
            }

            id++;
        }

        Vec2 r1, r2, i1, i2;

        if(poly == 1)
        {
            r1 = b1.transformed[rid];
            r2 = b1.transformed[(rid + 1) % N1];

            float anti = std::numeric_limits<float>::infinity();
            int iid;

            std::vector<Vec2>& antiNorms = meshdata::meshes[b2.meshID].normals;

            for(int i = 0; i < N2; i++)
            {
                Vec2 anorm = Vec2::rotate(antiNorms[i], b2.cosTheta, b2.sinTheta);
                float dot = Vec2::dot(normal, anorm);
                if(dot < anti)
                {
                    anti = dot;
                    iid = i;
                }
            }

            i1 = b2.transformed[iid];
            i2 = b2.transformed[(iid + 1) % N2];
        }
        else
        {
            r1 = b2.transformed[rid];
            r2 = b2.transformed[(rid + 1) % N2];

            float anti = std::numeric_limits<float>::infinity();
            int iid;

            std::vector<Vec2>& antiNorms = meshdata::meshes[b1.meshID].normals;

            for(int i = 0; i < N1; i++)
            {
                Vec2 anorm = Vec2::rotate(antiNorms[i], b1.cosTheta, b1.sinTheta);
                float dot = Vec2::dot(normal, anorm);
                if(dot < anti)
                {
                    anti = dot;
                    iid = i;
                }
            }

            i1 = b1.transformed[iid];
            i2 = b1.transformed[(iid + 1) % N1];
        }

        Vec2 tangent = Vec2(-normal.y, normal.x);
        std::vector<Vec2> pts = {i1, i2};
        clip(pts,  tangent, Vec2::dot(tangent, r1));
        clip(pts, -tangent, -Vec2::dot(tangent, r2));

        float rd = Vec2::dot(r1, normal);
        CollisionResult res;

        int cnt = 0;
        for(int i = 0; i < pts.size(); i++)
        {
            float depth = rd - Vec2::dot(pts[i], normal);
            if(depth > 0)
            {
                res.contact[cnt] = pts[i];
                cnt++;
            }
        }

        res.collide = cnt;
        res.normal = normal;
        res.depth = minOverlap;
        return res;
    }

    // Dispatches to the correct SAT helper based on meshID.
    // Ensures returned normal is oriented from b1 -> b2.
    static CollisionResult performSAT(const Body& b1, const Body& b2)
    {
        CollisionResult res;
        if(b1.meshID == 1000 && b2.meshID == 1000)
            res = circleCircle(b1, b2);
        else if(b1.meshID == 1000)
            res = circlePoly(b2, b1);
        else if(b2.meshID == 1000)
            res = circlePoly(b1, b2);
        else
            res = polyPoly(b1, b2);
        
        if(!res.collide)
            return res;
        
        if(Vec2::dot(b2.position - b1.position, res.normal) < 0) 
            res.normal = -res.normal;
        
        return res;
    }

    // Applies normal impulse and friction impulse to velocities and angular velocities.
    // Uses Baumgarte-like positional correction with `corrFactor` and `slop`.
    static void resolve(Body& b1, Body& b2, const CollisionResult& res, float corrFactor = 0.40f, float slop = 0.05f)
    {   
        Vec2 corr = res.normal * corrFactor * (std::max(res.depth - slop, 0.0f) / (b1.invMass + b2.invMass));
        b1.correction -= corr * b1.invMass;
        b2.correction += corr * b2.invMass;

        for(int i = 0; i < res.collide; i++)
        {
            Vec2 r1 = res.contact[i] - b1.position;
            Vec2 r2 = res.contact[i] - b2.position;

            Vec2 v1 = b1.velocity + Vec2(-r1.y, r1.x) * b1.omega;
            Vec2 v2 = b2.velocity + Vec2(-r2.y, r2.x) * b2.omega;

            Vec2 rVel = v2 - v1;
            float velNorm = Vec2::dot(rVel, res.normal);
            if(velNorm >= 0)
                continue;
            
            float iMag = -(1.0f + std::min(b1.restitution, b2.restitution)) * velNorm;

            float c1 = Vec2::cross(r1, res.normal);
            float c2 = Vec2::cross(r2, res.normal);

            iMag /= (b1.invMass + b2.invMass + b1.invMoI * c1*c1 + b2.invMoI * c2*c2);
            Vec2 impulse = res.normal * iMag;

            float mus = std::sqrt(b1.sFriction * b2.sFriction);
            float muk = std::sqrt(b1.kFriction * b2.kFriction);

            float fS = iMag * mus;
            float fK = iMag * muk;
            Vec2 tangent = Vec2(-res.normal.y, res.normal.x);
            float velTang = Vec2::dot(rVel, tangent);

            float t1 = Vec2::cross(r1, tangent);
            float t2 = Vec2::cross(r2, tangent);
            float fMag = -velTang / (b1.invMass + b2.invMass + b1.invMoI * t1*t1 + b2.invMoI * t2*t2);
            
            if(fMag > fS)
                fMag = fK;
            else if(fMag < -fS)
                fMag = -fK;

            impulse += tangent * fMag;

            b1.velocity -= impulse * b1.invMass;
            b2.velocity += impulse * b2.invMass;

            b1.omega -= b1.invMoI * Vec2::cross(r1, impulse);
            b2.omega += b2.invMoI * Vec2::cross(r2, impulse);
        }
    }
};

