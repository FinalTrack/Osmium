#pragma once
#include "math/Vec2.hpp"
#include <algorithm>

struct AABB {
    Vec2 min;
    Vec2 max;   

    AABB() = default;
    AABB(const Vec2& tr, const Vec2& bl) : min(tr), max(bl) {}

    bool overlaps(const AABB& other) const {
        return !(max.x < other.min.x || min.x > other.max.x ||
                 max.y < other.min.y || min.y > other.max.y);
    }

    static AABB merge(const AABB& a, const AABB& b) {
        return {
            { std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y) },
            { std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y) }
        };
    }

    float perimeter() const {
        float wx = max.x - min.x;
        float wy = max.y - min.y;
        return 2.0f * (wx + wy);
    }

    bool contains(const AABB& other) const {
        return (min.x <= other.min.x && min.y <= other.min.y &&
                max.x >= other.max.x && max.y >= other.max.y);
    }
};
