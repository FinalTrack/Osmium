#pragma once
#include <cmath>

struct Vec2 {
    float x = 0.0f, y = 0.0f;

    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
    Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }

    Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
    Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }

    Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    float lengthSquared() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSquared()); }

    Vec2 normalized() const {
        float len = length();
        return (len > 0) ? (*this) / len : Vec2(0, 0);
    }

    static float dot(const Vec2& a, const Vec2& b) {
        return a.x * b.x + a.y * b.y;
    }

    static float cross(const Vec2& a, const Vec2& b) {
        return a.x * b.y - a.y * b.x;
    }

    static Vec2 min(const Vec2& a, const Vec2& b) {
        return Vec2(std::min(a.x, b.x), std::min(a.y, b.y));
    }

    static Vec2 max(const Vec2& a, const Vec2& b) {
        return Vec2(std::max(a.x, b.x), std::max(a.y, b.y));
    }

    static Vec2 rotate(const Vec2& v, float ct, float st) {
        return Vec2(v.x * ct - v.y * st, v.x * st + v.y * ct);
    }
};
