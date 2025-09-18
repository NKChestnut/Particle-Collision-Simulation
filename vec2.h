#ifndef VEC2_H
#define VEC2_H

#include <cmath>

/*
1. Purpose
   Lightweight 2D vector with basic operations used in physics updates.

2. Key Ops
   - Addition, subtraction, scalar multiply
   - Dot product
   - Squared norm (avoids unnecessary sqrt)
*/
struct Vec2 {
    double x;
    double y;

    Vec2() : x(0.0), y(0.0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double s)     const { return Vec2(x * s,     y * s);   }

    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double norm2()             const { return x * x + y * y;    }
};

#endif // VEC2_H
