#ifndef RAYTRACER_VEC3_H
#define RAYTRACER_VEC3_H

#include <cmath>
#include <iostream>
#include <algorithm>

using std::sqrt;

class Vec3 {
public:
    double e[3];

    Vec3() : e{0, 0, 0} {
    }

    Vec3(double e0, double e1, double e2) : e{e0, e1, e2} {
    }

    [[nodiscard]] double x() const { return e[0]; }

    [[nodiscard]] double y() const { return e[1]; }

    [[nodiscard]] double z() const { return e[2]; }

    Vec3 operator-() const { return Vec3(-e[0], -e[1], -e[2]); }

    double operator[](int i) const { return e[i]; }

    double &operator[](int i) { return e[i]; }

    Vec3 &operator+=(const Vec3 &v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    Vec3 &operator*=(double t) {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    Vec3 &operator/=(double t) {
        return *this *= 1 / t;
    }

    Vec3 operator/(const Vec3 &v) const {
        return {e[0] / v.e[0], e[1] / v.e[1], e[2] / v.e[2]};
    }

    [[nodiscard]] double length() const {
        return sqrt(lengthSquared());
    }

    [[nodiscard]] double lengthSquared() const {
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    }
};

// point3 is just an alias for Vec3, but useful for geometric clarity in the code.
using point3 = Vec3;


// Vector Utility Functions

inline std::ostream &operator<<(std::ostream &out, const Vec3 &v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline Vec3 operator+(const Vec3 &u, const Vec3 &v) {
    return {u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]};
}

inline Vec3 operator-(const Vec3 &u, const Vec3 &v) {
    return {u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]};
}

inline Vec3 operator*(const Vec3 &u, const Vec3 &v) {
    return {u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]};
}

inline Vec3 operator*(double t, const Vec3 &v) {
    return {t * v.e[0], t * v.e[1], t * v.e[2]};
}

inline Vec3 operator*(const Vec3 &v, double t) {
    return t * v;
}

inline Vec3 operator/(Vec3 v, double t) {
    return (1 / t) * v;
}

inline double dot(const Vec3 &u, const Vec3 &v) {
    return u.e[0] * v.e[0]
           + u.e[1] * v.e[1]
           + u.e[2] * v.e[2];
}

inline Vec3 cross(const Vec3 &u, const Vec3 &v) {
    return {u.e[1] * v.e[2] - u.e[2] * v.e[1],
            u.e[2] * v.e[0] - u.e[0] * v.e[2],
            u.e[0] * v.e[1] - u.e[1] * v.e[0]};
}

inline Vec3 unitVector(Vec3 v) {
    return v / v.length();
}

// Create a perfect mirror reflection
inline Vec3 reflect(const Vec3 &v, const Vec3 &n) {
    return v - 2 * dot(v, n) * n;
}

// Clamp double values to a specific value
inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Clamp vector values to a specific value range
inline Vec3 clamp(const Vec3 &v, double min_val, double max_val) {
    return {
            clamp(v.x(), min_val, max_val),
            clamp(v.y(), min_val, max_val),
            clamp(v.z(), min_val, max_val)
    };
}

double vecMin(const Vec3 &v) {
    return std::min(v.e[0], std::min(v.e[1], v.e[2]));
}

double vecMax(const Vec3 &v) {
    return std::max(v.e[0], std::max(v.e[1], v.e[2]));
}

inline Vec3 min(const Vec3 &a, const Vec3 &b) {
    return {std::min(a.e[0], b.e[0]), std::min(a.e[1], b.e[1]), std::min(a.e[2], b.e[2])};
}

inline Vec3 max(const Vec3 &a, const Vec3 &b) {
    return {std::max(a.e[0], b.e[0]), std::max(a.e[1], b.e[1]), std::max(a.e[2], b.e[2])};
}


#endif //RAYTRACER_VEC3_H
