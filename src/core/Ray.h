#ifndef RAY_H
#define RAY_H

#include "Vec3.h"

// Simple ray class
struct Ray {
    Vec3 origin, direction;
    double t = std::numeric_limits<double>::max();

    Ray(const Vec3 &origin, const Vec3 &direction) : origin(origin), direction(unitVector(direction)) {
    }
};

#endif //RAY_H
