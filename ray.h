#ifndef RAY_H
#define RAY_H

#include "vec3.h"

// Simple ray class
struct Ray {
    vec3 origin, direction;
    double t = std::numeric_limits<double>::max();

    Ray(const vec3&origin, const vec3&direction) : origin(origin), direction(unit_vector(direction)) {
    }
};

#endif //RAY_H
