#ifndef RAY_H
#define RAY_H

#include "vec3.h"

struct Ray {
    vec3 origin, direction;

    Ray(const vec3&origin, const vec3&direction) : origin(origin), direction(unit_vector(direction)) {
    }
};

#endif //RAY_H
