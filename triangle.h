#ifndef RAYTRACER_TRIANGLE_H
#define RAYTRACER_TRIANGLE_H

#include "rtweekend.h"
#include "quad.h"

#include <cmath>

class triangle : public quad {
public:
    using quad::quad;

    virtual bool is_interior(double a, double b, hit_record &rec) const {
        // Return true if point is inside the triangle, return false if not

        auto triangleTest = a + b;

        if ((a < 0) || (b < 0)) {
            return false;
        } else if (triangleTest > 1) {
            return false;
        }

        return true;
    }
};


#endif //RAYTRACER_TRIANGLE_H
