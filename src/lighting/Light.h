#ifndef LIGHT_H
#define LIGHT_H

#include "../core/Mat3.h"
#include "geometry/Hittable.h"

class Light {
public:
    std::vector<std::shared_ptr<Hittable>> lights;

    Light(const std::vector<std::shared_ptr<Hittable>> &_lights) : lights(_lights) {
    }
};


#endif //LIGHT_H
