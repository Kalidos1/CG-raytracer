#ifndef RAYTRACER_DATASTRUCTURE_H
#define RAYTRACER_DATASTRUCTURE_H

#include "../geometry/Hittable.h"

class DataStructure {
public:
    virtual ~DataStructure() = default;

    virtual void build(const std::vector<std::shared_ptr<Hittable>> &hittables) = 0;

    virtual void intersect(Ray &ray, const std::vector<std::shared_ptr<Hittable>> &hittables, int &hitObject) = 0;
};

#endif //RAYTRACER_DATASTRUCTURE_H
