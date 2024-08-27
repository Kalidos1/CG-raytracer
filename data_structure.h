#ifndef RAYTRACER_DATA_STRUCTURE_H
#define RAYTRACER_DATA_STRUCTURE_H

#include "hittable.h"

class DataStructure {
public:
    virtual ~DataStructure() = default;

    virtual void build(const std::vector<std::shared_ptr<Hittable>> &hittables) = 0;

    virtual void intersect(Ray &ray, const std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) = 0;
};

#endif //RAYTRACER_DATA_STRUCTURE_H
