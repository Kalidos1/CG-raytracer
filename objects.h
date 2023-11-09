#ifndef RAYTRACER_OBJECTS_H
#define RAYTRACER_OBJECTS_H

#include "hittable_list.h"
#include "quad.h"
#include "triangle.h"

inline shared_ptr<hittable_list> cube(const point3 &a, const point3 &b, shared_ptr<material> mat) {
    auto sides = make_shared<hittable_list>();

    // Construct the two opposite vertices with the minimum and maximum coordinates.
    auto min = point3(fmin(a.x(), b.x()), fmin(a.y(), b.y()), fmin(a.z(), b.z()));
    auto max = point3(fmax(a.x(), b.x()), fmax(a.y(), b.y()), fmax(a.z(), b.z()));

    auto dx = vec3(max.x() - min.x(), 0, 0);
    auto dy = vec3(0, max.y() - min.y(), 0);
    auto dz = vec3(0, 0, max.z() - min.z());

    sides->add(make_shared<quad>(point3(min.x(), min.y(), max.z()), dx, dy, mat)); // front
    sides->add(make_shared<quad>(point3(max.x(), min.y(), max.z()), -dz, dy, mat)); // right
    sides->add(make_shared<quad>(point3(max.x(), min.y(), min.z()), -dx, dy, mat)); // back
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dz, dy, mat)); // left
    sides->add(make_shared<quad>(point3(min.x(), max.y(), max.z()), dx, -dz, mat)); // top
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dx, dz, mat)); // bottom

    return sides;
}

// Might be difficult to rotate - need to check
inline shared_ptr<hittable_list>
pyramid(const point3 &a, const point3 &b, const double height, shared_ptr<material> mat) {
    auto sides = make_shared<hittable_list>();

    // Construct the base quad
    auto dx = vec3(b.x() - a.x(), 0, 0);
    auto dz = vec3(0, 0, b.z() - a.z());
    auto pyramid_scale = height / 2;
    sides->add(make_shared<quad>(point3(a.x(), a.y(), a.z()), dx, dz, mat)); // bottom

    auto quad_base = vec3(-pyramid_scale, height, -pyramid_scale);
    auto quad_z = vec3(-pyramid_scale, height, pyramid_scale);
    auto quad_x = vec3(pyramid_scale, height, -pyramid_scale);

    // Construct the pyramid sides
    sides->add(make_shared<triangle>(point3(a.x(), a.y(), a.z()), quad_base, dx, mat));
    sides->add(make_shared<triangle>(point3(a.x(), a.y(), a.z()), quad_base, dz, mat));
    sides->add(make_shared<triangle>(point3(a.x(), a.y(), b.z()), quad_z, dx, mat));
    sides->add(make_shared<triangle>(point3(b.x(), a.y(), a.z()), quad_x, dz, mat));

    return sides;
}

#endif //RAYTRACER_OBJECTS_H
