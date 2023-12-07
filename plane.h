#ifndef FLOOR_H
#define FLOOR_H
#include "hittable.h"
#include "mat3.h"

class Plane : public Hittable {
public:
    // Plane is defined by position and normal
    Plane(const vec3&_position, const vec3&_normal, const color&_color,
          const std::shared_ptr<Material>&material) : position(_position),
                                                      normal(_normal),
                                                      Hittable(_color, material) {
    }

    bool intersect(const Ray&ray, double&t) const override {
        // Calculate the relation of the ray to the plane normal
        const double denom = dot(normal, ray.direction);
        // Ray is parallel to the plane -> Return false
        // Fabs so that it does not matter from which direction we come from
        if (fabs(denom) > 1e-10) {
            // Calulate the vector form ray origin to the plane origin
            const vec3 rayOriginToPlaneHit = position - ray.origin;
            // Calculate the position of the intersection point with the plane
            t = dot(rayOriginToPlaneHit, normal) / denom;
            if (t >= 0) return true;
        }
        return false;
    }

    // Return the constant normal vector of plane
    [[nodiscard]] vec3 calculate_normal(const point3&hit_point, const Ray&ray) const override {
        return unit_vector(normal);
    }

    // Do not change the floor with the transforms
    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear, double angle) override {
    }

    // Might need to transform position with translation and normal with rotation
    // TODO - Need to check how to do it properly
    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle, const point3&cam) override {
        // Create the rotation matrix and the inverse
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));
        const mat3 inverse = R.inverse();

        normal = inverse * normal;

        position = position + translation;
    }

    double degrees_to_radians(double degrees) {
        return degrees * M_PI / 180.0;
    }

private:
    vec3 position;
    vec3 normal;
};

#endif //FLOOR_H
