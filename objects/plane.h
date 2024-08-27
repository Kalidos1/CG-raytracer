#ifndef FLOOR_H
#define FLOOR_H

#include "hittable.h"
#include "mat3.h"

class Plane : public Hittable {
public:
    // Plane is defined by position and normal
    Plane(const vec3 &_position, const vec3 &_normal, const color &_color,
          const std::shared_ptr <Material> &material) : position(_position),
                                                        normal(_normal),
                                                        Hittable(_color, material) {
    }

    bool intersect(const Ray &ray, double &t) const override {
        // Calculate the relation of the ray to the plane normal
        const double denom = dot(normal, ray.direction);
        // Ray is parallel to the plane -> Return false
        // Fabs so that it does not matter from which direction we come from
        if (fabs(denom) > 1e-6) {
            // Calculate the vector form ray origin to the plane origin
            const vec3 rayOriginToPlaneHit = position - ray.origin;
            // Calculate the position of the intersection point with the plane
            t = dot(rayOriginToPlaneHit, normal) / denom;
            if (t >= 0) return true;
        }
        return false;
    }

    // Return the constant normal vector of plane
    [[nodiscard]] vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const override {
        return unitVector(normal);
    }

    // Return the constant position vector of plane
    [[nodiscard]] point3 calculateCenter() const override {
        return position;
    }

    // No model transform for now since used for static floor
    void applyModelTransform(const vec3 &translation, const vec3 &rotation, const vec3 &shear, double angle,
                             const point3 &objectCenter) override {
    }

    // View transform since we want simulate camera movement
    void applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle, const point3 &cam) override {
        //Create the rotation matrix and the inverse
        const mat3 R = mat3::rotationMatrix(rotation, degreesToRadians(angle));
        inverse_transformation_matrix = mat3::rotationMatrix(rotation, degreesToRadians(-angle));

        // Apply rotation
        normal = R * normal;

        // Apply translation
        applyTranslation(-translation);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    point3 transform_inverse(const point3 &world_point) const {
        return inverse_transformation_matrix * world_point;
    }

    [[nodiscard]] BoundingBox getBoundingBox() const override {
        // Return an invalid bounding box
        return {vec3(-INFINITY, -INFINITY, -INFINITY), vec3(INFINITY, INFINITY, INFINITY)};
    }

private:
    vec3 position, normal;
    mat3 inverse_transformation_matrix;

    void applyTranslation(const vec3 &translation) {
        position += translation;
    }

    void applyRotation(const vec3 &rotation, const double angle) {
        // Translate plane to origin
        const vec3 plane_origin = position;
        applyTranslation(-plane_origin);

        // Create a inverse rotation matrix to mirror rotation direction
        const mat3 R = mat3::rotationMatrix(rotation, degreesToRadians(angle));
        const mat3 inverse = R.inverse();

        // Apply rotation matrix to the plane normal
        normal = inverse.operator*(normal);

        // Translate back to original position;
        applyTranslation(plane_origin);
    }

    //TODO
    void applyShear(const vec3 &shear) {
    }
};

#endif //FLOOR_H
