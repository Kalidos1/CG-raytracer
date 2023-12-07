#ifndef BOUNDED_PLANE_H
#define BOUNDED_PLANE_H

class BoundedPlane final : public Hittable {
public:
    BoundedPlane(const vec3&_position, const vec3&_normal, const double _size, const color&_plane_color,
                 const std::shared_ptr<Material>&_material)
        : Hittable(_plane_color, _material), normal(_normal), position(_position), size(_size) {
    }

    bool intersect(const Ray&ray, double&t) const override {
        const double denom = dot(normal, ray.direction);
        if (fabs(denom) > 1e-10) {
            const vec3 rayOriginToPlaneHit = position - ray.origin;
            t = dot(rayOriginToPlaneHit, normal) / denom;

            const vec3 intersection_point = ray.origin + ray.direction * t;

            if (t < 0) return false;

            // Divide by 2 to compare coordinates with respect to the center
            // Transform the absolute coordinates from intersection point to the local coordinate system of the plane
            if (fabs(intersection_point.x() - position.x()) <= size / 2.0 &&
                fabs(intersection_point.y() - position.y()) <= size / 2.0 &&
                fabs(intersection_point.z() - position.z()) <= size / 2.0) {
                return true;
            }
            return false;
        }
        return false;
    }

    // Constant defined normal
    [[nodiscard]] vec3 calculate_normal(const point3&hit_point, const Ray&ray) const override {
        return unit_vector(normal);
    }

    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear,
                               const double angle) override {
    }

    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle,
                              const point3&camera) override {
    }

private:
    vec3 normal;
    point3 position;
    double size;
};

#endif //BOUNDED_PLANE_H
