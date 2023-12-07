#ifndef MATERIAL_H
#define MATERIAL_H

#include "color.h"
#include "light.h"

class Material {
public:
    virtual ~Material() = default;

    [[nodiscard]] virtual color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                                      const vec3&normal,
                                      const color&object_color) const = 0;
};

class PhongMaterial final : public Material {
public:
    explicit PhongMaterial(const double _shininess) : shininess(_shininess) {
    }

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color&object_color) const override {
        // Calculate the normal of the object
        const vec3 viewer_direction = unit_vector(hit_point - ray.origin);
        const vec3 light_direction = unit_vector(light->origin - hit_point);
        const color white = color(1, 1, 1);

        //Diffuse
        const double diffuse_component = std::max(0.0, dot(light_direction, normal));

        //Specular
        const vec3 reflection_vector = unit_vector(normal * diffuse_component * 2 - light_direction);
        const double specular_component = std::pow(std::max(dot(viewer_direction, -reflection_vector), 0.0), shininess);

        //Ambient
        const color ambient_color = object_color * 0.5;

        //Phong Formula
        const vec3 ambient_reflection = 1 / M_PI * ambient_color;
        const vec3 surface_illumination = light->light_color * std::max(0.0, dot(light_direction, normal)) * light->
                                          intensity;
        const vec3 diffuse_reflection = 1 / M_PI * object_color * light->intensity;
        const vec3 specular_reflection = white * specular_component * light->intensity;

        // Might be possible to do multiple light sources
        const vec3 phong_shade = ambient_reflection + surface_illumination * (
                                     diffuse_reflection + specular_reflection);

        // Calculate the min and max value so that we do not overshoot and get a negative color value
        // -> Gets black or other color which we do not want to have
        return clamp(phong_shade, 0, 1);
    }

private:
    double shininess;
};

class LambertianMaterial final : public Material {
public:
    ~LambertianMaterial() override = default;

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color&object_color) const override {
        const vec3 light_direction = unit_vector(light->origin - hit_point);

        const auto diffuse_component = light->light_color * std::max(0.0, dot(normal, light_direction));
        const auto diffuse_reflection = 1 / M_PI * object_color * light->intensity;

        return clamp(diffuse_reflection * diffuse_component, 0, 1);
    }
};

class CheckeredMaterial final : public Material {
public:
    explicit CheckeredMaterial(const double _checkered_size, const color&_color1,
                               const color&_color2) : size(_checkered_size), color1(_color1), color2(_color2) {
    }

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color&object_color) const override {
        const int square_x_coord = static_cast<int>(floor(hit_point.x() / size)); // Use floor to round downward
        const int square_y_coord = static_cast<int>(floor(hit_point.y() / size));
        const int square_z_coord = static_cast<int>(floor(hit_point.z() / size));

        color final_color_1 = clamp(color1 * light->intensity, 0, 1);
        color final_color_2 = clamp(color2 * light->intensity, 0, 1);
        // Calculate both of the cell values -> Even number C1, Odd number C2
        if (normal.x() == 0 && normal.z() == 0)
            return (square_x_coord + square_z_coord) % 2 == 0
                       ? final_color_1
                       : final_color_2;
        if (normal.y() == 0 && normal.z() == 0)
            return (square_z_coord + square_y_coord) % 2 == 0
                       ? final_color_1
                       : final_color_2;
        return (square_x_coord + square_y_coord) % 2 == 0 ? final_color_1 : final_color_2;
    }

private:
    double size;
    color color1;
    color color2;
};

class Mirror final : public Material {
public:
    vec3 albedo;

    explicit Mirror(const vec3&_albedo) : albedo(_albedo) {
    }

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color&object_color) const override {
        // Return background because mirror effect algorithm is insde the trace function
        return {0.7, 0.8, 1.0};
    }
};

#endif //MATERIAL_H
