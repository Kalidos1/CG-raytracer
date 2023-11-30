#ifndef MATERIAL_H
#define MATERIAL_H

#include "color.h"

class Material {
public:
    virtual ~Material() = default;

    [[nodiscard]] virtual color shade(const color&light_color, const point3&hit_point, const Ray&ray,
                                      const vec3&light_position, const vec3&normal,
                                      const color&object_color) const = 0;
};

class PhongMaterial final : public Material {
public:
    explicit PhongMaterial(const double _shininess) : shininess(_shininess) {
    }

    [[nodiscard]] color shade(const color&light_color, const point3&hit_point, const Ray&ray,
                              const vec3&light_position, const vec3&normal, const color&object_color) const override {
        // Calculate the normal of the object
        const vec3 viewer_direction = unit_vector(hit_point - ray.origin);
        const vec3 light_direction = unit_vector(light_position - hit_point);
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
        const vec3 surface_illumination = light_color * std::max(0.0, dot(light_direction, normal));
        const vec3 diffuse_reflection = 1 / M_PI * object_color;
        const vec3 specular_reflection = white * specular_component;

        // Might be possible to do multiple light sources
        const vec3 phong_shade = ambient_reflection + surface_illumination * (
                                     diffuse_reflection + specular_reflection);

        // Calculate the min and max value so that we do not overshoot and get a negative color value
        // -> Gets black or other color which we do not want to have
        const double phong_red = std::max(0.0, std::min(phong_shade.x(), 1.0));
        const double phong_green = std::max(0.0, std::min(phong_shade.y(), 1.0));
        const double phong_blue = std::max(0.0, std::min(phong_shade.z(), 1.0));

        return {phong_red, phong_green, phong_blue};
    }

private:
    double shininess;
};

class LambertianMaterial final : public Material {
public:
    explicit LambertianMaterial(const vec3&_albedo) : albedo(_albedo) {
    }

    [[nodiscard]] color shade(const color&light_color, const point3&hit_point, const Ray&ray,
                              const vec3&light_position, const vec3&normal, const color&object_color) const override {
        const vec3 light_direction = unit_vector(light_position - hit_point);
        const double diffuse_component = std::max(0.0, dot(normal, light_direction));
        return albedo * diffuse_component * object_color;
    }

private:
    vec3 albedo;
};

class CheckeredMaterial final : public Material {
public:
    explicit CheckeredMaterial(const double _size) : size(_size) {
    }

    [[nodiscard]] color shade(const color&light_color, const point3&hit_point, const Ray&ray,
                              const vec3&light_position, const vec3&normal, const color&object_color) const override {
        const color white = color(1, 1, 1);
        const int square_x = static_cast<int>(floor(hit_point.x() / size));
        const int square_y = static_cast<int>(floor(hit_point.z() / size));

        // Check for modulo 2 which results in this checkered pattern, else return white
        if ((square_x + square_y) % 2 == 0) return object_color;
        return white;
    }

private:
    double size;
};

class Mirror final : public Material {
public:
    vec3 albedo;

    explicit Mirror(const vec3&_albedo) : albedo(_albedo) {
    }

    [[nodiscard]] color shade(const color&light_color, const point3&hit_point, const Ray&ray,
                              const vec3&light_position, const vec3&normal, const color&object_color) const override {
        return {0.7, 0.8, 1.0};
    }
};

#endif //MATERIAL_H
