#ifndef MATERIAL_H
#define MATERIAL_H

#include "color.h"
#include "light.h"

class Material {
public:
    virtual ~Material() = default;

    [[nodiscard]] virtual color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                                      const vec3&normal,
                                      const color &object_color, const double interpolated_uv[2]) const = 0;
};

class PhongMaterial final : public Material {
public:
    explicit PhongMaterial(const double _shininess, const unsigned char *texture_data, int texture_width,
                           int texture_height, int texture_channels, color ambient_color, color diffuse_color,
                           color specular_color)
            : shininess(_shininess), texture_data(texture_data), texture_width(texture_width),
              texture_height(texture_height), texture_channels(texture_channels), ambient_color_temp(ambient_color),
              diffuse_color_temp(diffuse_color), specular_color_temp(specular_color) {}

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color &object_color, const double interpolated_uv[2]) const override {
        // Calculate the normal of the object
        const vec3 viewer_direction = unit_vector(hit_point - ray.origin);
        const vec3 light_direction = unit_vector(light->origin - hit_point);
        const color white = color(1, 1, 1);

        // Fetch texture color
        //color temp_color = get_texture_color(interpolated_uv);
        color temp_color = object_color;

        //Diffuse
        const double diffuse_component = std::max(0.0, dot(light_direction, normal));

        //Specular
        const vec3 reflection_vector = unit_vector(normal * diffuse_component * 2 - light_direction);
        const double specular_component = std::pow(std::max(dot(viewer_direction, -reflection_vector), 0.0), shininess);

        //Ambient
        const color ambient_color = temp_color * 0.5;

        //Phong Formula
        const vec3 ambient_reflection = 1 / M_PI * ambient_color * light->intensity;
        const vec3 surface_illumination = light->light_color * std::max(0.0, dot(light_direction, normal)) * light->
                                          intensity;
        const vec3 diffuse_reflection = 1 / M_PI * (temp_color) * light->intensity;
        const vec3 specular_reflection = white * specular_component * light->intensity;

        const vec3 phong_shade = ambient_reflection + surface_illumination * (
                                     diffuse_reflection + specular_reflection);

        return phong_shade;
    }

    color
    get_texture_color(const double uv[2]) const {
        int tex_x = uv[0] * (texture_width - 1);
        int tex_y = uv[1] * (texture_height - 1);
        tex_x = std::max(0, std::min(tex_x, texture_width - 1));
        tex_y = std::max(0, std::min(tex_y, texture_height - 1));

        int index = (tex_y * texture_width + tex_x) * texture_channels;
        return {texture_data[index] / 255.0, texture_data[index + 1] / 255.0, texture_data[index + 2] / 255.0};
    }

private:
    double shininess;
    const unsigned char *texture_data;
    int texture_width;
    int texture_height;
    int texture_channels;
    color ambient_color_temp, diffuse_color_temp, specular_color_temp;
};

class LambertianMaterial final : public Material {
public:
    ~LambertianMaterial() override = default;

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color &object_color, const double interpolated_uv[2]) const override {
        const vec3 light_direction = unit_vector(light->origin - hit_point);

        const auto diffuse_component = light->light_color * std::max(0.0, dot(normal, light_direction));
        const auto diffuse_reflection = 1 / M_PI * object_color * light->intensity;

        return diffuse_reflection * diffuse_component;
    }
};

class CheckeredMaterial final : public Material {
public:
    explicit CheckeredMaterial(const double _checkered_size, const color&_color1,
                               const color&_color2) : size(_checkered_size), color1(_color1), color2(_color2) {
    }

    [[nodiscard]] color shade(const std::shared_ptr<Light>&light, const point3&hit_point, const Ray&ray,
                              const vec3&normal,
                              const color &object_color, const double interpolated_uv[2]) const override {
        const int square_x_coord = static_cast<int>(round(hit_point.x() / size)); // Use round to round downward
        const int square_y_coord = static_cast<int>(round(hit_point.y() / size));
        const int square_z_coord = static_cast<int>(round(hit_point.z() / size));

        color final_color_1 = color1 * light->intensity;
        color final_color_2 = color2 * light->intensity;
        // Calculate both of the cell values -> Even number C1, Odd number C2
        if ((square_x_coord + square_y_coord + square_z_coord) % 2 == 0) {
            return final_color_1;
        }
        return final_color_2;
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
                              const color &object_color, const double interpolated_uv[2]) const override {
        // Return background because mirror effect algorithm is insde the trace function
        return {0.7, 0.8, 1.0};
    }
};

#endif //MATERIAL_H
