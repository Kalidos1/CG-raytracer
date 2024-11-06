#ifndef MATERIAL_H
#define MATERIAL_H

#include "../core/Color.h"
#include "../lighting/Light.h"
#include "core/Ray.h"

class Material {
public:
    virtual ~Material() = default;

    [[nodiscard]] virtual Color shade(const std::shared_ptr<Light> &light, const point3 &hitPoint, const Ray &ray,
                                      const Vec3 &normal,
                                      const Color &objectColor, const double interpolatedUV[2]) const = 0;
};

class PhongMaterial final : public Material {
public:
    explicit PhongMaterial(const double _shininess, const unsigned char *textureData, int textureWidth,
                           int textureHeight, int textureChannels, Color ambient_color, Color diffuse_color,
                           Color specular_color)
            : shininess(_shininess), textureData(textureData), textureWidth(textureWidth),
              textureHeight(textureHeight), textureChannels(textureChannels), ambient_color_temp(ambient_color),
              diffuse_color_temp(diffuse_color), specular_color_temp(specular_color) {}

    [[nodiscard]] Color shade(const std::shared_ptr<Light> &light, const point3 &hitPoint, const Ray &ray,
                              const Vec3 &normal,
                              const Color &objectColor, const double interpolatedUV[2]) const override {
        // Calculate the normal of the object
        const Vec3 viewerDirection = unitVector(hitPoint - ray.origin);
        const Vec3 lightDirection = unitVector(light->origin - hitPoint);
        const Color white = Color(1, 1, 1);

        // Fetch texture Color
        //Color temp_color = get_texture_color(interpolatedUV);
        Color tempColor = objectColor;

        //Diffuse
        const double diffuseComponent = std::max(0.0, dot(lightDirection, normal));

        //Specular
        const Vec3 reflectionVector = unitVector(normal * diffuseComponent * 2 - lightDirection);
        const double specularComponent = std::pow(std::max(dot(viewerDirection, -reflectionVector), 0.0), shininess);

        //Ambient
        //const Color ambientColor = temp_color * 0.5;

        //Phong Formula
        const Vec3 ambientReflection = 1 / M_PI * ambient_color_temp * light->intensity;
        const Vec3 surfaceIllumination = light->lightColor * std::max(0.0, dot(lightDirection, normal)) * light->
                intensity;
        const Vec3 diffuseReflection = 1 / M_PI * (diffuse_color_temp) * light->intensity;
        const Vec3 specularReflection = specular_color_temp * specularComponent * light->intensity;

        const Vec3 phongShade = ambientReflection + surfaceIllumination * (
                diffuseReflection + specularReflection);

        return phongShade;
    }

    Color
    get_texture_color(const double uv[2]) const {
        int tex_x = uv[0] * (textureWidth - 1);
        int tex_y = uv[1] * (textureHeight - 1);
        tex_x = std::max(0, std::min(tex_x, textureWidth - 1));
        tex_y = std::max(0, std::min(tex_y, textureHeight - 1));

        int index = (tex_y * textureWidth + tex_x) * textureChannels;
        return {textureData[index] / 255.0, textureData[index + 1] / 255.0, textureData[index + 2] / 255.0};
    }

private:
    double shininess;
    const unsigned char *textureData;
    int textureWidth;
    int textureHeight;
    int textureChannels;
    Color ambient_color_temp, diffuse_color_temp, specular_color_temp;
};

class LambertianMaterial final : public Material {
public:
    ~LambertianMaterial() override = default;

    [[nodiscard]] Color shade(const std::shared_ptr<Light> &light, const point3 &hitPoint, const Ray &ray,
                              const Vec3 &normal,
                              const Color &objectColor, const double interpolatedUV[2]) const override {
        // Ambient component
        const Color ambient = 0.3 * objectColor;

        // Diffuse component
        const Vec3 lightDirection = unitVector(light->origin - hitPoint);

        // Calculate light attenuation based on distance
        const double distance = (light->origin - hitPoint).length();
        const double attenuation =
                1.0 / (1.0 + 0.09 * distance + 0.032 * distance * distance); // TODO: CHECK WHERE IT COMES FROM

        const auto diffuseComponent = light->lightColor * std::max(0.0, dot(normal, lightDirection));
        const auto diffuseReflection = (1.0 / M_PI) * objectColor * light->intensity * attenuation;

        // Combine components
        return ambient + diffuseReflection * diffuseComponent;
    }
};

class CheckeredMaterial final : public Material {
public:
    explicit CheckeredMaterial(const double _checkeredSize, const Color &_color1,
                               const Color &_color2) : size(_checkeredSize), color1(_color1), color2(_color2) {
    }

    [[nodiscard]] Color shade(const std::shared_ptr<Light> &light, const point3 &hitPoint, const Ray &ray,
                              const Vec3 &normal,
                              const Color &objectColor, const double interpolatedUV[2]) const override {
        const int squareXCoord = static_cast<int>(round(hitPoint.x() / size)); // Use round to round downward
        const int squareYCoord = static_cast<int>(round(hitPoint.y() / size));
        const int squareZCoord = static_cast<int>(round(hitPoint.z() / size));

        Color finalColor1 = color1 * light->intensity;
        Color finalColor2 = color2 * light->intensity;
        // Calculate both of the cell values -> Even number C1, Odd number C2
        if ((squareXCoord + squareYCoord + squareZCoord) % 2 == 0) {
            return finalColor1;
        }
        return finalColor2;
    }

private:
    double size;
    Color color1;
    Color color2;
};

class Mirror final : public Material {
public:
    Vec3 albedo;

    explicit Mirror(const Vec3 &_albedo) : albedo(_albedo) {
    }

    [[nodiscard]] Color shade(const std::shared_ptr<Light> &light, const point3 &hitPoint, const Ray &ray,
                              const Vec3 &normal,
                              const Color &objectColor, const double interpolatedUV[2]) const override {
        // Return background because mirror effect algorithm is insde the trace function
        return {0.7, 0.8, 1.0};
    }
};

#endif //MATERIAL_H
