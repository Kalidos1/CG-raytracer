#ifndef MATERIAL_H
#define MATERIAL_H

#include "../core/Color.h"
#include "core/Ray.h"

enum MaterialType {
    Emissive, Diffuse, Specular, Mirror, Mixed, Transparent
};


class Material {
public:
    virtual ~Material() = default;

    [[nodiscard]] virtual MaterialType getType() const = 0;

    [[nodiscard]] virtual Color shade() const = 0;

    [[nodiscard]] virtual double getRoughness() const { return 0.2; }

    [[nodiscard]] virtual Color getEmissionColor() const {
        return {0, 0, 0};
    }

    [[nodiscard]] virtual bool isMirror() const { return false; }

    // Default diffuse ratio for mixed materials (will be overridden by MixedMaterial)
    [[nodiscard]] virtual double getDiffuseRatio() const { return 0.5; }

    [[nodiscard]] virtual double getIOR() const { return 1.5; }

};

class EmissiveMaterial final : public Material {
public:
    Color emissionColor;
    double emissionStrength;

    explicit EmissiveMaterial(const Color &emissionColor, double emissionStrength = 1.0)
            : emissionColor(emissionColor), emissionStrength(emissionStrength) {}

    [[nodiscard]] MaterialType getType() const override {
        return Emissive;
    }

    [[nodiscard]] Color shade() const override {
        return emissionColor;
    }

    [[nodiscard]] Color getEmissionColor() const override {
        return emissionColor * emissionStrength;
    }
};

class DiffuseMaterial final : public Material {
public:
    Color color;

    [[nodiscard]] MaterialType getType() const override {
        return Diffuse;
    }

    explicit DiffuseMaterial(const Color &color) : color(color) {}

    [[nodiscard]] Color shade() const override {
        return color;
    }
};

class SpecularMaterial final : public Material {
public:
    Color color;
    double roughness;

    explicit SpecularMaterial(const Color &color, const double &roughness = 0.2) : color(color), roughness(roughness) {}

    [[nodiscard]] MaterialType getType() const override {
        return Specular;
    }

    [[nodiscard]] Color shade() const override {
        return color;
    }

    [[nodiscard]] double getRoughness() const override {
        return roughness;
    }
};

class MixedMaterial final : public Material {
public:
    Color color;
    double diffuseRatio; // Ratio of diffuse to specular (0.0 = fully specular, 1.0 = fully diffuse)

    // Constructor with default 50/50 mix
    explicit MixedMaterial(const Color &color, double diffuseRatio = 0.5)
            : color(color), diffuseRatio(clamp(diffuseRatio, 0.0, 1.0)) {}

    [[nodiscard]] MaterialType getType() const override {
        return Mixed;
    }

    [[nodiscard]] Color shade() const override {
        return color;
    }

    [[nodiscard]] double getDiffuseRatio() const override {
        return diffuseRatio;
    }

private:
    // Helper to ensure ratio stays in valid range
    static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

class MirrorMaterial final : public Material {
public:

    [[nodiscard]] MaterialType getType() const override {
        return Mirror;
    }

    [[nodiscard]] bool isMirror() const override { return true; }

    [[nodiscard]] Color shade() const override {
        return {1, 1, 1};
    }
};

class TransparentMaterial final : public Material {
public:
    Color color;
    double ior;

    explicit TransparentMaterial(const Color &color, double indexOfRefraction = 1.5)
            : color(color), ior(indexOfRefraction) {}

    [[nodiscard]] MaterialType getType() const override {
        return Transparent;
    }

    [[nodiscard]] Color shade() const override {
        return color;
    }

    [[nodiscard]] double getIOR() const override {
        return ior;
    }
};

#endif //MATERIAL_H
