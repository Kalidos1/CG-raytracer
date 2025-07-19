#ifndef RAYTRACER_TONEMAP_H
#define RAYTRACER_TONEMAP_H

#include "core/Color.h"

class ToneMap {
public:
    virtual ~ToneMap() = default;

    virtual Color apply(const Color &color) const = 0;

protected:
    static double gamma_correct(const double colorValue) {
        return std::pow(std::max(0.0, colorValue), 1.0 / 2.2);
    }
};

class ReinhardToneMap : public ToneMap {
public:
    explicit ReinhardToneMap(double white_point)
            : white_point_(white_point) {}

    [[nodiscard]] Color apply(const Color &color) const override {
        const Color toneMappedColor = reinhard_extended(color);

        return {
                gamma_correct(toneMappedColor.x()),
                gamma_correct(toneMappedColor.y()),
                gamma_correct(toneMappedColor.z())
        };
    }

private:
    double white_point_;

    [[nodiscard]] static double luminance(const Vec3 &v) {
        return dot(v, Vec3(0.2126f, 0.7152f, 0.0722f));
    }

    static Vec3 changeLuminance(const Vec3 &cIn, const double &lOut) {
        double l_in = luminance(cIn);
        return cIn * (lOut / l_in);
    }

    [[nodiscard]] Vec3 extendedLuminance(const Vec3 &v) const {
        double lOld = luminance(v);
        double lNew = lOld * (1.0 + (lOld / (white_point_ * white_point_))) / (1.0 + lOld);
        return changeLuminance(v, lNew);
    }

    [[nodiscard]] Vec3 reinhard_extended(const Vec3 &v) const {
        const Vec3 numerator = v * (1.0f + (v / (white_point_ * white_point_)));
        return numerator / (1.0f + v);
    }
};

#endif //RAYTRACER_TONEMAP_H
