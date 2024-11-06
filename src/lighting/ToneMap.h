#ifndef RAYTRACER_TONEMAP_H
#define RAYTRACER_TONEMAP_H

#include "core/Color.h"

class ToneMap {
public:
    virtual ~ToneMap() = default;

    virtual Color apply(const Color &color) const = 0;

protected:
    static double gamma_correct(const double colorValue) {
        if (colorValue <= 0.0) {
            return 0.0;
        }
        return std::pow(colorValue, 2.2);
    }
};

class ReinhardToneMap : public ToneMap {
public:
    explicit ReinhardToneMap(double white_point = 4.0)
            : white_point_(white_point) {}

    [[nodiscard]] Color apply(const Color &color) const override {
        const double rValue = tone_map(color.x());
        const double gValue = tone_map(color.y());
        const double bValue = tone_map(color.z());

        return {
                gamma_correct(rValue),
                gamma_correct(gValue),
                gamma_correct(bValue)
        };
    }

private:
    double white_point_;

    [[nodiscard]] double tone_map(const double hit) const {
        return hit * (1 + hit / (white_point_ * white_point_)) / (1.0 + hit);
    }
};

#endif //RAYTRACER_TONEMAP_H
