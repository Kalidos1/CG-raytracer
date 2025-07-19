#ifndef RAYTRACER_FILTER_H
#define RAYTRACER_FILTER_H

#include <cmath>
#include <algorithm>

// Filter base class
class Filter {
public:
    Filter(double xWidth, double yWidth) : mXWidth(xWidth), mYWidth(yWidth) {}

    virtual ~Filter() = default;

    [[nodiscard]] double getXWidth() const { return mXWidth; }

    [[nodiscard]] double getYWidth() const { return mYWidth; }

    // Evaluate the filter at point (x, y)
    [[nodiscard]] virtual double evaluate(double x, double y) const = 0;

    // Return the normalization term for the filter
    // This is the integral of the filter function over its domain
    [[nodiscard]] virtual double getNormalizeTerm() const = 0;

protected:
    double mXWidth, mYWidth;
};

// Box filter implementation
class BoxFilter : public Filter {
public:
    explicit BoxFilter(double xWidth = 0.5, double yWidth = 0.5) : Filter(xWidth, yWidth) {}

    [[nodiscard]] double evaluate(double x, double y) const override {
        // Box filter returns 1.0 if point is within the filter extent, 0.0 otherwise
        return (std::abs(x) <= mXWidth && std::abs(y) <= mYWidth) ? 1.0 : 0.0;
    }

    [[nodiscard]] double getNormalizeTerm() const override {
        // For box filter, the integral is simply the area: 4 * width * height
        return 4.0 * mXWidth * mYWidth;
    }
};

// Triangle filter implementation (tent filter)
class TriangleFilter : public Filter {
public:
    explicit TriangleFilter(double xWidth = 1.0, double yWidth = 1.0) : Filter(xWidth, yWidth) {}

    [[nodiscard]] double evaluate(double x, double y) const override {
        // Linear falloff from center
        return std::max(0.0, mXWidth - std::abs(x)) * std::max(0.0, mYWidth - std::abs(y));
    }

    [[nodiscard]] double getNormalizeTerm() const override {
        // Integral of triangle filter over its domain
        return mXWidth * mXWidth * mYWidth * mYWidth;
    }
};

// Gaussian filter implementation
class GaussianFilter : public Filter {
public:
    GaussianFilter(double xWidth = 2.0, double yWidth = 2.0, double alpha = 2.0)
            : Filter(xWidth, yWidth), mAlpha(alpha),
              mExpX(std::exp(-alpha * xWidth * xWidth)),
              mExpY(std::exp(-alpha * yWidth * yWidth)) {}

    [[nodiscard]] double evaluate(double x, double y) const override {
        return gaussian(x, mExpX) * gaussian(y, mExpY);
    }

    [[nodiscard]] double getNormalizeTerm() const override {
        // Numerical approximation of the Gaussian integral over finite domain
        const size_t step = 20;
        const double deltaX = mXWidth / static_cast<double>(step);
        const double deltaY = mYWidth / static_cast<double>(step);
        double result = 0.0;

        for (size_t i = 0; i < step; ++i) {
            for (size_t j = 0; j < step; ++j) {
                result += 4.0 * deltaX * deltaY *
                          gaussian(i * deltaX, mExpX) *
                          gaussian(j * deltaY, mExpY);
            }
        }

        return result;
    }

private:
    double mAlpha;
    double mExpX, mExpY;

    [[nodiscard]] double gaussian(double d, double expv) const {
        return std::max(0.0, std::exp(-mAlpha * d * d) - expv);
    }
};

// Mitchell filter implementation
class MitchellFilter : public Filter {
public:
    explicit MitchellFilter(double xWidth = 2.0, double yWidth = 2.0, double b = 1.0 / 3.0, double c = 1.0 / 3.0)
            : Filter(xWidth, yWidth), mB(b), mC(c) {}

    [[nodiscard]] double evaluate(double x, double y) const override {
        return mitchell1D(x / mXWidth) * mitchell1D(y / mYWidth);
    }

    [[nodiscard]] double getNormalizeTerm() const override {
        // This is a simplified approximation of the Mitchell filter integral
        return 4.0 * ((12.0 - 9.0 * mB - 6.0 * mC) / 4.0 +
                      (-18.0 + 12.0 * mB + 6.0 * mC) / 3.0 + (6.0 - 2.0 * mB) +
                      15.0 * (-mB - 6.0 * mB) / 4.0 + 7.0 * (6.0 * mB + 30.0 * mC) / 3.0 +
                      3.0 * (-12.0 * mB - 48.0 * mC) / 2.0 + (8.0 * mB + 24.0 * mC)) / 6.0;
    }

private:
    double mB, mC;

    [[nodiscard]] double mitchell1D(double x) const {
        x = std::abs(2.0 * x);
        if (x > 1.0) {
            return ((-mB - 6.0 * mC) * x * x * x + (6.0 * mB + 30.0 * mC) * x * x +
                    (-12.0 * mB - 48.0 * mC) * x + (8.0 * mB + 24.0 * mC)) / 6.0;
        } else {
            return ((12.0 - 9.0 * mB - 6.0 * mC) * x * x * x +
                    (-18.0 + 12.0 * mB + 6.0 * mC) * x * x +
                    (6.0 - 2.0 * mB)) / 6.0;
        }
    }
};

#endif //RAYTRACER_FILTER_H