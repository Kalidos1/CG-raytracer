#ifndef RAYTRACER_FILM_H
#define RAYTRACER_FILM_H

#include <utility>
#include <vector>
#include <memory>
#include "core/Color.h"
#include "utils/Filter.h"
#include "Renderer.h"

class Film {
public:
    Film(int width, int height, std::shared_ptr<Filter> filter)
            : mWidth(width),
              mHeight(height),
              mFilter(std::move(filter)),
              mPixelColors(width * height, Color(0, 0, 0)) {

        // Precompute filter table for efficiency
        initFilterTable();

        // Calculate pixel world area (needed for normalization)
        // Might not need the filmarea not sure
        mInvPixelArea = 1.0 / ((1.0 / static_cast<double>(width) * 1.0 / static_cast<double>(height)));
    }

    // Reset the film
    void clear() {
        std::fill(mPixelColors.begin(), mPixelColors.end(), Color(0, 0, 0));
    }

    void addLightSample(double xSample, double ySample, const Color &sampleColor) {
        // Convert sample position to continuous coordinates
        double continuousX = xSample - 0.5;
        double continuousY = ySample - 0.5;

        // Calculate the pixel range covered by filter centered at sample
        double xWidth = mFilter->getXWidth();
        double yWidth = mFilter->getYWidth();
        int x0 = static_cast<int>(std::ceil(continuousX - xWidth));
        int x1 = static_cast<int>(std::floor(continuousX + xWidth));
        int y0 = static_cast<int>(std::ceil(continuousY - yWidth));
        int y1 = static_cast<int>(std::floor(continuousY + yWidth));

        // Clamp to image boundaries
        x0 = std::max(0, x0);
        x1 = std::min(mWidth - 1, x1);
        y0 = std::max(0, y0);
        y1 = std::min(mHeight - 1, y1);

        // Filter table resolution
        constexpr int FILTER_TABLE_SIZE = 16;

        for (int y = y0; y <= y1; ++y) {
            double fy = std::abs(FILTER_TABLE_SIZE * (y - continuousX) / yWidth);
            int iy = std::min(static_cast<int>(std::floor(fy)), FILTER_TABLE_SIZE - 1);

            for (int x = x0; x <= x1; ++x) {
                double fx = std::abs(FILTER_TABLE_SIZE * (x - continuousY) / xWidth);
                int ix = std::min(static_cast<int>(std::floor(fx)), FILTER_TABLE_SIZE - 1);

                // Get filter weight (already normalized by filter integration)
                double weight = mFilterTable[iy * FILTER_TABLE_SIZE + ix] * mInvPixelArea;

                // Add weighted sample to pixel
                int index = y * mWidth + x;
                mPixelColors[index] += weight * sampleColor;
            }
        }
    }

    std::vector<Color> getImage(int totalLightSamples, double filmArea) {
        double scale = filmArea / totalLightSamples;

        for (auto &color: mPixelColors) {
            color = color * scale;
            const auto maxColorValue = vecMax(color);
            if (maxColorValue > maxWhitePoint) maxWhitePoint = maxColorValue;
        }

        return mPixelColors;
    }

    [[nodiscard]] double getMaxWhitePoint() const { return maxWhitePoint; }

private:
    int mWidth, mHeight;
    std::shared_ptr<Filter> mFilter;
    std::vector<Color> mPixelColors;
    std::vector<double> mFilterTable;
    double maxWhitePoint;
    double mInvPixelArea;

    void initFilterTable() {
        constexpr int FILTER_TABLE_SIZE = 16;
        mFilterTable.resize(FILTER_TABLE_SIZE * FILTER_TABLE_SIZE);

        double deltaX = mFilter->getXWidth() / FILTER_TABLE_SIZE;
        double deltaY = mFilter->getYWidth() / FILTER_TABLE_SIZE;
        double normalizeTerm = mFilter->getNormalizeTerm();

        int index = 0;
        for (int y = 0; y < FILTER_TABLE_SIZE; ++y) {
            double fy = y * deltaY;
            for (int x = 0; x < FILTER_TABLE_SIZE; ++x) {
                double fx = x * deltaX;
                mFilterTable[index++] = mFilter->evaluate(fx, fy) / normalizeTerm;
            }
        }
    }
};

#endif //RAYTRACER_FILM_H