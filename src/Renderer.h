#ifndef RAYTRACER_RENDERER_H
#define RAYTRACER_RENDERER_H

#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include <random>
#include "Trace.h"
#include "lighting/BDPT.h"
#include "Film.h"
#include "LightTracer.h"

enum class SamplingStrategy {
    STRATIFIED,
    NROOKS,
    HALTON
};

// Enum for rendering modes
enum class RenderMode {
    PATH_TRACING,
    LIGHT_TRACING,
    BDPT
};

class Renderer {
public:
    struct RenderConfig {
        int imageWidth;
        int imageHeight;
        bool enableSupersampling;
        int depth;
        int tileSize;
        RenderMode renderMode;
        SamplingStrategy samplingStrategy;
        std::shared_ptr<Filter> filter;
        int lightTracingSamples;
        int bdptSamples;
    };

    struct Tile {
        int startX, startY, endX, endY;
    };

    explicit Renderer(const RenderConfig &config) : config(config) {
        initializePixelBuffer();
        film = std::make_unique<Film>(config.imageWidth, config.imageHeight, config.filter);
        std::random_device rd;
        generator = std::mt19937(rd());
        distribution = std::uniform_real_distribution<double>(0.0, 1.0);
    }

    void render(const std::string &outputPath,
                Camera &camera,
                const std::vector<std::shared_ptr<Hittable>> &objects,
                const Light &lights,
                std::unique_ptr<DataStructure> &accelerationStructure) {

        if (config.renderMode == RenderMode::LIGHT_TRACING) {
            // For light tracing, use a separate method
            renderLightTracing(outputPath, camera, objects, lights, accelerationStructure);
        } else {
            // For path tracing or BDPT, use the traditional tile-based method
            renderTileBased(outputPath, camera, objects, lights, accelerationStructure);
        }
    }

private:
    RenderConfig config;
    std::unique_ptr<Film> film;
    std::vector<Color> pixelColors;
    std::mutex queueMutex;
    double maxWhitePoint = 0.0;
    std::mt19937 generator;
    std::uniform_real_distribution<double> distribution;

    void renderLightTracing(const std::string &outputPath,
                            Camera &camera,
                            const std::vector<std::shared_ptr<Hittable>> &objects,
                            const Light &lights,
                            std::unique_ptr<DataStructure> &accelerationStructure) {

        // Clear the film before starting
        film->clear();

        // Create a light tracer with specified max depth
        LightTracer lightTracer(config.depth, 1); // 1 sample per call

        // Simple progress tracking
        int progressStep = config.lightTracingSamples / 20; // Show progress every 5%
        if (progressStep < 1) progressStep = 1;

        // Simply trace the specified number of light paths
        for (int i = 0; i < config.lightTracingSamples; ++i) {
            // Progress reporting
            if (i % progressStep == 0) {
                double percentage = 100.0 * i / config.lightTracingSamples;
                std::cout << "Light tracing progress: " << static_cast<int>(percentage) << "%" << std::endl;
            }

            // Trace a single light path
            lightTracer.render(objects, lights, camera, *film, accelerationStructure);
        }

        std::cout << "Light tracing completed with " << config.lightTracingSamples << " samples" << std::endl;

        // Write the final image to file with proper normalization
        writeToFile(outputPath, true, camera);
    }

    void renderBDPT(const std::string &outputPath,
                    Camera &camera,
                    const std::vector<std::shared_ptr<Hittable>> &objects,
                    const Light &lights,
                    std::unique_ptr<DataStructure> &accelerationStructure) {

        // Clear the film before starting
        film->clear();

        // Create BDPT instance
        BDPT bdpt(1, config.depth); // 1 sample per call, max depth from config

        // Calculate total number of samples (similar to light tracing samples)
        int totalSamples = config.bdptSamples;

        // Simple progress tracking
        int progressStep = totalSamples / 20; // Show progress every 5%
        if (progressStep < 1) progressStep = 1;

        // Generate samples across the image
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        for (int i = 0; i < totalSamples; ++i) {
            // Progress reporting
            if (i % progressStep == 0) {
                double percentage = 100.0 * i / totalSamples;
                std::cout << "BDPT progress: " << static_cast<int>(percentage) << "%" << std::endl;
            }

            // Generate random pixel coordinates
            double pixelX = distribution(generator) * config.imageWidth;
            double pixelY = distribution(generator) * config.imageHeight;

            // Convert to camera ray (similar to your existing camera ray generation)
            const double u = static_cast<double>(pixelX - config.imageWidth / 2) / config.imageWidth;
            const double v = static_cast<double>(config.imageHeight / 2 - pixelY) / config.imageHeight;

            Ray cameraRay(camera.getPosition(), camera.getDirection() + Vec3(u, v, 0));

            // Trace BDPT paths
            bdpt.computeColorForRay(cameraRay, objects, lights, accelerationStructure,
                                    camera, *film, pixelX, pixelY);
        }

        std::cout << "BDPT completed with " << totalSamples << " samples" << std::endl;

        // Write the final image to file with proper normalization
        writeToFile(outputPath, true, camera);
    }

    void renderTileBased(const std::string &outputPath,
                         Camera &camera,
                         const std::vector<std::shared_ptr<Hittable>> &objects,
                         const Light &lights,
                         std::unique_ptr<DataStructure> &accelerationStructure) {

        std::vector<Tile> tiles = createTiles();
        unsigned int threadCount = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;

        auto worker = [this, &camera, &objects, &lights, &accelerationStructure, &tiles]() {
            while (true) {
                Tile tile{};
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    if (tiles.empty()) break;
                    tile = tiles.back();
                    tiles.pop_back();
                }
                renderTile(tile, camera, objects, lights, accelerationStructure);
            }
        };

        threads.reserve(threadCount);
        for (unsigned int i = 0; i < threadCount; ++i) {
            threads.emplace_back(worker);
        }

        for (auto &thread: threads) {
            thread.join();
        }

        bool isLightTracing = true;

        if (config.renderMode == RenderMode::PATH_TRACING) {
            isLightTracing = false;
        }
        writeToFile(outputPath, isLightTracing, camera);
    }

    double haltonSequence(int index, int base) {
        double result = 0;
        double f = 1.0 / base;
        int i = index;
        while (i > 0) {
            result += f * (i % base);
            i = i / base;
            f = f / base;
        }
        return result;
    }

    Color samplePixel(const double &u, const double &v, Camera &camera,
                      const std::vector<std::shared_ptr<Hittable>> &objects,
                      const Light &lights,
                      std::unique_ptr<DataStructure> &accelerationStructure,
                      int pixelJ, int pixelK) {
        Trace trace;
        BDPT bdpt(config.bdptSamples);
        Color pixelColor(0, 0, 0);
        int numberOfSamples = config.renderMode == RenderMode::PATH_TRACING ? trace.monteCarlo.numberOfSamples
                                                                            : bdpt.monteCarlo.numberOfSamples;

        switch (config.samplingStrategy) {
            case SamplingStrategy::STRATIFIED: {
                int gridSize = static_cast<int>(sqrt(numberOfSamples));
                for (int py = 0; py < gridSize; py++) {
                    for (int px = 0; px < gridSize; px++) {
                        double stratifiedU = u + (px + distribution(generator)) / (gridSize * config.imageWidth);
                        double stratifiedV = v + (py + distribution(generator)) / (gridSize * config.imageHeight);

                        Ray ray(camera.getPosition(), camera.getDirection() + Vec3(stratifiedU, stratifiedV, 0));

                        if (config.renderMode == RenderMode::PATH_TRACING) {
                            pixelColor += trace.computeColorForRay(ray, objects, lights, config.depth,
                                                                   accelerationStructure);
                        } else {
                            double pixelX = (stratifiedU + 0.5) * config.imageWidth;
                            double pixelY = (0.5 - stratifiedV) * config.imageHeight;

                            bdpt.computeColorForRay(ray, objects, lights,
                                                    accelerationStructure, camera, *film, pixelX, pixelY);
                        }
                    }
                }
                break;
            }

            case SamplingStrategy::NROOKS: {
                std::vector<int> xPositions(numberOfSamples);
                std::vector<int> yPositions(numberOfSamples);

                for (int i = 0; i < numberOfSamples; i++) {
                    xPositions[i] = i;
                    yPositions[i] = i;
                }

                std::shuffle(xPositions.begin(), xPositions.end(), generator);
                std::shuffle(yPositions.begin(), yPositions.end(), generator);

                for (int i = 0; i < numberOfSamples; i++) {
                    double jitterX = (xPositions[i] + distribution(generator)) / numberOfSamples;
                    double jitterY = (yPositions[i] + distribution(generator)) / numberOfSamples;

                    double pixelX = pixelK + jitterX;
                    double pixelY = pixelJ + jitterY;

                    // Use proper perspective ray generation
                    Ray ray = camera.generateRay(pixelX, pixelY);
                    if (config.renderMode == RenderMode::PATH_TRACING) {
                        pixelColor += trace.computeColorForRay(ray, objects, lights, config.depth,
                                                               accelerationStructure);
                    } else {
                        bdpt.computeColorForRay(ray, objects, lights, accelerationStructure,
                                                camera, *film, pixelX, pixelY);
                    }
                }
                break;
            }

            case SamplingStrategy::HALTON: {
                for (int i = 0; i < numberOfSamples; i++) {
                    double haltonU = haltonSequence(i, 2);  // Base 2 for X
                    double haltonV = haltonSequence(i, 3);  // Base 3 for Y

                    double sampleU = u + haltonU / config.imageWidth;
                    double sampleV = v + haltonV / config.imageHeight;

                    Ray ray(camera.getPosition(), camera.getDirection() + Vec3(sampleU, sampleV, 0));
                    if (config.renderMode == RenderMode::PATH_TRACING) {
                        pixelColor += trace.computeColorForRay(ray, objects, lights, config.depth,
                                                               accelerationStructure);
                    } else {
                        double pixelX = (sampleU + 0.5) * config.imageWidth;
                        double pixelY = (0.5 - sampleV) * config.imageHeight;
                        bdpt.computeColorForRay(ray, objects, lights,
                                                accelerationStructure, camera, *film, pixelX, pixelY);
                    }
                }
                break;
            }
        }

        return pixelColor / numberOfSamples;
    }

    void renderTile(const Tile &tile, Camera &camera,
                    const std::vector<std::shared_ptr<Hittable>> &objects,
                    const Light &lights,
                    std::unique_ptr<DataStructure> &accelerationStructure) {
        for (int j = tile.startY; j < tile.endY; ++j) {
            for (int k = tile.startX; k < tile.endX; ++k) {
                if (config.enableSupersampling) {
                    Color pixelColor;
                    for (int sy = 0; sy < 2; ++sy) {
                        for (int sx = 0; sx < 2; ++sx) {
                            double u = static_cast<double>(k - config.imageWidth / 2 + sx * 0.5) / config.imageWidth;
                            double v = static_cast<double>(config.imageHeight / 2 - j - sy * 0.5) / config.imageHeight;

                            Color subpixelColor = samplePixel(u, v, camera, objects, lights, accelerationStructure, j,
                                                              k);
                            pixelColor += subpixelColor * 0.25; // Average the 4 supersamples
                        }
                    }

                    const auto maxColorValue = vecMax(pixelColor);
                    if (maxColorValue > maxWhitePoint) maxWhitePoint = maxColorValue;
                    pixelColors[j * config.imageWidth + k] = pixelColor;
                } else {
                    const double u = static_cast<double>(k - config.imageWidth / 2) / config.imageWidth;
                    const double v = static_cast<double>(config.imageHeight / 2 - j) / config.imageHeight;

                    Color pixelColor = samplePixel(u, v, camera, objects, lights, accelerationStructure, j, k);

                    const auto maxColorValue = vecMax(pixelColor);
                    if (maxColorValue > maxWhitePoint) maxWhitePoint = maxColorValue;
                    pixelColors[j * config.imageWidth + k] = pixelColor;
                }
            }
        }
    }

    void writeToFile(const std::string &outputPath, bool isLightTracing, const Camera &camera) {
        std::ofstream file(outputPath);
        file << "P3\n" << config.imageWidth << ' ' << config.imageHeight << "\n255\n";

        if (isLightTracing) {
            int numberOfSamples = config.imageWidth * config.imageHeight * config.bdptSamples;
            std::vector<Color> finalImage = film->getImage(numberOfSamples,
                                                           camera.getFilmArea());
            std::unique_ptr<ToneMap> tone_mapper = std::make_unique<ReinhardToneMap>(film->getMaxWhitePoint());

            for (const auto &pixel_color: finalImage) {
                Color finalColor = tone_mapper->apply(pixel_color);
                writeColor(file, finalColor);
            }
        } else {
            std::unique_ptr<ToneMap> tone_mapper = std::make_unique<ReinhardToneMap>(maxWhitePoint);
            for (const auto &pixel_color: pixelColors) {
                Color finalColor = tone_mapper->apply(pixel_color);
                writeColor(file, finalColor);
            }
        }

        file.close();
    }

    void initializePixelBuffer() {
        pixelColors.resize(config.imageWidth * config.imageHeight);
    }

    [[nodiscard]] std::vector<Tile> createTiles() const {
        std::vector<Tile> tiles;
        for (int y = 0; y < config.imageHeight; y += config.tileSize) {
            for (int x = 0; x < config.imageWidth; x += config.tileSize) {
                tiles.push_back({
                                        x, y,
                                        std::min(x + config.tileSize, config.imageWidth),
                                        std::min(y + config.tileSize, config.imageHeight)
                                });
            }
        }
        return tiles;
    }
};

#endif //RAYTRACER_RENDERER_H
