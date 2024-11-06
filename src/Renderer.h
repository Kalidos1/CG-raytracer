#ifndef RAYTRACER_RENDERER_H
#define RAYTRACER_RENDERER_H

#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include "Trace.h"

class Renderer {
public:
    struct RenderConfig {
        int imageWidth;
        int imageHeight;
        bool enableSupersampling;
        int maxDepth;
        int tileSize;
    };

    struct Tile {
        int startX, startY, endX, endY;
    };

    explicit Renderer(const RenderConfig &config) : config(config) {
        initializePixelBuffer();
    }

    void render(const std::string &outputPath,
                const point3 &camera,
                const Vec3 &cameraDirection,
                const std::vector<std::shared_ptr<Hittable>> &objects,
                const std::vector<std::shared_ptr<Light>> &lights,
                std::unique_ptr<DataStructure> &accelerationStructure) {
        std::vector<Tile> tiles = createTiles();
        unsigned int threadCount = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;

        auto worker = [this, &camera, &cameraDirection, &objects, &lights, &accelerationStructure, &tiles]() {
            while (true) {
                Tile tile{};
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    if (tiles.empty()) break;
                    tile = tiles.back();
                    tiles.pop_back();
                }
                renderTile(tile, camera, cameraDirection, objects, lights, accelerationStructure);
            }
        };

        threads.reserve(threadCount);
        for (unsigned int i = 0; i < threadCount; ++i) {
            threads.emplace_back(worker);
        }

        for (auto &thread: threads) {
            thread.join();
        }

        writeToFile(outputPath);
    }

private:
    RenderConfig config;
    std::vector<Color> pixelColors;
    std::mutex queueMutex;

    void renderTile(const Tile &tile,
                    const point3 &camera,
                    const Vec3 &cameraDirection,
                    const std::vector<std::shared_ptr<Hittable>> &objects,
                    const std::vector<std::shared_ptr<Light>> &lights,
                    std::unique_ptr<DataStructure> &accelerationStructure) {
        for (int j = tile.startY; j < tile.endY; ++j) {
            for (int k = tile.startX; k < tile.endX; ++k) {
                if (config.enableSupersampling) {
                    Color pixelColor;
                    for (int sy = 0; sy < 2; ++sy) {
                        for (int sx = 0; sx < 2; ++sx) {
                            const double u = (k - config.imageWidth / 2 + 0.5 + sx * 0.5) / config.imageWidth;
                            const double v = (config.imageHeight / 2 - j - 0.5 - sy * 0.5) / config.imageHeight;
                            Ray ray(camera, cameraDirection + Vec3(u, v, 0));
                            pixelColor =
                                    pixelColor +
                                    Trace::computeColorForRay(ray, objects, lights, config.maxDepth,
                                                              accelerationStructure);
                        }
                    }
                    pixelColors[j * config.imageWidth + k] = pixelColor * 0.25;
                } else {
                    const double u = static_cast<double>(k - config.imageWidth / 2) / config.imageWidth;
                    const double v = static_cast<double>(config.imageHeight / 2 - j) / config.imageHeight;
                    Ray ray(camera, cameraDirection + Vec3(u, v, 0));
                    Color pixelColor = Trace::computeColorForRay(ray, objects, lights, config.maxDepth,
                                                                 accelerationStructure);
                    pixelColors[j * config.imageWidth + k] = pixelColor;
                }
            }
        }
    }

    void writeToFile(const std::string &outputPath) {
        std::ofstream file(outputPath);
        file << "P3\n" << config.imageWidth << ' ' << config.imageHeight << "\n255\n";
        for (const auto &pixel_color: pixelColors) {
            writeColor(file, pixel_color);
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
