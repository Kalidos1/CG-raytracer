// Lib
#include "Scene.h"
#include "Renderer.h"
#include "utils/Camera.h"
#include "LightTracer.h"

// C++ Lib
#include <iostream>
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono;

int main() {
    // Default render mode
    RenderMode renderMode = RenderMode::LIGHT_TRACING;

    // Create initial scene
    auto sceneConfig = Scene::createDefaultConfig();
    auto [objects, lights] = Scene::setupObjects(sceneConfig);
    auto accelerationStructure = Scene::setupAccelerationStructure(objects);

    // Create a filter for the renderer (can be any of the implemented filters)
    std::shared_ptr<Filter> filter = std::make_shared<BoxFilter>();
    const int numLightSamples = sceneConfig.imageHeight * sceneConfig.imageWidth * 1;
    const int numBdptSamples = 1;

    // Configure renderer
    Renderer::RenderConfig renderConfig{
            sceneConfig.imageWidth,
            sceneConfig.imageHeight,
            false,  // supersampling
            10,      // maxDepth
            32,     // tileSize
            renderMode,  // renderMode - can be PATH_TRACING, LIGHT_TRACING, or BDPT
            SamplingStrategy::NROOKS,  // samplingStrategy
            filter,                     // filter,
            numLightSamples,
            numBdptSamples
    };

    // Create renderer
    Renderer renderer(renderConfig);

    const int numRenderings = 0;
    auto totalStart = high_resolution_clock::now();

    if (numRenderings == 0) {
        // Original behavior: render a single image
        std::string outputPath;
        if (renderMode == RenderMode::LIGHT_TRACING) {
            outputPath = "../image_light_tracing.ppm";
        } else if (renderMode == RenderMode::BDPT) {
            outputPath = "../image_bdpt.ppm";
        } else {
            outputPath = "../image.ppm";
        }

        auto start = high_resolution_clock::now();

        renderer.render(outputPath,
                        sceneConfig.camera,
                        objects,
                        lights,
                        accelerationStructure);

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        std::cout << "Render completed in " << duration.count() / 1000 << " milliseconds" << std::endl;
    } else {
        // Multiple renderings
        std::vector<std::string> outputPaths;
        std::string basePath;

        if (renderMode == RenderMode::LIGHT_TRACING) {
            basePath = "../images/light_tracing_";
        } else if (renderMode == RenderMode::BDPT) {
            basePath = "../images/bdpt_";
        } else {
            basePath = "../images/image_";
        }

        for (int i = 0; i < numRenderings; i++) {
            std::string outputPath = basePath + std::to_string(i) + ".ppm";
            outputPaths.push_back(outputPath);

            auto start = high_resolution_clock::now();

            renderer.render(outputPath,
                            sceneConfig.camera,
                            objects,
                            lights,
                            accelerationStructure);

            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            std::cout << "Rendering " << (i + 1) << " of " << numRenderings
                      << " completed in " << duration.count() / 1000 << " milliseconds" << std::endl;
        }

        auto totalStop = high_resolution_clock::now();
        auto totalDuration = duration_cast<microseconds>(totalStop - totalStart);
        std::cout << "All " << numRenderings << " renderings completed in "
                  << totalDuration.count() / 1000 << " milliseconds" << std::endl;
    }

    return 0;
}