// Lib
#include "Scene.h"
#include "Renderer.h"

// C++ Lib
#include <iostream>
#include <chrono>

using namespace std::chrono;

/*
 * TODO:
 * 1. FIX Compact Grid intersection when using cornell box
 * 2. Make Light with OBJ aswell -> Flächenlichtquelle
 */


int main() {
    // Create initial scene
    auto sceneConfig = Scene::createDefaultConfig();
    auto lights = Scene::setupLights();
    auto objects = Scene::setupObjects(sceneConfig);
    auto accelerationStructure = Scene::setupAccelerationStructure(objects);

    // Configure renderer
    Renderer::RenderConfig renderConfig{
            sceneConfig.imageWidth,
            sceneConfig.imageHeight,
            false,  // supersampling
            2,      // maxDepth
            32      // tileSize
    };

    // Create renderer and render scene
    Renderer renderer(renderConfig);

    auto start = high_resolution_clock::now();

    renderer.render("../image.ppm",
                    sceneConfig.camera,
                    sceneConfig.cameraDirection,
                    objects,
                    lights,
                    accelerationStructure);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "Total render time: " << duration.count() / 1000 << " milliseconds" << std::endl;

    return 0;
}
