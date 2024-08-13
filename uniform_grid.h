#ifndef RAYTRACER_UNIFORM_GRID_H
#define RAYTRACER_UNIFORM_GRID_H

#include <vector>

struct GridCell {
    int start;  // Start index in the object list
    int numTriangles;  // Number of objects in this cell
};

class UniformGrid {
public:
    explicit UniformGrid(int numTriangles) : gridDimensions(vec3(0, 0, 0)),
                                             cellSize(vec3(0, 0, 0)), grid(),
                                             objectList(numTriangles), sceneBounds() {}

    void build(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        createGrid(hittables);
        buildCompactGrid(hittables);
    }

    void intersect(Ray &ray, std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) {
        if (sceneBounds.intersect(ray) >= std::numeric_limits<double>::infinity()) return;

        // Initialize ray parameters
        vec3 rayOrigin = (ray.origin - sceneBounds.min) / cellSize;
        vec3 rayDir = ray.direction / cellSize;
        vec3 invDir = vec3(1.0, 1.0, 1.0) / rayDir;

        // Calculate initial cell
        int x = std::clamp(static_cast<int>(rayOrigin.x()), 0, static_cast<int>(gridDimensions.x()) - 1);
        int y = std::clamp(static_cast<int>(rayOrigin.y()), 0, static_cast<int>(gridDimensions.y()) - 1);
        int z = std::clamp(static_cast<int>(rayOrigin.z()), 0, static_cast<int>(gridDimensions.z()) - 1);

        // Calculate cell stepping direction
        int stepX = (rayDir.x() >= 0) ? 1 : -1;
        int stepY = (rayDir.y() >= 0) ? 1 : -1;
        int stepZ = (rayDir.z() >= 0) ? 1 : -1;

        // Calculate t-max values
        double tMaxX = (x + std::max(0, stepX) - rayOrigin.x()) * invDir.x();
        double tMaxY = (y + std::max(0, stepY) - rayOrigin.y()) * invDir.y();
        double tMaxZ = (z + std::max(0, stepZ) - rayOrigin.z()) * invDir.z();

        // Calculate t-delta values
        double tDeltaX = std::abs(invDir.x());
        double tDeltaY = std::abs(invDir.y());
        double tDeltaZ = std::abs(invDir.z());

        // Initialize p (linear array index)
        int p = x + y * gridDimensions.x() + z * gridDimensions.x() * gridDimensions.y();
        int px = 1;
        int py = gridDimensions.x();
        int pz = gridDimensions.x() * gridDimensions.y();

        while (x >= 0 && x < gridDimensions.x() &&
               y >= 0 && y < gridDimensions.y() &&
               z >= 0 && z < gridDimensions.z()) {


            // Check for intersection in current cell
            const GridCell &cell = grid[p];
            for (int i = 0; i < cell.numTriangles; ++i) {
                int obj_index = objectList[cell.start + i];
                double nearestHit = ray.t;
                if (hittables[obj_index]->intersect(ray)) {
                    if (ray.t < nearestHit) nearestHit = ray.t;
                    else return; // Early termination if we already have a nearer intersection
                    hit_object = obj_index;
                }
            }

            // Move to next cell
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                x += stepX;
                tMaxX += tDeltaX;
                p += stepX * px;
            } else if (tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
                p += stepY * py;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
                p += stepZ * pz;
            }
        }
    }

private:
    vec3 gridDimensions;
    vec3 cellSize;
    std::vector<GridCell> grid;
    std::vector<int> objectList;
    BoundingBox sceneBounds;

    void createGrid(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        //TODO: Same as the morton codes -> Calculate average triangle size

        // Calculate global bounding box
        sceneBounds = calculateSceneBounds(hittables);

        // Estimate the volume of the bounding box
        const double boundingBoxVolume = sceneBounds.volume();
        const int gridDensity = 4; // Can be adjusted, might make a nice comparison
        const vec3 boundingBoxSize = sceneBounds.max - sceneBounds.min;
        const double cellsPerUnit = std::cbrt((gridDensity * hittables.size()) / boundingBoxVolume);

        gridDimensions[0] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.x() * cellsPerUnit)));
        gridDimensions[1] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.y() * cellsPerUnit)));
        gridDimensions[2] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.z() * cellsPerUnit)));

        // Create empty grid with everything 0, 0
        cellSize = boundingBoxSize / gridDimensions;
        int totalCells = gridDimensions.x() * gridDimensions.y() * gridDimensions.z();
        grid.resize(totalCells, {0, 0});
    }

    void buildCompactGrid(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        // Count objects per cell
        for (const auto &hittable: hittables) {
            std::vector<int> overlappingCells = getOverlappingCells(hittable->get_bounding_box());
            for (int cellIndex: overlappingCells) {
                grid[cellIndex].numTriangles++;
            }
        }

        // Compute offsets
        int totalObjects = 0;
        for (int i = 0; i < grid.size(); ++i) {
            int count = grid[i].numTriangles;
            grid[i].start = totalObjects;
            totalObjects += count;
            grid[i].numTriangles = 0;  // Reset count for the second pass
        }

        // Allocate object list
        objectList.resize(totalObjects);

        // Second pass: fill object list
        for (int i = 0; i < hittables.size(); i++) {
            std::vector<int> overlappingCells = getOverlappingCells(hittables[i]->get_bounding_box());
            for (int cellIndex: overlappingCells) {
                int index = grid[cellIndex].start + grid[cellIndex].numTriangles;
                objectList[index] = i;
                grid[cellIndex].numTriangles++;
            }
        }
    }

    std::vector<int> getOverlappingCells(const BoundingBox &box) {
        vec3 minCell = (box.min - sceneBounds.min) / cellSize;
        vec3 maxCell = (box.max - sceneBounds.min) / cellSize;

        std::vector<int> cells;
        for (int x = std::max(0, static_cast<int>(minCell.x()));
             x <= std::min(static_cast<int>(gridDimensions.x()) - 1, static_cast<int>(maxCell.x())); x++) {
            for (int y = std::max(0, static_cast<int>(minCell.y()));
                 y <= std::min(static_cast<int>(gridDimensions.y()) - 1, static_cast<int>(maxCell.y())); y++) {
                for (int z = std::max(0, static_cast<int>(minCell.z()));
                     z <= std::min(static_cast<int>(gridDimensions.z()) - 1, static_cast<int>(maxCell.z())); z++) {
                    int index = x + y * gridDimensions.x() + z * gridDimensions.x() * gridDimensions.y();
                    cells.push_back(index);
                }
            }
        }
        return cells;
    }


    BoundingBox calculateSceneBounds(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BoundingBox bounds;
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        bounds.min = vec3(imax, imax, imax);
        bounds.max = vec3(imin, imin, imin);
        for (const auto &hittable: hittables) {
            bounds = bounds.bb_union(hittable->get_bounding_box());
        }
        return bounds;
    }
};


#endif //RAYTRACER_UNIFORM_GRID_H
