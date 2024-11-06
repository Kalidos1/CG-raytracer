#ifndef RAYTRACER_BVHBUILDER_H
#define RAYTRACER_BVHBUILDER_H


#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include "DataStructure.h"

enum Split {
    Middle, SAH, Linear, LinearSAH
};

struct Bin {
    BoundingBox bounds;
    int triCount = 0;
};

struct BVHNode {
    BoundingBox boundingBox;
    int leftFirst{}, triCount{}, totalTriCount{};

    [[nodiscard]] bool isLeaf() const { return triCount > 0; }
};

struct MortonPrimitive {
    int mortonCode;
    int primitiveIndex;
};

class BVHBuilder : public DataStructure {
public:
    explicit BVHBuilder(int numTriangles, Split split) : triIdx(new int[numTriangles]),
                                                         bvhNode(new BVHNode[2 * numTriangles]), rootNodeIdx(0),
                                                         nodesUsed(1), numTriangles(numTriangles), split(split) {}

    void build(const std::vector<std::shared_ptr<Hittable>> &hittables) override {
        if (split == Split::Linear) {
            // Build BVH Bottom-Up
            buildBvhBottomUp(hittables, 0);
        } else if (split == Split::LinearSAH) {
            // Build BVH Bottom-Up
            buildBvhBottomUp(hittables, 64);
        } else {
            // Build BVH Top-Down
            for (int i = 0; i < numTriangles; i++) triIdx[i] = i;

            BVHNode &root = bvhNode[rootNodeIdx];
            root.leftFirst = 0, root.triCount = numTriangles;
            updateNodeBounds(rootNodeIdx, hittables);
            subdivide(rootNodeIdx, hittables);
        }
    }

    void intersect(Ray &ray, const std::vector<std::shared_ptr<Hittable>> &hittables, int &hitObject) override {
        intersectBvh(ray, rootNodeIdx, hittables, hitObject);
    }

    void
    intersectBvh(Ray &ray, const int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables,
                 int &hitObject) {
        BVHNode *node = &bvhNode[nodeIdx], *stack[64];
        int stackPtr = 0;

        while (true) {
            if (node->isLeaf()) {
                for (int i = 0; i < node->triCount; i++) {
                    int objIndex = triIdx[node->leftFirst + i];
                    double nearestHit = ray.t;
                    if (hittables[objIndex]->intersect(ray)) {
                        if (ray.t < nearestHit) nearestHit = ray.t;
                        else return; // Early termination if we already have a nearer intersection
                        hitObject = objIndex;
                    }
                }
                if (stackPtr == 0) break;
                else node = stack[--stackPtr];
                continue;
            }
            BVHNode *child1 = &bvhNode[node->leftFirst];
            BVHNode *child2 = &bvhNode[node->leftFirst + 1];

            double dist1 = child1->boundingBox.intersect(ray);
            double dist2 = child2->boundingBox.intersect(ray);

            if (dist1 > dist2) {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }

            if (dist1 == std::numeric_limits<double>::infinity()) {
                if (stackPtr == 0) break;
                else node = stack[--stackPtr];
            } else {
                node = child1;
                if (dist2 != std::numeric_limits<double>::infinity()) stack[stackPtr++] = child2;
            }
        }
    }

private:
    int *triIdx;
    BVHNode *bvhNode;
    int rootNodeIdx, nodesUsed, numTriangles;
    Split split;

    void updateNodeBounds(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BVHNode &node = bvhNode[nodeIdx];
        for (int first = node.leftFirst, i = 0; i < node.triCount; i++) {
            const int leafTriIdx = triIdx[first + i];
            const BoundingBox &boundingBox = hittables[leafTriIdx]->getBoundingBox();
            node.boundingBox.min = min(node.boundingBox.min, boundingBox.min);
            node.boundingBox.max = max(node.boundingBox.max, boundingBox.max);
        }
    }

    double findBestSplit(BVHNode &node, int &axis, double &splitPos,
                         const std::vector<std::shared_ptr<Hittable>> &hittables) {
        double bestCost = 1e30f;

        // Determine the longest axis
        Vec3 diag = node.boundingBox.max - node.boundingBox.min;
        int longestAxis = 0;
        if (diag.y() > diag.x()) longestAxis = 1;
        if (diag.z() > diag[longestAxis]) longestAxis = 2;

        int a = longestAxis;

        double boundsMin = node.boundingBox.min[a];
        double boundsMax = node.boundingBox.max[a];
        if (boundsMin == boundsMax) return bestCost;

        // populate bins
        const int numberOfIntervals = 8;
        Bin bins[numberOfIntervals];
        double scale = numberOfIntervals / (boundsMax - boundsMin);
        for (int k = 0; k < node.triCount; k++) {
            const std::shared_ptr<Hittable> &triangle = hittables[triIdx[node.leftFirst + k]];
            int binIdx = std::min(numberOfIntervals - 1,
                                  (int) ((triangle->calculateCenter()[a] - boundsMin) * scale));
            bins[binIdx].triCount++;
            bins[binIdx].bounds.grow(triangle->getV0());
            bins[binIdx].bounds.grow(triangle->getV1());
            bins[binIdx].bounds.grow(triangle->getV2());
        }

        double leftArea[numberOfIntervals - 1], rightArea[numberOfIntervals - 1];
        int leftCount[numberOfIntervals - 1], rightCount[numberOfIntervals - 1];

        BoundingBox leftBox, rightBox;
        int leftSum = 0, rightSum = 0;
        for (int i = 0; i < numberOfIntervals - 1; i++) {
            leftSum += bins[i].triCount;
            leftCount[i] = leftSum;
            leftBox.grow(bins[i].bounds);
            leftArea[i] = leftBox.area();

            rightSum += bins[numberOfIntervals - 1 - i].triCount;
            rightCount[numberOfIntervals - 2 - i] = rightSum;
            rightBox.grow(bins[numberOfIntervals - 1 - i].bounds);
            rightArea[numberOfIntervals - 2 - i] = rightBox.area();
        }

        scale = (boundsMax - boundsMin) / numberOfIntervals;
        for (int i = 0; i < numberOfIntervals - 1; i++) {
            double planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
            if (planeCost < bestCost) {
                axis = a;
                splitPos = boundsMin + scale * (i + 1);
                bestCost = planeCost;
            }
        }
        return bestCost;
    }

    void subdivide(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BVHNode &node = bvhNode[nodeIdx];
        if (node.triCount <= 2) return;

        int axis = 0;
        double splitPos = 0.0;

        if (split == Split::SAH) {
            // SAH Split
            findBestSplit(node, axis, splitPos, hittables);

            // terminate recursion
//            double noSplitCost = calculateNodeCost(node);
//            if (splitCost >= noSplitCost) return;

        } else {
            // Middle Split
            Vec3 extent = node.boundingBox.max - node.boundingBox.min;
            if (extent.y() > extent.x()) axis = 1;
            if (extent.z() > extent[axis]) axis = 2;
            splitPos = node.boundingBox.min[axis] + extent[axis] * 0.5f;
        }

        // in-place partition
        int i = node.leftFirst;
        int j = i + node.triCount - 1;
        while (i <= j) {
            if (hittables[triIdx[i]]->calculateCenter()[axis] < splitPos) i++;
            else std::swap(triIdx[i], triIdx[j--]);
        }

        int leftCount = i - node.leftFirst;
        if (leftCount == 0 || leftCount == node.triCount) return;

        //create child nodes
        int leftChildIdx = nodesUsed++;
        int rightChildIdx = nodesUsed++;
        bvhNode[leftChildIdx].leftFirst = node.leftFirst;
        bvhNode[leftChildIdx].triCount = leftCount;
        bvhNode[rightChildIdx].leftFirst = i;
        bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
        node.leftFirst = leftChildIdx;
        node.triCount = 0;
        updateNodeBounds(leftChildIdx, hittables);
        updateNodeBounds(rightChildIdx, hittables);

        //recursive call
        subdivide(leftChildIdx, hittables);
        subdivide(rightChildIdx, hittables);
    }

    static inline int expandBits(int x) {
        if (x == (1 << 10)) --x;
        x = (x | (x << 16)) & 0b00000011000000000000000011111111;
        x = (x | (x << 8)) & 0b00000011000000001111000000001111;
        x = (x | (x << 4)) & 0b00000011000011000011000011000011;
        x = (x | (x << 2)) & 0b00001001001001001001001001001001;
        return x;
    }

    static inline int encodeMorton3(const Vec3 &v, const BoundingBox &globalBounds, int gridResolution) {
        // Normalize to [0, 1] inside the global bounds
        //const Vec3 normalize = (v - globalBounds.min) / (globalBounds.max - globalBounds.min);
        const Vec3 normalize = globalBounds.offset(v);

        int scale = 160;

        // Scale to [0, scale] depending on the average triangle volume
        int x = static_cast<int>(normalize.x() * scale);
        int y = static_cast<int>(normalize.y() * scale);
        int z = static_cast<int>(normalize.z() * scale);

        return (morton3D(x, y, z));
    }

    static inline int morton3D(int x, int y, int z) {
        return (expandBits(x) << 2) | (expandBits(y) << 1) |
               expandBits(z);
    }

    static int determineGridResolution(double averageTriangleVolume, const BoundingBox &globalBounds) {
        Vec3 boundsSize = globalBounds.max - globalBounds.min;

        // Estimate the volume of the bounding box
        double boundingBoxVolume = globalBounds.volume();

        // Estimate the number of cells
        double numCells = boundingBoxVolume / averageTriangleVolume;

        double gridSizeX = std::cbrt(numCells * (boundsSize.x() / boundingBoxVolume));
        double gridSizeY = std::cbrt(numCells * (boundsSize.y() / boundingBoxVolume));
        double gridSizeZ = std::cbrt(numCells * (boundsSize.z() / boundingBoxVolume));
        const int gridResolution = (gridSizeX + gridSizeY + gridSizeZ) / 3;

        return std::max(1, std::min(gridResolution, 1024));
    }

    static double calculateSAHCost(const BVHNode &node1, const BVHNode &node2) {
        BoundingBox combinedBB = node1.boundingBox.bbUnion(node2.boundingBox);
        return combinedBB.area() * (node1.totalTriCount + node2.totalTriCount);
    }

    void buildSAHNodes(int offset, int numPrimitives) {
        // Create vector out of remaining nodes while maintaining indexes
        std::vector<int> remainingNodes(numPrimitives);
        for (int i = 0; i < numPrimitives; i++) {
            remainingNodes[i] = offset + i;
        }

        // Go through every node and calculate the SAH Cost of every different combination
        // TODO: Might be better to apply same theories used at the top for this here (Bins)
        while (remainingNodes.size() > 1) {
            double bestCost = std::numeric_limits<double>::max();
            int bestFirst = -1, bestSecond = -1;
            int listIdxFirst = -1, listIdxSecond = -1;

            // Find the best pair to merge
            for (int i = 0; i < remainingNodes.size(); i++) {
                for (int j = i + 1; j < remainingNodes.size(); j++) {
                    int firstNodeIdx = remainingNodes[i];
                    int secondNodeIdx = remainingNodes[j];

                    double currentSAHCost = calculateSAHCost(bvhNode[firstNodeIdx], bvhNode[secondNodeIdx]);
                    if (currentSAHCost < bestCost) {
                        bestCost = currentSAHCost;
                        bestFirst = firstNodeIdx;
                        bestSecond = secondNodeIdx;
                        listIdxFirst = i;
                        listIdxSecond = j;
                    }
                }
            }

            // Do not change the structure is important -> The indexes should be the same
            // Swap the corresponding bounding boxes and underlying leftfirst values to keep list structure intact
            if (listIdxFirst != 0) {
                std::swap(bvhNode[remainingNodes[0]], bvhNode[bestFirst]);
            }

            if (listIdxSecond != 1) {
                std::swap(bvhNode[remainingNodes[1]], bvhNode[bestSecond]);
            }

            // Create new parent node out of the best combination
            int parentIdx = nodesUsed++;

            bvhNode[parentIdx].boundingBox = bvhNode[remainingNodes[0]].boundingBox.bbUnion(
                    bvhNode[remainingNodes[1]].boundingBox);
            bvhNode[parentIdx].leftFirst = remainingNodes[0];
            bvhNode[parentIdx].triCount = 0;
            bvhNode[parentIdx].totalTriCount =
                    bvhNode[remainingNodes[0]].totalTriCount + bvhNode[remainingNodes[1]].totalTriCount;

            // Remove the first 2 combined nodes and add the parent to the list
            remainingNodes.erase(remainingNodes.begin(), remainingNodes.begin() + 2);
            remainingNodes.push_back(parentIdx);
        }
    }


    void buildBvhBottomUp(const std::vector<std::shared_ptr<Hittable>> &hittables, int sahSwitchThreshold) {
        double totalTriangleVolume = 0.0;
        double averageTriangleVolume = 0.0;

        for (const auto &hittable: hittables) {
            double triangleVolume = hittable->volume();
            totalTriangleVolume += triangleVolume;
        }

        if (numTriangles > 0) averageTriangleVolume = totalTriangleVolume / numTriangles;

        std::vector<MortonPrimitive> mortonPrims(numTriangles);
        BoundingBox globalBounds;
        std::vector<BoundingBox> primitiveBounds(numTriangles);

        // Calculate global bounding box
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        globalBounds.min = Vec3(imax, imax, imax);
        globalBounds.max = Vec3(imin, imin, imin);
        for (int i = 0; i < numTriangles; i++) {
            BoundingBox boundingBox = hittables[i]->getBoundingBox();
            primitiveBounds[i] = boundingBox;
            globalBounds = globalBounds.bbUnion(boundingBox);
        }

        const int gridResolution = determineGridResolution(averageTriangleVolume, globalBounds);

        // Compute Morton Codes
        for (int i = 0; i < numTriangles; i++) {
            const Vec3 center = primitiveBounds[i].center();
            mortonPrims[i].mortonCode = encodeMorton3(center, globalBounds, gridResolution);
            mortonPrims[i].primitiveIndex = i;
        }

        // Might be nice to check out other things but it achieves quite the nice result while being easily implemented

        // Sort primitives based on morton code
        std::sort(mortonPrims.begin(), mortonPrims.end(), [](const MortonPrimitive &a, const MortonPrimitive &b) {
            return a.mortonCode < b.mortonCode;
        });


        // Initialize leaf nodes
        int totalNodes = 2 * numTriangles + 1; // Total nodes needed (leaves + internal)
        bvhNode = new BVHNode[totalNodes];
        nodesUsed = numTriangles;

        for (int i = 0; i < numTriangles; i++) {
            int primIdx = mortonPrims[i].primitiveIndex;
            bvhNode[i].boundingBox = primitiveBounds[primIdx];
            bvhNode[i].leftFirst = i;
            bvhNode[i].triCount = 1;
            bvhNode[i].totalTriCount = 1;
            triIdx[i] = primIdx;
        }

        // Build hierarchy
        int offset = 0;
        while (numTriangles > 1) {
            if (numTriangles <= sahSwitchThreshold && split == Split::LinearSAH) {
                // Switch to SAH-based building
                // Instead of going through everything might be possible to just save the cost everytime to one of the nodes and then compare again
                // 1. Go through all remaining boxes
                // 2. Calculate every possible box combinations and their cost -> How it would be
                // 3. Merge the best one
                // 4. Continue with the rest of them until only one is left
                buildSAHNodes(offset, numTriangles);
                break;
            } else {
                int newNumTriangles = (numTriangles + 1) / 2;

                for (int i = 0; i < newNumTriangles; i++) {
                    int leftIdx = 2 * i + offset;
                    int rightIdx = std::min(leftIdx + 1, numTriangles + offset - 1);

                    int parentIdx = nodesUsed++;

                    if (leftIdx == rightIdx) {
                        bvhNode[parentIdx].boundingBox = bvhNode[leftIdx].boundingBox;
                        bvhNode[parentIdx].leftFirst = leftIdx;
                        bvhNode[parentIdx].triCount = 0;
                        bvhNode[parentIdx].totalTriCount = bvhNode[leftIdx].totalTriCount;
                    } else {
                        bvhNode[parentIdx].boundingBox = bvhNode[leftIdx].boundingBox.bbUnion(
                                bvhNode[rightIdx].boundingBox);
                        bvhNode[parentIdx].leftFirst = leftIdx;
                        bvhNode[parentIdx].triCount = 0;
                        bvhNode[parentIdx].totalTriCount =
                                bvhNode[leftIdx].totalTriCount + bvhNode[rightIdx].totalTriCount;
                    }
                }
                offset += numTriangles;
                numTriangles = newNumTriangles;
            }
        }
        rootNodeIdx = nodesUsed - 1;
    }
};

#endif //RAYTRACER_BVHBUILDER_H
