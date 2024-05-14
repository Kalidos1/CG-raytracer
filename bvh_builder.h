#ifndef RAYTRACER_BVH_BUILDER_H
#define RAYTRACER_BVH_BUILDER_H


#include <vector>
#include <memory>
#include "vec3.h"
#include "hittable.h"

struct BVHNode {
    BoundingBox boundingBox;
    int leftFirst{}, triCount{};

    [[nodiscard]] bool isLeaf() const { return triCount > 0; }
};

class BVHBuilder {
public:
    explicit BVHBuilder(int numTriangles) : triIdx(new int[numTriangles]), bvhNode(new BVHNode[2 * numTriangles]),
                                            rootNodeIdx(0),
                                            nodesUsed(1), numTriangles(numTriangles) {}
// Destroys both arrays, check when to call to free up memory
//    ~BVHBuilder() {
//        delete[] triIdx;
//        delete[] bvhNode;
//    }

    void build_bvh(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        for (int i = 0; i < numTriangles; i++) triIdx[i] = i;

        BVHNode &root = bvhNode[rootNodeIdx];
        root.leftFirst = 0, root.triCount = numTriangles;
        update_node_bounds(rootNodeIdx, hittables);
        subdivide(rootNodeIdx, hittables);
    }

    void intersect_bvh(Ray &ray, std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) {
        intersect_bvh(ray, rootNodeIdx, hittables, hit_object);
    }

    void
    intersect_bvh(Ray &ray, const int nodeIdx, std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) {
        BVHNode &node = bvhNode[nodeIdx];
        if (!node.boundingBox.intersect(ray)) return;

        if (node.isLeaf()) {
            for (int i = 0; i < node.triCount; i++) {
                int objIndex = triIdx[node.leftFirst + i];
                if (hittables[objIndex]->intersect(ray)) {
                    hit_object = objIndex;
                }
            }
        } else {
            intersect_bvh(ray, node.leftFirst, hittables, hit_object);
            intersect_bvh(ray, node.leftFirst + 1, hittables, hit_object);
        }
    }

    [[nodiscard]] BVHNode get_bvh_node(int index) const {
        return bvhNode[index];
    }

    [[nodiscard]] int get_root_node_idx() const {
        return rootNodeIdx;
    }

private:
    int *triIdx;
    BVHNode *bvhNode;
    int rootNodeIdx, nodesUsed, numTriangles;

    void update_node_bounds(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BVHNode &node = bvhNode[nodeIdx];
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        node.boundingBox.min = vec3(imax, imax, imax);
        node.boundingBox.max = vec3(imin, imin, imin);
        for (int first = node.leftFirst, i = 0; i < node.triCount; i++) {
            const int leafTriIdx = triIdx[first + i];
            const BoundingBox &bounding_box = hittables[leafTriIdx]->get_bounding_box();
            node.boundingBox.min = min(node.boundingBox.min, bounding_box.min);
            node.boundingBox.max = max(node.boundingBox.max, bounding_box.max);
        }
    }

    void subdivide(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        //terminate recursion
        BVHNode &node = bvhNode[nodeIdx];
        if (node.triCount <= 2) return;

        //determine split axis and position
        vec3 extent = node.boundingBox.max - node.boundingBox.min;
        int axis = 0;
        if (extent.y() > extent.x()) axis = 1;
        if (extent.z() > extent[axis]) axis = 2;
        double splitPos = node.boundingBox.min[axis] + extent[axis] * 0.5f;

        // in-place partition
        int i = node.leftFirst;
        int j = i + node.triCount - 1;
        while (i <= j) {
            if (hittables[triIdx[i]]->calculate_center()[axis] < splitPos) i++;
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
        update_node_bounds(leftChildIdx, hittables);
        update_node_bounds(rightChildIdx, hittables);

        //recursive call
        subdivide(leftChildIdx, hittables);
        subdivide(rightChildIdx, hittables);
    }

    static bool intersect_aabb(const Ray &ray, const vec3 bmin, const vec3 bmax) {
        double tx1 = (bmin.x() - ray.origin.x()) / ray.direction.x(), tx2 =
                (bmax.x() - ray.origin.x()) / ray.direction.x();
        double tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
        double ty1 = (bmin.y() - ray.origin.y()) / ray.direction.y(), ty2 =
                (bmax.y() - ray.origin.y()) / ray.direction.y();
        tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
        double tz1 = (bmin.z() - ray.origin.z()) / ray.direction.z(), tz2 =
                (bmax.z() - ray.origin.z()) / ray.direction.z();
        tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
        return tmax >= tmin && tmin < ray.t && tmax > 0;
    }
};

#endif //RAYTRACER_BVH_BUILDER_H
