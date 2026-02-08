#include "buffers.h"
#include "query.h"
#include "scene.h"
#include "shader.h"
#include "antialias.h"

#include <cfloat>
#include <cmath>
#include <iostream>


#ifdef __CUDACC__
__global__ void __launch_bounds__(BLOCK_X * BLOCK_Y)
renderCUDA(const int numTriangles,
       int W, int H,
       int max_depth,
       const Camera cam,
       const Vec3 missColor,
       const BVHNode* __restrict__ nodes,
       const AABB* __restrict__ aabbs,
       const Triangle* __restrict__ triangles,
       const int32_t* __restrict__ triObjectIds,
       const Material* __restrict__ objectMaterials,
       const int numObjectMaterials,
       const int numRays,
       const Light* __restrict__ lights,
       const int numLights,
       Vec3* __restrict__ output)
{
    (void)numRays;
    auto block = cg::this_thread_block();
    uint2 pix_min = { block.group_index().x * BLOCK_X, block.group_index().y * BLOCK_Y };
    uint2 pix = { pix_min.x + block.thread_index().x, pix_min.y + block.thread_index().y };

    if (pix.x >= W || pix.y >= H) return;

    if (nodes == nullptr || aabbs == nullptr || triangles == nullptr || output == nullptr) {
        return;
    }

    const int pix_id = W * pix.y + pix.x;
    const Ray ray = cam.get_ray(pix.x,pix.y);

    output[pix_id] = TraceRayIterative(
        ray,
        max_depth,
        missColor,
        numTriangles,
        nodes, aabbs, triangles,
        triObjectIds, objectMaterials, numObjectMaterials,
        lights, numLights
    );
}
#endif

void render(
    const size_t numTriangles,
    int W, int H,
    const Camera cam,
    const Vec3 missColor,
    const int max_depth,
    const int spp,
    const BVHNode* __restrict__ nodes,
    const AABB* __restrict__ aabbs,
    const Triangle* __restrict__ triangles,
    const int32_t* __restrict__ triObjectIds,
    const Material* __restrict__ objectMaterials,
    const int numObjectMaterials,
    const int numRays,
    const Light* __restrict__ lights,
    const int numLights,
    Vec3* __restrict__ output)
{
#ifdef __CUDACC__
    dim3 tile_grid((W + BLOCK_X - 1) / BLOCK_X, (H + BLOCK_Y - 1) / BLOCK_Y, 1);
    dim3 block(BLOCK_X, BLOCK_Y, 1);
    CHECK_CUDA((renderCUDA<<<tile_grid, block>>>(
        numTriangles,
        W, H,
        max_depth,
        cam,
        missColor,
        nodes,
        aabbs,
        triangles,
        triObjectIds,
        objectMaterials,
        numObjectMaterials,
        numRays,
        lights,
        numLights,
        output
    )), true);
#else
    if (nodes == nullptr || aabbs == nullptr || triangles == nullptr || output == nullptr) {
        return;
    }

    const int triCount = static_cast<int>(numTriangles);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int pix_id = W * y + x;

            Vec3 col{0,0,0};

            auto offsets = jittered_samples(spp, 42u);

            for (const auto &o : offsets) {
                float px = float(x) + o.first;
                float py = float(y) + o.second;

                const Ray ray = cam.get_ray(px, py);

                col = col + TraceRayIterative(
                    ray,
                    max_depth,
                    missColor,
                    triCount,
                    nodes, aabbs, triangles,
                    triObjectIds, objectMaterials, numObjectMaterials,
                    lights, numLights
                );
            }
            output[pix_id] = col/float(spp);
        }
    }
#endif
}
