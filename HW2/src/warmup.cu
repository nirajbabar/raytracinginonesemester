#include "warmup.h"
#include "bvh.h"
#include "MeshOBJ.h"
#include "buffers.h"
#include <iostream>
#include <vector>

void warmupGPU() {
#ifdef __CUDACC__
    std::cout << "[Warmup] initializing GPU resources...\n";
    
    // 1. Create dummy mesh (single triangle)
    Mesh dummyMesh;
    dummyMesh.positions.push_back(make_vec3(0,0,0));
    dummyMesh.positions.push_back(make_vec3(1,0,0));
    dummyMesh.positions.push_back(make_vec3(0,1,0));
    dummyMesh.indices.push_back(0);
    dummyMesh.indices.push_back(1);
    dummyMesh.indices.push_back(2);
    
    size_t P = 1;

    // 2. Allocate Device Memory for BVH
    // calling required<BVHState> works because template is in header
    size_t size = required<RayTracer::BVHState>(P);
    char* d_buffer = nullptr;
    if (cudaMalloc(&d_buffer, size) != cudaSuccess) {
        return;
    }
    
    // We replicate the obtain logic to be self-contained and avoid linking issues
    // with BVHState::fromChunk if it's defined in another translation unit.
    RayTracer::BVHState state;
    char* walker = d_buffer;
    RayTracer::obtain(walker, state.Nodes, 2 * P - 1, 128);
    RayTracer::obtain(walker, state.AABBs, 2 * P - 1, 128);
    
    // 3. Run Pipeline
    AccStruct::BVH bvh;
    // calculateAABBs internally allocates temporary memory for positions/indices
    // and runs the computeAABB kernel.
    bvh.calculateAABBs(dummyMesh, state.AABBs);

    // Thrust Reduce (instantiates thrust::reduce for AABB)
    AABB default_aabb;
    AABB sceneBox = thrust::reduce(
        thrust::device_pointer_cast(state.AABBs + (P - 1)),
        thrust::device_pointer_cast(state.AABBs + (2*P - 1)),
        default_aabb,
        [] __device__ __host__ (const AABB& lhs, const AABB& rhs) {
            return AABB::merge(lhs, rhs);
        });

    thrust::device_vector<unsigned int> indices(P);
    thrust::sequence(indices.begin(), indices.end());

    // Build (exercises sort, gather, etc.)
    bvh.buildBVH(
        state.Nodes,
        state.AABBs,
        sceneBox,
        &indices,
        (int)P
    );
    
    cudaDeviceSynchronize();
    cudaFree(d_buffer);
    std::cout << "[Warmup] Done.\n";
#endif
}
