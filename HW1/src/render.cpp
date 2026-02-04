#include "MeshOBJ.h"
#include "camera.h"
#include "ray.h"
#include "raytracer.h"
#include "vec3.h"
#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include "antialias.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main(int argc, char** argv)
{
    using vec3 = Vec3;
    using point3 = Vec3;

    // std::string path = "../assets/meshes/frog.obj";
    std::string path = "../assets/meshes/sphere.obj";
    if (argc >= 2) path = argv[1]; // provide mesh via command line (optional)
    std::cout << "Loading OBJ: " << path << "\n";

    MeshSOA mesh;
    if (!LoadOBJ_ToMeshSOA(path, mesh))
    {
        std::cerr << "Failed to load OBJ: " << path << "\n";
        return 1;
    }

    const size_t vertexCount = mesh.positions.size();
    
    const size_t indexCount  = mesh.indices.size();
    const size_t triCount    = indexCount / 3;

    std::cout << "Loaded OBJ: " << path << "\n";
    std::cout << "Vertices:   " << vertexCount << "\n";
    std::cout << "Triangles:  " << triCount << "\n";
    std::cout << "Has UVs:    " << (mesh.hasUVs() ? "yes" : "no") << "\n";
    std::cout << "Has Normals:" << (mesh.hasNormals() ? "yes" : "no") << "\n";
    
    const point3 camera_position{0.0, -1.0, 1.0};
    const point3 look_at{0.0, 0.15, 0.0};
    const vec3 up{0.0, 0.0, 1.0};

    const double focal_length_mm = 255.0;
    const double sensor_height_mm = 24.0; // full-frame
    const int pixel_width = 320;
    const int pixel_height = 180;

    

    camera cam(camera_position, look_at, up, focal_length_mm, sensor_height_mm, pixel_width, pixel_height);

    Light light;
    light.position = make_vec3(-3.0f, 0.0f, 1.0f);
    light.color = make_vec3(1.0f, 0.0f, 1.0f);

    std::vector<Vec3> image(pixel_width * pixel_height);
    std::string output_filename = "output.png";

    int count = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    const int samples_per_pixel = 1;
    auto offsets = jittered_samples(samples_per_pixel, 42u); 


    auto center = cam.get_center();
    for (int j = 0; j < pixel_height; j++) {
        for (int i = 0; i < pixel_width; i++) {

            Vec3 accum_color = make_vec3(0.0f, 0.0f, 0.0f);

            for (const auto &o : offsets) {

                float px = float(i) + o.first;
                float py = float(j) + o.second;


                Ray r = Ray(center, cam.get_pixel_position(px, py) - center);
                HitRecord prev;
                prev.hit = false;
                prev.t = std::numeric_limits<float>::max();
                auto color = shade(r, prev, light);

                for (int k = 0; k < indexCount; k += 3) {
                    Triangle tri;

                    tri.v0 = mesh.positions[mesh.indices[k]];
                    tri.v1 = mesh.positions[mesh.indices[k + 1]];
                    tri.v2 = mesh.positions[mesh.indices[k + 2]];

                    // need to check for normals first
                    tri.n0 = mesh.normals[mesh.indices[k]];
                    tri.n1 = mesh.normals[mesh.indices[k + 1]];
                    tri.n2 = mesh.normals[mesh.indices[k + 2]];

                    HitRecord rec = ray_intersection(r, tri);

                    if(rec.hit && rec.t < prev.t) {
                        color = shade(r, rec, light);
                        prev = rec;
                    }
                }

                accum_color = accum_color + color;
            }

            Vec3 final_color = accum_color / float(offsets.size());

            image[(j) * pixel_width + i] = final_color;
        }
    }

    std::cout << count;
    std::vector<unsigned char> png_data(pixel_width * pixel_height * 3);
    for (int k = 0; k < pixel_width * pixel_height; ++k) {
        png_data[k*3 + 0] = (unsigned char)(255.99f * image[k].x);
        png_data[k*3 + 1] = (unsigned char)(255.99f * image[k].y);
        png_data[k*3 + 2] = (unsigned char)(255.99f * image[k].z);
    }
    stbi_write_png(output_filename.c_str(), pixel_width, pixel_height, 3, png_data.data(), pixel_width * 3);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    

    std::cout << "\nTotal render time: " << elapsed.count() << " seconds\n";


    return 0;
}
