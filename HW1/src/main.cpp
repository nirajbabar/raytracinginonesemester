#include <array>
#include <iostream>
#include <vector>
#include <sstream>

#include <glm/glm.hpp>

#include "camera.h"
#include "visualization.h"

#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

using point3d = glm::dvec3;
using vec3d   = glm::dvec3;
using vec3f   = glm::vec3;

// Build a list of pixel world positions (as glm::vec3 for Polyscope)
static std::vector<vec3f> build_pixel_points(const camera& cam) {
    std::vector<vec3f> pixels;
    pixels.reserve(static_cast<size_t>(cam.pixel_width) * static_cast<size_t>(cam.pixel_height));

    for (int j = 0; j < cam.pixel_height; ++j) {
        for (int i = 0; i < cam.pixel_width; ++i) {
            point3d p = cam.get_pixel_position(i, j);
            pixels.emplace_back(viz::to_glm_vec3(p));
        }
    }
    return pixels;
}

// Build the ray network: one camera node + one pixel node per pixel, edges from camera -> pixel
static void build_ray_network(const camera& cam,
                              std::vector<vec3f>& out_nodes,
                              std::vector<std::array<size_t, 2>>& out_edges)
{
    const point3d cam_center = cam.get_center();
    out_nodes.clear();
    out_edges.clear();

    const size_t cam_node_idx = 0;
    out_nodes.reserve(1 + static_cast<size_t>(cam.pixel_width) * static_cast<size_t>(cam.pixel_height));
    out_nodes.emplace_back(viz::to_glm_vec3(cam_center)); // single camera node

    // Add pixel nodes and edges (camera -> pixel)
    for (int j = 0; j < cam.pixel_height; ++j) {
        for (int i = 0; i < cam.pixel_width; ++i) {
            point3d p = cam.get_pixel_position(i, j);
            size_t pixel_node_idx = out_nodes.size();
            out_nodes.emplace_back(viz::to_glm_vec3(p));
            out_edges.push_back({cam_node_idx, pixel_node_idx});
        }
    }
}

// Register data with Polyscope (point clouds and curve network)
static void register_with_polyscope(const std::vector<vec3f>& camera_point,
                                    const std::vector<vec3f>& pixel_points,
                                    const std::vector<vec3f>& ray_nodes,
                                    const std::vector<std::array<size_t,2>>& ray_edges)
{
    constexpr bool radius_is_relative = true;

    auto* cam_cloud = polyscope::registerPointCloud("Camera Position", camera_point);
    cam_cloud->setPointRadius(0.01, /*isRelative=*/radius_is_relative);

    auto* pixel_cloud = polyscope::registerPointCloud("Pixel Positions", pixel_points);
    pixel_cloud->setPointRadius(0.003, /*isRelative=*/radius_is_relative);

    auto* ray_network = polyscope::registerCurveNetwork("Camera Rays", ray_nodes, ray_edges);
    ray_network->setRadius(0.001, /*isRelative=*/radius_is_relative);
}

static void print_pixel_positions(const camera& cam) {
    std::ostringstream oss;
    for (int j = 0; j < cam.pixel_height; ++j) {
        for (int i = 0; i < cam.pixel_width; ++i) {
            point3d p = cam.get_pixel_position(i, j);
            oss << "Pixel (" << i << ", " << j << "): ("
                << p.x << ", " << p.y << ", " << p.z << ")\n";
        }
    }
    std::cout << oss.str();
}

int main() {
    // World units: meters
    const point3d camera_position{0.0, 0.0, 0.3};
    const point3d look_at{0.0, 4.0, 0.3};
    const vec3d up{0.0, 0.0, 1.0};

    const double focal_length_mm = 500.0;
    const double sensor_height_mm = 200.0; // full-frame
    const int pixel_width = 80;
    const int pixel_height = 45;

    camera cam(camera_position, look_at, up, focal_length_mm, sensor_height_mm, pixel_width, pixel_height);

    // Initializing Polyscope using visualization helper 
    viz::init_polyscope_zup();

    // Camera point cloud (single point)
    const point3d cam_center = cam.get_center();
    std::vector<vec3f> camera_point;
    camera_point.emplace_back(viz::to_glm_vec3(cam_center));

    // Pixel points
    const std::vector<vec3f> pixel_points = build_pixel_points(cam);

    // Ray network
    std::vector<vec3f> ray_nodes;
    std::vector<std::array<size_t,2>> ray_edges;
    build_ray_network(cam, ray_nodes, ray_edges);

    // Print pixel coordinates to console
    print_pixel_positions(cam);

    // Register with Polyscope
    register_with_polyscope(camera_point, pixel_points, ray_nodes, ray_edges);

    // Add axes and show UI
    constexpr double axis_length = 0.5;
    viz::AxesVizOptions axes_opt;
    axes_opt.radius_is_relative = true;
    axes_opt.axis_radius = 0.005;
    viz::register_axes(axis_length, glm::vec3(0.0f, 0.0f, 0.0f), axes_opt);

    viz::show();

    return 0;
}
