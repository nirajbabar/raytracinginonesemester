#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <cmath>
#include <iostream>
#include "ray.h"

struct Light {
    Vec3 position;
    Vec3 color;
};

HYBRID_FUNC Vec3 clamp(Vec3& color) {
    if (color.x > 1.0) color.x = 1.0;
    if (color.y > 1.0) color.y = 1.0;
    if (color.z > 1.0) color.z = 1.0;

    return color;
}

HYBRID_FUNC Vec3 shade(const Ray& r, const HitRecord& rec, const Light& light) {
    if (!rec.hit) {
        Vec3 unit_dir = unit_vector(r.direction());
        float t = 0.5f * (unit_dir.z + 1.0f);
        return make_vec3(1.0f, 1.0f, 1.0f)*(1.0f-t) + make_vec3(0.5f, 0.7f, 1.0f)*t;
    }

    // Ambient
    Vec3 ambient = rec.mat.albedo * 0.1f;

    // Diffuse
    Vec3 lightDir = unit_vector(light.position - rec.p);
    float diff = fmaxf(dot(rec.normal, lightDir), 0.0f);
    Vec3 diffuse = (rec.mat.albedo * light.color) * diff;

    // Specular
    Vec3 specular = make_vec3(0,0,0);
    if (rec.mat.type == MAT_METAL) {
        Vec3 viewDir = unit_vector(r.origin() - rec.p);
        Vec3 halfDir = unit_vector(lightDir + viewDir);
        float spec = powf(fmaxf(dot(rec.normal, halfDir), 0.0f), rec.mat.shininess);
        specular = light.color * spec;
    }
    
    auto final_colour = ambient + diffuse + specular;

    return clamp(final_colour);
}

#endif