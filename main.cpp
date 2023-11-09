#include "rtweekend.h"

#include "camera.h"
#include "color.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "objects.h"


int main() {

    hittable_list world;

    auto ground_material = make_shared<lambertian>(color(0.5, 0.5, 0.5));
    world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));

    auto sun = make_shared<diffuse_light>(color(15, 15, 15));
    world.add(make_shared<sphere>(point3(-4, 1, 3), 1.5, sun));

    // Materials
    auto left_red = make_shared<lambertian>(color(1.0, 0.2, 0.2));
    auto back_green = make_shared<lambertian>(color(0.2, 1.0, 0.2));
    auto right_blue = make_shared<lambertian>(color(0.2, 0.2, 1.0));
    auto upper_orange = make_shared<lambertian>(color(1.0, 0.5, 0.0));
    auto lower_teal = make_shared<lambertian>(color(0.2, 0.8, 0.8));

    // Quads
    world.add(pyramid(point3(4, 0, 3.5), point3(3, 0, 2.5), 1.0, left_red));
    world.add(cube(point3(3, 0, -1), point3(4.5, 1.5, -2.5), back_green));
    world.add(cube(point3(-2.5, 0, 0), point3(-3.5, 1.5, -1.5), left_red));
    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
    world.add(make_shared<sphere>(point3(4, 1, 1), 0.5, material3));

    // Add lights and check shadows
    // Add phong as a lightning component -> Create light sources
    // Bounding boxes
    // Move Camera / Translate the objects
    // Make it realtime??
    // Clean code up

    camera cam;

    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 800;
    cam.samples_per_pixel = 500;
    cam.max_depth = 50;
    cam.background = color(0, 0, 0);

    cam.vfov = 20;
    cam.lookfrom = point3(13, 2, 3);
    cam.lookat = point3(0, 0, 0);
    cam.vup = vec3(0, 1, 0);

    cam.defocus_angle = 0;
    cam.focus_dist = 10.0;

    cam.render(world);

}