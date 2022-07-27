#include "rtweekend.h"

#include "box.h"
#include "bvh.h"
#include "camera.h"
#include "color.h"
#include "hittable_list.h"
#include "material.h"
#include "moving_sphere.h"
#include "sphere.h"
#include "triangle.h"
#include "texture.h"

#include "learnply.h"
#include "learnply_io.h"

#include <iostream>

hittable_list random_scene(){
    hittable_list world;

    // auto ground_material = make_shared<lambertian>(color(0.3,0.3,0.3));
    // world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, ground_material));
    auto checker = make_shared<checker_texture>(color(0.2,0.3,0.1), color(0.9,0.9,0.9));
    world.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(checker)));

    for (int a= -11; a< 11; a++){
        for(int b = -11; b< 11; b++){
            auto choose_mat = random_double();
            point3 center(a + 0.9 * random_double(), 0.2, b + 0.9*random_double());

            if((center - vec3(4, 0.2, 0)).length() > 0.9){
                shared_ptr<material> sphere_material;

                if(choose_mat <0.8){
                    //diffue
                    auto albedo = color::random() * color::random();
                    sphere_material = make_shared<lambertian>(albedo);
                    auto center2 = center + vec3(0,random_double(0,.5), 0);
                    world.add(make_shared<moving_sphere>(
                        center, center2, 0.0, 1.0, 0.2, sphere_material));
                }else if(choose_mat < 0.95){
                    //metal
                    auto albedo = color::random(0.5, 1);
                    auto fuzz = random_double(0, 0.5);
                    sphere_material = make_shared<metal>(albedo, fuzz);

                    world.add(make_shared<sphere>(center, 0.2, sphere_material));

                }
                else{
                    //glass
                    sphere_material = make_shared<dielectric>(1.5);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    auto material1 = make_shared<dielectric>(1.5);
    world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

    auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
    world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
    world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

    return world;
}
hittable_list two_spheres() {
    hittable_list objects;

    auto checker = make_shared<checker_texture>(color(0.2,0.3,0.1), color(0.9,0.9,0.9));
    objects.add(make_shared<sphere>(point3(0,-10, 0), 10, make_shared<lambertian>(checker)));
    objects.add(make_shared<sphere>(point3(0,10, 0), 10, make_shared<lambertian>(checker)));
    return objects;
}
hittable_list triangles(){
    hittable_list objects;
    // objects.add(make_shared<sphere>(point3(0,0, 0), 0.5, make_shared<lambertian>(color(0.2,0.3,0.1))));
    objects.add(make_shared<triangle>(point3(-1, -1, 0),point3(1, -1, 0),point3(0,  1, 0), make_shared<lambertian>(color(0.3,0.3,0.3))));

    auto light = make_shared<diffuse_light>(color(5, 5, 5));
    objects.add(make_shared<xz_rect>(0, 5, 0, 3, 5, light));

    return objects;
}
hittable_list simple_light() {
    hittable_list objects;


    auto pertext = make_shared<checker_texture>(color(0.2,0.3,0.1), color(0.9,0.9,0.9));
    objects.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
    objects.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

    auto light = make_shared<diffuse_light>(color(5, 5, 5));
    objects.add(make_shared<xz_rect>(0, 5, 0, 3, 5, light));
    objects.add(make_shared<yz_rect>(0, 5, 0, 5, 0, light));

    auto difflight = make_shared<diffuse_light>(color(4,4,4));
    objects.add(make_shared<xy_rect>(3, 7, 1, 5, -2, difflight));

    return objects;
}
hittable_list cornell_box() {
    hittable_list objects;

    // auto red   = make_shared<lambertian>(color(.90, .34, .39));
    // auto white = make_shared<lambertian>(color(.30, .33, .35));
    // auto green = make_shared<lambertian>(color(.36, .34, .28));
    // auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<xz_rect>(213, 343, 227, 332, 554, light));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

    // objects.add(make_shared<box>(point3(130, 0, 65), point3(295, 165, 230), white));
    // objects.add(make_shared<box>(point3(265, 0, 295), point3(430, 330, 460), white));
    shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0), point3(165, 330, 165), white);
    box1 = make_shared<rotate_y>(box1,15);
    box1 = make_shared<translate>(box1, vec3(265, 0, 295));
    objects.add(box1);
    shared_ptr<hittable> box2 = make_shared<box>(point3(0,0,0), point3(165, 165, 165), white);
    box1 = make_shared<rotate_y>(box1,-45);
    box1 = make_shared<translate>(box1, vec3(130, 0, 65));
    objects.add(box2);

    return objects;
}
hittable_list test_box() {

    hittable_list objects;
    auto checker = make_shared<checker_texture>(color(0.2,0.3,0.1), color(0.9,0.9,0.9));
    objects.add(make_shared<sphere>(point3(0, -1000, 0), 1000, make_shared<lambertian>(checker)));

    // auto red   = make_shared<lambertian>(color(.90, .34, .39));
    // auto white = make_shared<lambertian>(color(.30, .33, .35));
    // auto green = make_shared<lambertian>(color(.36, .34, .28));
    // auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    objects.add(make_shared<yz_rect>(0, 50, 0, 50, 50, green));
    objects.add(make_shared<yz_rect>(0, 50, 0, 50, 0, red));
    objects.add(make_shared<xz_rect>(213, 343, 227, 332, 554, light));
    objects.add(make_shared<xz_rect>(0, 50, 0, 50, 50, white));
    objects.add(make_shared<xz_rect>(0, 50, 0, 50, 0, white));
    objects.add(make_shared<xy_rect>(0, 50, 0, 50, 50, white));


    return objects;
}

hittable_list poly_mesh(){
    hittable_list triangles;

    Polyhedron* poly;

    FILE* this_file = fopen("../model3d/tetrahedron.ply", "r");
    if (this_file == NULL) { return triangles; }
    poly = new Polyhedron(this_file);
    fclose(this_file);

    poly->initialize();
    poly->finalize();

    for (int i = 0; i < poly->ntris(); i++)
	{
        triangles.add(make_shared<triangle>(poly->tlist[i]->verts[0]->pos, poly->tlist[i]->verts[1]->pos, poly->tlist[i]->verts[2]->pos, make_shared<lambertian>(color(0.3,0.3,0.3))));
    }
    return triangles;
}


color ray_color(const ray& r, const color& background,const hittable& world, int depth) {
    hit_record rec;

    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0)
        return color(0,0,0);


    if (!world.hit(r, 0.001, infinity, rec)) 
        return background;
    ray scattered;
    color attenuation;
    color emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);

    if(!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
        return emitted;
    return emitted + attenuation * ray_color(scattered, background, world, depth -1);

    

    // vec3 unit_direction = unit_vector(r.direction());
    // auto t = 0.5*(unit_direction.y() + 1.0);
    // return (1.0-t)*color(1.0, 1.0, 1.0) + t*color(0.5, 0.7, 1.0);
}

int main(){


    // Image

    auto aspect_ratio = 16.0 / 9.0;
    int image_width = 400;
    int samples_per_pixel = 50;
    int max_depth = 10;

    //World
    hittable_list world;

    point3 lookfrom;
    point3 lookat;
    auto vfov = 40;
    auto aperture = 0.0;
    color background(0,0,0);

    switch (0)
    {


    case 1:
        world = random_scene();
        background = color(0.7, 0.8, 1.00);
        lookfrom = point3(13,2,3);
        lookat = point3(0,0,0);
        vfov = 20.0;
        aperture = 0.1;
        break;
    
    case 2:
        world = two_spheres();
        background = color(0.7, 0.8, 1.00);
        lookfrom = point3(13,2,3);
        lookat = point3(0,0,0);
        vfov = 20.0;
        break;

    case 3:
        world = triangles();
        background = color(0, 0, 0);
        lookfrom = point3(0,10,10);
        lookat = point3(0,0,0);
        break;


    case 4:
        world = simple_light();
        samples_per_pixel = 100;
        background = color(0.0,0.0,0.0);
        lookfrom = point3(26,3,6);
        lookat = point3(0,3,0);
        vfov = 20.0;
        break;

    case 5:
        world = cornell_box();
        background = color(0.5, 0.5, 0.5);

        aspect_ratio = 1.0;
        image_width = 400;
        samples_per_pixel = 10;
        lookfrom = point3(278, 278, -2000);
        lookat = point3(278, 278, 0);
        vfov = 40.0;
        break;
        
    
    case 6:
        world = test_box();
        background = color(0.7, 0.8, 1.00);
        aspect_ratio = 1.0;
        image_width = 200;
        samples_per_pixel = 10;
        lookfrom = point3(25,25,-150);
        lookat = point3(25,25,0);
        vfov = 40.0;
        break;
    default:

    case 7:
        world = poly_mesh();
        background = color(0.0,0.0,0.0);
        lookfrom = point3(5,5,5);
        lookat = point3(0,0,0);
        break;
    }




    //Camera
    vec3 vup(0,1,0);
    // (lookfrom-lookat).length()
    auto dist_to_focus = 10.0;
    int image_height = static_cast<int>(image_width / aspect_ratio);


    camera cam(lookfrom, lookat ,vup , 20, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);

    // Render
    std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";

    for (int j = image_height-1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            color pixel_color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s) {
                auto u = (i + random_double()) / (image_width-1);
                auto v = (j + random_double()) / (image_height-1);
                ray r = cam.get_ray(u, v);
                pixel_color += ray_color(r, background, world, max_depth);
            }
            write_color(std::cout, pixel_color, samples_per_pixel);
        }
    }

    std::cerr << "\nDone.\n";
}