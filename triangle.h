#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "rtweekend.h"
#include "hittable.h"
#include "vec3.h"

class triangle : public hittable {
    public:
        point3 v0;
        point3 v1;
        point3 v2;
        // vec3 normal;
        shared_ptr<material> mat_ptr;

        triangle(){}
        //bottom left corner, bottom right, top
        triangle(point3 _v0, point3 _v1, point3 _v2, shared_ptr<material>(m))
            : v0(_v0), v1(_v1) , v2(_v2), mat_ptr(m) {

            };

        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
            // The bounding box must have non-zero width in each dimension, so pad the Z
            // dimension a small amount.
            output_box = aabb(point3(v0.x(),v0.y(), v0.z()-0.0001), point3(v1.x(),v2.y(), v2.z()-0.0001));
            return true;
        }
};
 
bool triangle::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    vec3 v0v1 = v1 - v0;
    vec3 v0v2 = v2 - v0;
    vec3 normal = cross(v0v1,v0v2);

    double denom  = dot(normal, normal);

    double ndotdir = dot(normal, r.direction());
    // std::cerr << "ndotdir: " << ndotdir << std::endl;

    // if(fabs(ndotdir) < 0.00000001){
    //     return false;
    // }

    // std::cerr << "n.x: " << normal.x() << " n.y: "<< normal.y() << " n.z "<< normal.z()<< std::endl;

    double d = dot(normal, v0);
    // std::cerr << "d: " << d << std::endl;

    double t = -(dot(normal, r.origin()) + d) / ndotdir;
    // std::cerr << "t: " << t << std::endl;

    if (t < t_min || t > t_max){
        return false;
    }



    //calc intersection point
    vec3 P = r.at(t);


// Step 2: inside-outside test
    vec3 C;  //vector perpendicular to triangle's plane 
 
    // edge 0
    vec3 edge0 = v1 - v0; 
    vec3 vp0 = P - v0; 
    C = cross(edge0,vp0); 
    if (dot(normal,C) < 0) return false;  //P is on the right side 
 
    // edge 1
    vec3 edge1 = v2 - v1; 
    vec3 vp1 = P - v1; 
    C = cross(edge1,vp1); 
    if (dot(normal,C) < 0)  return false;  //P is on the right side 
 
    // edge 2
    vec3 edge2 = v0 - v2; 
    vec3 vp2 = P - v2; 
    C = cross(edge2,vp2); 
    if (dot(normal,C) < 0) return false;  //P is on the right side; 
    

    // rec.u = rec.u / denom;
    // rec.v = rec.v / denom;
    rec.t = t;
    rec.set_face_normal(r, unit_vector(normal));
    rec.mat_ptr = mat_ptr;
    rec.p = P;

    return true;  //this ray hits the triangle 
}

#endif