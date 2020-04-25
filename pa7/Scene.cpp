//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // find intersection
    Intersection p_inter = Scene::intersect(ray);

    // use black as the background color
    Vector3f hitColor(0.);

    // no intersection with any object, black
    if (!p_inter.happened) {
        return hitColor;
    }

    // hit light source, white
    if (p_inter.emit.norm() > 0)
    {
        hitColor = Vector3f(1);
        return hitColor;
    }

    Vector3f p = p_inter.coords;
    Vector3f N = p_inter.normal;
    Material *m = p_inter.m;
    Vector3f wo = - ray.direction;

    // direct illumination
    Vector3f L_dir = 0.0;
    {
        Intersection intersection;
        float pdf_light = 0.0;
        sampleLight(intersection, pdf_light);
        Vector3f x = intersection.coords;
        Vector3f ws = normalize(x - p);

        // check if it is blocked in the middle
        Ray shoot_ray(p, ws);
        Intersection ray_inter = Scene::intersect(shoot_ray);
        if ((ray_inter.coords - x).norm() < 0.1) {  // not blocked
            Vector3f NN = intersection.normal;

            // compute L_dir
            Vector3f L_i = intersection.emit; // intensity
            Vector3f f_r = m->eval(wo, ws, N);
            float cos_theta = dotProduct(ws, N);
            float cos_theta_x = dotProduct(- ws, NN);
            float dis = (x - p).norm() * (x - p).norm();  // distance
            L_dir = L_i * f_r * cos_theta * cos_theta_x / dis / pdf_light;
        }
    }
    
    // indirect illumination
    Vector3f L_indir = 0.0;
    {
        float p_RR = get_random_float();
        if (p_RR < RussianRoulette) {
            Vector3f wi = m->sample(wo, N);
            Ray secondary_ray(p, wi);
            Vector3f emit_indir = castRay(secondary_ray, depth + 1);
            L_indir = emit_indir * m->eval(wi, wo, N) * dotProduct(wi, N) / m->pdf(wi, wo, N) / RussianRoulette;
        }
    }
    

    hitColor = L_dir + L_indir;
    return hitColor;
}