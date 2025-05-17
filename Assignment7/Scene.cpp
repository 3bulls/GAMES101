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
    auto inter = intersect(ray);
    if(!inter.happened) return Vector3f(0.0f,0.0f,0.0f);
    // hit the light source, just return the light
    if (inter.m->hasEmission()) return inter.m->getEmission();
    
    Material *m = inter.m;
    Vector3f L_dir = {0.0f,0.0f,0.0f};
    float pdf_light;
    Intersection inter_light;
    // get the direct light sample
    sampleLight(inter_light,pdf_light);
    Vector3f x_light = inter_light.coords;
    Vector3f ws = x_light-inter.coords;
    ws = ws.normalized();
    Ray lightRay(inter.coords,ws);
    auto blockedInter = intersect(lightRay);
    // the intersection p can directly receive the light
    // although the unit of distance is time, 
    // because the velocity of ray is 1 dis/time, you can compare them directly
    if(abs((x_light-inter.coords).norm()-blockedInter.distance)<0.01) {
        auto emit = inter_light.emit;
        L_dir = emit * m->eval(ray.direction,ws,inter.normal) 
                    * dotProduct(ws,inter.normal) 
                    * dotProduct(-ws,inter_light.normal)
                    / distance2(x_light,inter.coords)
                    / pdf_light;
    }

    Vector3f L_indir = {0.0f,0.0f,0.0f};
    if(get_random_float() < RussianRoulette){

        auto wi = m->sample(ray.direction,inter.normal).normalized();
        Ray iray(inter.coords,wi);
        auto L_indir_Inter = intersect(iray);
        float pdf = m->pdf(ray.direction,wi,inter.normal);
        if(pdf>EPSILON) {
            if (L_indir_Inter.happened && !L_indir_Inter.m->hasEmission()) {
                L_indir = castRay(iray,depth+1) 
                * m->eval(ray.direction,wi,inter.normal)
                * dotProduct(wi,inter.normal)
                / m->pdf(ray.direction,wi,inter.normal)
                / RussianRoulette;
            }
        }
    }

    return L_dir+L_indir;
}