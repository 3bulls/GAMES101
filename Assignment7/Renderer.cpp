//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>

#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

void para(std::vector<Vector3f> &framBuffer,
    int startLine, int endLine,
    const Scene &scene,
    int *sharedCount,
    std::mutex &mtx,
    float scale,
    float imageAspectRatio,
    const Vector3f &eye_pos
  )
{
    int spp = 1024;
    std::cout<<"startline:"<<startLine<<std::endl;
    for (uint32_t j = startLine; j < endLine; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            int m = scene.width*j+i;

            for (int k = 0; k < spp; k++){
                // generate primary ray direction
                float x = (2 * (i + ((float)k)/spp) / (float)scene.width - 1) *
                imageAspectRatio * scale;
                float y = (1 - 2 * (j + ((float)k)/spp) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                framBuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            }
        }
        mtx.lock();
        (*sharedCount)++;
        UpdateProgress( (*sharedCount)/ (float)scene.height);
        mtx.unlock();
    }
}



// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    /* single thread*/
    // int m = 0;
    // change the spp value to change sample ammount
    // int spp = 16;
    // std::cout << "SPP: " << spp << "\n";
    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    /*multi thread*/
    int thread_num = 32;
    int linesPerTh = scene.height/thread_num;
    int shared_counter = 0;
    std::mutex mtx;

    std::vector<std::thread> rays;
    for(int i=0;i<thread_num;i++){
        rays.push_back(std::thread(para,std::ref(framebuffer),
                                        i*linesPerTh, 
                                        std::min(scene.height,(i+1)*linesPerTh),
                                        std::ref(scene),
                                        &shared_counter,
                                        std::ref(mtx),
                                        scale,
                                        imageAspectRatio,
                                        eye_pos));
    }
    for(std::thread &t:rays){
        t.join();
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("multiThread1024NASS.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}


