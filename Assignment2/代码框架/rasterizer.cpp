// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool superSamplingInsideTriangle(int x, int y, const Vector3f* _v)
{
    Vector3f P(x,y,0.0f);
    Vector3f A = Vector3f(_v[0].x(), _v[0].y(), 0.0f);
    Vector3f B = Vector3f(_v[1].x(), _v[1].y(), 0.0f);
    Vector3f C = Vector3f(_v[2].x(), _v[2].y(), 0.0f);
    float z1 = (A-P).cross(A-B).z();
    float z2 = (B-P).cross(C-B).z();
    float z3 = (C-P).cross(A-C).z();
    if (z1>0&&z2>0&&z3>0){
        return true;
    } else if (z1<0&&z2<0&&z3<0){
        return true;
    } else {
        return false;
    }
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P(x + 0.5f, y + 0.5f, 0.0f);  
    Vector3f A = Vector3f(_v[0].x(), _v[0].y(), 0.0f);
    Vector3f B = Vector3f(_v[1].x(), _v[1].y(), 0.0f);
    Vector3f C = Vector3f(_v[2].x(), _v[2].y(), 0.0f);
    
    float z1 = (A-P).cross(B-A).z();
    float z2 = (B-P).cross(C-B).z();
    float z3 = (C-P).cross(A-C).z();

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            // MYTODO: why three times?
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        // rasterize_triangle(t);
        superSamplingRasterizeTriangle(t);
    }
}

void rst::rasterizer::superSamplingRasterizeTriangle(const Triangle& t){
    auto v = t.toVector4();
    // TODO : Find out the bounding box of current triangle.
    float xstart=std::numeric_limits<float>::max();
    float ystart=std::numeric_limits<float>::max();
    float xend=std::numeric_limits<float>::min();
    float yend=std::numeric_limits<float>::min();
    for (auto i:v) {
        xstart = xstart>floor(i.x()) ? floor(i.x()) : xstart;
        xend = xend<ceil(i.x()) ? ceil(i.x()) : xend;
        ystart = ystart>floor(i.y()) ? floor(i.y()) : ystart;
        yend = yend<ceil(i.y()) ? ceil(i.y()) : yend;
    }

    for (float x= xstart;x<xend;x+=1){
        for(float y= ystart;y<yend;y+=1){
            Vector2f pointArr[4] = {
                {x+0.25,y+0.25},
                {x+0.25,y+0.75},
                {x+0.75,y+0.25},
                {x+0.75,y+0.75},
            } ;
            float depthAll = 0.0f;
            int count = 0;
            for(int i=0;i<4;i++){
                if(superSamplingInsideTriangle(pointArr[i].x(),pointArr[i].y(),t.v)){
                    auto[alpha, beta, gamma] = computeBarycentric2D(pointArr[i].x(), pointArr[i].y(), t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = (alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w())*w_reciprocal;
                    count++;
                    depthAll += z_interpolated;
                }
            }
            if (count>0) {
                std::cout<<"test count"<<count<<std::endl;
                if (depthAll/count < depth_buf[x*width+y]) {
                    depth_buf[x*width+y] = depthAll/count;
                    Vector3f point = {700-x,700-y,depthAll/count};
                    // this triangle color is one of the vectex
                    Vector3f colorBuf = t.getColor();
                    colorBuf = colorBuf*(float)count/4.0;
                    set_pixel(point,colorBuf);
                }
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    int xstart=INT_MAX,ystart=INT_MAX,xend=INT_MIN,yend=INT_MIN;
    for (auto i:v) {
        xstart = xstart>floor(i.x()) ? floor(i.x()) : xstart;
        xend = xend<ceil(i.x()) ? ceil(i.x()) : xend;
        ystart = ystart>floor(i.y()) ? floor(i.y()) : ystart;
        yend = yend<ceil(i.y()) ? ceil(i.y()) : yend;
    }

    // for debug
        // x start:210
        // x end:490
        // y start:210
        // y end:350
        // x start:154
        // x end:406
        // y start:266
        // y end:406
    // std::cout<<"x start:"<<xstart<<std::endl;
    // std::cout<<"x end:"<<xend<<std::endl;
    // std::cout<<"y start:"<<ystart<<std::endl;
    // std::cout<<"y end:"<<yend<<std::endl;

    // iterate through the pixel and find if the current pixel is inside the triangle
     // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();

    for (int x = xstart;x<xend;x++){
        for (int y = ystart;y<yend;y++) {
            if (insideTriangle(x,y,t.v)){
                auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = (alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w())*w_reciprocal;
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if (z_interpolated <= depth_buf[x*width+y]) {
                    depth_buf[x*width+y] = z_interpolated;
                    Vector3f pixel = {700-x,700-y,z_interpolated};
                    set_pixel(pixel,t.getColor());
                }
            }
        }
    }

    
    // for(int i = xstart;i < xend;++i)
    // {
    //     for(int j = ystart;j < yend;++j)
    //     {
    //         if(insideTriangle(i,j,t.v))
    //         {
    //             auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;
    //             if(z_interpolated <= this->depth_buf[i * width + j])
    //             {
    //                 this->depth_buf[i * width + j] = z_interpolated;
    //                 set_pixel(Eigen::Vector3f(700-i,700-j,z_interpolated),t.getColor());
    //             }
    //         }
    //     }
    // }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on