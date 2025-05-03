#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Vector3f axis, float angle);

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation;
    rotation << cos(rotation_angle/180*M_PI),-sin(rotation_angle/180*M_PI),0,0
             , sin(rotation_angle/180*M_PI),cos(rotation_angle/180*M_PI),0,0
             , 0,0,1,0
             , 0,0,0,1;

    // use my rotate
    Eigen::Vector3f zaxis(0.0f,0.0f,1.0f);
    Eigen::Matrix4f rotation2 = get_rotation(zaxis,rotation_angle);
    // model = rotation*model;
    model = rotation2 * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float t = tan(eye_fov/2/180*M_PI)*zNear;
    float r = t*aspect_ratio;
    Eigen::Matrix4f orthographicM;
    Eigen::Matrix4f orthoTranslate;
    Eigen::Matrix4f orthoSacle;
    Eigen::Matrix4f squash;

    orthoTranslate << 1,0,0,0
                    ,0,1,0,0
                    ,0,0,1,-(zFar+zNear)/2
                    ,0,0,0,1;
    orthoSacle << 1/r,0,0,0
                ,0,1/t,0,0
                ,0,0,2/(zNear-zFar),0
                ,0,0,0,1;

    squash << zNear,0,0,0,
            0,zNear,0,0,
            0,0,zNear+zFar,-zNear*zFar,
            0,0,1,0;
    projection = orthoSacle*orthoTranslate*squash;

    return projection;
}


Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float xvalue = axis[0];
    float yvalue = axis[1];
    float zvalue = axis[2];
    float yangle = atan(xvalue/zvalue);
    float xangle = atan(yvalue/zvalue);

    Eigen::Matrix4f  RX;
    Eigen::Matrix4f  RY;
    Eigen::Matrix4f  RZ;
    RY << cos(yangle),0,sin(yangle),0
        ,0,1,0,0
        ,-sin(yangle),0,cos(yangle), 0
        ,0,0,0,1;

    RX << 1,0,0,0
        ,0,cos(xangle),-sin(xangle),0
        ,0,sin(xangle),cos(xangle), 0
        ,0,0,0,1;

    RZ << cos(angle),-sin(angle),0,0
        ,sin(angle),cos(angle),0,0
        ,0,0,1,0
        ,0,0,0,1;
    return RX*RY*RZ;

    /* Method2 Rodrige rotation Eq */

    // auto MatrixI = Eigen::Matrix3f::Identity();
    // float angle_pi = angle*M_PI/180;
    // Eigen::Matrix3f MatrixM;
    // MatrixM << 0,-axis[2],axis[1],
    //             axis[2],0,-axis[0],
    //             -axis[1],axis[0],0;

    // Eigen::Matrix3f R = cos(angle_pi)*MatrixI 
    //                     + (1-cos(angle_pi))*(axis*axis.transpose())
    //                     + sin(angle_pi)*MatrixM;
    // Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    // Result.block<3,3>(0,0) = R;
    // return Result;
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // rasterization. 1. load all vertexs
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
