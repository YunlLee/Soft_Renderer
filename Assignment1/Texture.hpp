
#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H

#include "Global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

class Texture
{
public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(u_img, v_img);
        return Eigen::Vector3f(color[0], color[1], color[3]);
    }
private:
    cv::Mat image_data;
};

#endif RASTERIZER_TEXTURE_H