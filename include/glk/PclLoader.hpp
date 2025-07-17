
#pragma once

#include <iostream>
#include <vector>
#include <GL/gl3w.h>
#include <string>
#include <mutex>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <glk/glsl_shader.hpp>

class PclLoader
{
private:
    // color for the terminals
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";
    // pcd map
    GLuint _vao = 0, _vbo = 0;
    int pcd_num_points = 0;
    std::string pcd_frame_id_ = "map";
    bool has_pcd_setted = false;

public:
    PclLoader(/* args */);
    ~PclLoader();
    void PointCloudBuffer(const std::string &cloud_filename, std::string pcd_frame_id);
    void renderpcl(glk::GLSLShader &shader, std::mutex &tf_mutex, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
    bool isLoaded() const;
    size_t getPointCount() const;
};
