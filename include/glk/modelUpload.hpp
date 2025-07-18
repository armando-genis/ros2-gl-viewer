
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

#include <glk/glsl_shader.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>

struct PlyMesh
{
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0; // Element buffer object
    size_t vertex_count = 0;
    size_t index_count = 0;       // Total number of indices
    std::string frame_id = "map"; // Default frame ID
};

class modelUpload
{
private:
    /* data */
public:
    modelUpload(/* args */);
    ~modelUpload();
    PlyMesh loadPlyBinaryLE(const std::string &path);
    void renderMesh(const PlyMesh &mesh,
                    glk::GLSLShader &shader,
                    std::mutex &tf_mutex,
                    const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
};
