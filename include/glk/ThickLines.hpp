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

class ThickLinesRenderer
{
public:
    ThickLinesRenderer() = default;
    ~ThickLinesRenderer()
    {
        cleanup();
    }

    // Initialize the thick lines shader and buffers
    bool initialize();

    // Draw coordinate frames with thick lines
    void drawCoordinateFrames(const Eigen::Matrix4f &view,
                              const Eigen::Matrix4f &projection,
                              const std::string &fixed_frame,
                              const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms,
                              float frame_size,
                              float line_thickness);

    // Cleanup resources
    void cleanup();

private:
    GLuint shader_program_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;

    struct AxisData
    {
        int axis_id; // 0=X, 1=Y, 2=Z
    };

    bool createShader();

    bool checkShaderCompilation(GLuint shader, const std::string &type);

    void setupBuffers();

    // Cache uniform locations
    GLint view_matrix_loc_ = -1;
    GLint projection_matrix_loc_ = -1;
    GLint model_matrix_loc_ = -1;
    GLint cylinder_radius_loc_ = -1;
    GLint cylinder_length_loc_ = -1;

    // Cache uniform locations after shader compilation
    void cacheUniformLocations();

    // Add frame counter to detect memory leaks
    mutable size_t render_call_count_ = 0;
    mutable size_t last_memory_check_ = 0;
};