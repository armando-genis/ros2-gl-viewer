
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
#include <filesystem>

// Add GLM includes for the mat4 types
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

struct cgltf_data;
struct cgltf_options;
struct cgltf_image;

struct PlyMesh
{
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0; // Element buffer object
    size_t vertex_count = 0;
    size_t index_count = 0;       // Total number of indices
    std::string frame_id = "map"; // Default frame ID

    // Texture support
    GLuint texture_id = 0;    // OpenGL texture ID
    bool has_texture = false; // Whether this mesh has a texture

    // Material properties
    float base_color[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // RGBA base color from material
    float metallic_factor = 0.0f;
    float roughness_factor = 1.0f;
};

class modelUpload
{
private:
    // Helper function to validate GLB file format
    bool validateGlbFile(const std::string &path);

    // Helper function to get detailed cgltf error message
    std::string getCgltfErrorMessage(int result);

    // Helper function to load texture from cgltf image
    GLuint loadTextureFromImage(const cgltf_image *image, const cgltf_data *data);

public:
    modelUpload(/* args */);
    ~modelUpload();
    bool createGLBShader(GLuint &shader_program);
    PlyMesh loadModel(const std::string &path);
    PlyMesh loadPlyBinaryLE(const std::string &path);
    PlyMesh loadGlb(const std::string &path);
    void renderMesh(const PlyMesh &mesh,
                    glk::GLSLShader &shader,
                    std::mutex &tf_mutex,
                    const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);

    void renderGLBMesh(const PlyMesh &mesh,
                       GLuint shader_program,
                       const glm::mat4 &view_matrix,
                       const glm::mat4 &projection_matrix,
                       std::mutex &tf_mutex,
                       const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
};
