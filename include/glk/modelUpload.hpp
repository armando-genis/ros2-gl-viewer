
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
struct cgltf_node;

struct PlyMesh
{
    int type = -1; // 0 for GLB, 1 for PLY and -1 for error
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0; // Element buffer object
    size_t vertex_count = 0;
    size_t index_count = 0;       // Total number of indices
    std::string frame_id = "map"; // Default frame ID

    // Texture supports
    GLuint texture_id = 0;    // OpenGL texture ID
    bool has_texture = false; // Whether this mesh has a texture

    // Material properties
    float base_color[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // RGBA base color from material
    float metallic_factor = 0.0f;
    float roughness_factor = 1.0f;

    Eigen::Matrix4f local_transform = Eigen::Matrix4f::Identity();
    
    // Offset from TF frame (in meters)
    Eigen::Vector3f offset = Eigen::Vector3f::Zero();
    
    // Optional: store model name for UI display
    std::string model_name = "";
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

    // Helper function to extract node transform
    Eigen::Matrix4f extractNodeTransform(const cgltf_node *node);

    // Store current view and projection matrices
    glm::mat4 current_view_matrix = glm::mat4(1.0f);
    glm::mat4 current_projection_matrix = glm::mat4(1.0f);

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
                       std::mutex &tf_mutex,
                       const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);

    static void cleanupMesh(PlyMesh &mesh);

    void setMatrices(const Eigen::Matrix4f &view_matrix, const Eigen::Matrix4f &projection_matrix);
};
