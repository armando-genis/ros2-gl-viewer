#pragma once

#include <vector>
#include <memory>
#include <GL/gl3w.h>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glk {

struct CloudVertex {
    Eigen::Vector3f position;
    Eigen::Vector2f uv;
    Eigen::Vector3f normal;
};

struct CloudUniforms {
    float uTime = 0.0f;
    Eigen::Vector3f uSunDirection = Eigen::Vector3f(0.5f, 0.5f, -0.5f).normalized();
    Eigen::Vector2f uResolution = Eigen::Vector2f(1920.0f, 1080.0f);
    float uCloudCoverage = 0.65f;
    float uCloudHeight = 600.0f;
    float uCloudThickness = 45.0f;
    float uCloudAbsorption = 1.03f;
    float uWindSpeedX = 5.0f;
    float uWindSpeedZ = 3.0f;
    float uMaxCloudDistance = 10000.0f;
    GLuint t_PerlinNoise = 0;
};

class CloudRenderer {
public:
    CloudRenderer();
    ~CloudRenderer();

    // Initialize the cloud rendering system
    bool initCloudRendering();
    
    // Load Perlin noise texture
    bool loadPerlinTexture(const std::string& texturePath);
    
    // Create a sky sphere mesh
    void createSkySphere(float radius = 20000.0f, int widthSegments = 64, int heightSegments = 32);
    
    // Update uniforms
    void updateTime(float time);
    void updateSunDirection(const Eigen::Vector3f& sunDirection);
    void updateCloudParameters(float coverage, float height, float thickness, 
                              float absorption, float windX, float windZ, float maxDistance);
    
    
    // Cleanup resources
    void cleanupCloudRendering();

    // Render the cloud volume
    void renderCloudVolume(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, 
                          const Eigen::Vector3f& cameraPosition, const Eigen::Vector3f& halfExtents);

    // add to class CloudRenderer
    GLuint vol_vao_ = 0, vol_vbo_ = 0, vol_ebo_ = 0;
    Eigen::Matrix4f volume_model_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f volume_half_extents_ = Eigen::Vector3f::Zero();

    void createVolumeBox(const Eigen::Vector3f& halfExtents);
    void setVolumeModelMatrix(const Eigen::Matrix4f& M) { volume_model_ = M; }

private:
    // OpenGL resources
    GLuint cloud_vao_;
    GLuint cloud_vbo_;
    GLuint cloud_ebo_;
    GLuint cloud_shader_program_;
    GLuint perlin_texture_;

    
    

    // Mesh data
    std::vector<CloudVertex> vertices_;
    std::vector<unsigned int> indices_;
    
    // Uniforms
    CloudUniforms uniforms_;
    
    // Shader creation
    bool createCloudShaders();
    GLuint compileShaderProgram(const char* vertex_source, const char* fragment_source);
    
    // Texture loading
    GLuint loadTexture(const std::string& path);
    GLuint createDefaultPerlinTexture();
    
    // Geometry generation
    void generateSphereGeometry(float radius, int widthSegments, int heightSegments);
};

} // namespace glk
