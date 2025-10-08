#pragma once

#include <vector>
#include <memory>
#include <GL/gl3w.h>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace glk {

struct PathVertex {
    Eigen::Vector3f position;
    Eigen::Vector2f texCoord;  // For gradient calculation
    float pathWidth;          // Width of the path at this vertex
    float pathType;           // 0 = normal line, 1 = car path
    float colorMode;          // 0 = flat, 1 = gradient center->edges, 2 = gradient edges->center
};

struct PathSegment {
    std::vector<PathVertex> vertices;
    Eigen::Vector3f color;
    float alpha;
    bool visible;
};

class PathRenderer {
public:
    PathRenderer();
    ~PathRenderer();

    // Initialize the path rendering system
    bool initPathRendering();
    
    // Add a path segment
    void addPathSegment(const std::vector<Eigen::Vector3f>& points, 
                       const Eigen::Vector3f& color,
                       float pathWidth = 1.5f,
                       int pathType = 0,
                       int colorMode = 0,
                       float alpha = 1.0f);
    
    // Render all path segments
    void renderPaths(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);
    
    // Clear all path segments
    void clearPaths();
    
    // Cleanup resources
    void cleanupPathRendering();

private:
    // OpenGL resources
    GLuint path_vao_;
    GLuint path_vbo_;
    GLuint path_shader_program_;
    
    // Path segments storage
    std::vector<PathSegment> path_segments_;
    
    // Shader creation
    bool createPathShaders();
    GLuint compileShaderProgram(const char* vertex_source, const char* fragment_source);
    
    // Geometry generation helpers
    void generatePathGeometry(const std::vector<Eigen::Vector3f>& points,
                             std::vector<PathVertex>& vertices,
                             float pathWidth, int pathType, int colorMode);
};

} // namespace glk
