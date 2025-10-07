#include <glk/PathRendering.hpp>
#include <iostream>
#include <algorithm>

namespace glk {

PathRenderer::PathRenderer() 
    : path_vao_(0), path_vbo_(0), path_shader_program_(0) {
}

PathRenderer::~PathRenderer() {
    cleanupPathRendering();
}

bool PathRenderer::initPathRendering() {
    // Create VAO and VBO
    glGenVertexArrays(1, &path_vao_);
    glGenBuffers(1, &path_vbo_);
    
    glBindVertexArray(path_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, path_vbo_);
    
    // Allocate buffer for dynamic data
    const size_t max_vertices = 100000; // Adjust based on needs
    glBufferData(GL_ARRAY_BUFFER, 
                 max_vertices * sizeof(PathVertex), 
                 nullptr, 
                 GL_DYNAMIC_DRAW);
    
    // Set up vertex attributes
    // Position (3 floats)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                         sizeof(PathVertex), (void*)offsetof(PathVertex, position));
    
    // Texture coordinates for gradient (2 floats)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 
                         sizeof(PathVertex), (void*)offsetof(PathVertex, texCoord));
    
    // Path width (1 float)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 
                         sizeof(PathVertex), (void*)offsetof(PathVertex, pathWidth));
    
    // Path type (1 float)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 
                         sizeof(PathVertex), (void*)offsetof(PathVertex, pathType));
    
    // Color mode (1 float)
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, 
                         sizeof(PathVertex), (void*)offsetof(PathVertex, colorMode));
    
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    // Create shaders
    if (!createPathShaders()) {
        std::cerr << "Failed to create path shaders" << std::endl;
        return false;
    }
    
    return true;
}

void PathRenderer::addPathSegment(const std::vector<Eigen::Vector3f>& points,
                                 const Eigen::Vector3f& color,
                                 float pathWidth,
                                 int pathType,
                                 int colorMode,
                                 float alpha) {
    if (points.size() < 2) return;
    
    PathSegment segment;
    segment.color = color;
    segment.alpha = alpha;
    segment.visible = true;
    
    // Generate geometry for this path segment
    generatePathGeometry(points, segment.vertices, pathWidth, pathType, colorMode);
    
    path_segments_.push_back(segment);
}

void PathRenderer::renderPaths(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) {
    if (path_segments_.empty()) return;
    
    glUseProgram(path_shader_program_);
    
    // Set uniforms
    glUniformMatrix4fv(glGetUniformLocation(path_shader_program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(path_shader_program_, "projection"), 1, GL_FALSE, projection.data());
    
    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glBindVertexArray(path_vao_);
    
    // Render each path segment
    for (const auto& segment : path_segments_) {
        if (!segment.visible || segment.vertices.empty()) continue;
        
        // Set segment-specific uniforms
        glUniform3f(glGetUniformLocation(path_shader_program_, "pathColor"), 
                    segment.color.x(), segment.color.y(), segment.color.z());
        glUniform1f(glGetUniformLocation(path_shader_program_, "pathAlpha"), segment.alpha);
        
        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, path_vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, 
                       segment.vertices.size() * sizeof(PathVertex),
                       segment.vertices.data());
        
        // Draw based on path type
        if (segment.vertices[0].pathType < 0.5f) {
            // Normal line path
            glDrawArrays(GL_LINE_STRIP, 0, segment.vertices.size());
        } else {
            // Car path (ribbon) - draw as triangle strip
            glDrawArrays(GL_TRIANGLE_STRIP, 0, segment.vertices.size());
        }
    }
    
    glBindVertexArray(0);
    glDisable(GL_BLEND);
}

void PathRenderer::clearPaths() {
    path_segments_.clear();
}

void PathRenderer::cleanupPathRendering() {
    if (path_vao_ != 0) {
        glDeleteVertexArrays(1, &path_vao_);
        path_vao_ = 0;
    }
    if (path_vbo_ != 0) {
        glDeleteBuffers(1, &path_vbo_);
        path_vbo_ = 0;
    }
    if (path_shader_program_ != 0) {
        glDeleteProgram(path_shader_program_);
        path_shader_program_ = 0;
    }
}

bool PathRenderer::createPathShaders() {
    // Vertex shader
    const char* vertex_shader = R"(
        #version 460 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec2 texCoord;
        layout (location = 2) in float pathWidth;
        layout (location = 3) in float pathType;
        layout (location = 4) in float colorMode;
        
        out vec2 TexCoord;
        out float PathWidth;
        out float PathType;
        out float ColorMode;
        
        uniform mat4 view;
        uniform mat4 projection;
        
        void main() {
            gl_Position = projection * view * vec4(position, 1.0);
            TexCoord = texCoord;
            PathWidth = pathWidth;
            PathType = pathType;
            ColorMode = colorMode;
        }
    )";
    
    // Fragment shader
    const char* fragment_shader = R"(
        #version 460 core
        in vec2 TexCoord;
        in float PathWidth;
        in float PathType;
        in float ColorMode;
        
        out vec4 FragColor;
        
        uniform vec3 pathColor;
        uniform float pathAlpha;
        
        void main() {
            if (PathType < 0.5) {
                // Normal line path - solid color
                FragColor = vec4(pathColor, pathAlpha);
            } else {
                // Car path (ribbon) - apply gradient based on color mode
                float alpha = pathAlpha;
                float distance_from_center = abs(TexCoord.x - 0.5) * 2.0; // 0 to 1
                
                if (ColorMode > 0.5) {
                    // Gradient mode
                    if (ColorMode < 1.5) {
                        // Mode 1: Center solid -> edges transparent
                        alpha *= (1.0 - distance_from_center);
                    } else {
                        // Mode 2: Edges solid -> center transparent
                        alpha *= distance_from_center;
                    }
                }
                
                // Apply anti-aliasing at edges
                float edge_fade = 1.0 - smoothstep(0.8, 1.0, distance_from_center);
                alpha *= edge_fade;
                
                FragColor = vec4(pathColor, alpha);
            }
        }
    )";
    
    path_shader_program_ = compileShaderProgram(vertex_shader, fragment_shader);
    return path_shader_program_ != 0;
}

GLuint PathRenderer::compileShaderProgram(const char* vertex_source, const char* fragment_source) {
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);
    
    GLint success;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "Path vertex shader compilation failed: " << info_log << std::endl;
        return 0;
    }
    
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);
    
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "Path fragment shader compilation failed: " << info_log << std::endl;
        return 0;
    }
    
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetProgramInfoLog(program, 512, NULL, info_log);
        std::cerr << "Path shader program linking failed: " << info_log << std::endl;
        return 0;
    }
    
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    return program;
}

void PathRenderer::generatePathGeometry(const std::vector<Eigen::Vector3f>& points,
                                      std::vector<PathVertex>& vertices,
                                      float pathWidth, int pathType, int colorMode) {
    vertices.clear();
    
    if (pathType == 0) {
        // Normal line path - simple line strip
        for (const auto& point : points) {
            PathVertex vertex;
            vertex.position = point;
            vertex.texCoord = Eigen::Vector2f(0.0f, 0.0f);
            vertex.pathWidth = pathWidth;
            vertex.pathType = 0.0f;
            vertex.colorMode = float(colorMode);
            vertices.push_back(vertex);
        }
    } else {
        // Car path (ribbon) - generate triangle strip
        const float half_width = pathWidth / 2.0f;
        
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            
            // Calculate forward direction
            Eigen::Vector3f forward;
            if (i < points.size() - 1) {
                forward = (points[i + 1] - point).normalized();
            } else if (i > 0) {
                forward = (point - points[i - 1]).normalized();
            } else {
                forward = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Default direction
            }
            
            // Calculate perpendicular direction
            Eigen::Vector3f right(-forward.y(), forward.x(), 0.0f);
            right.normalize();
            
            // Create left and right vertices for triangle strip
            PathVertex left_vertex, right_vertex;
            
            left_vertex.position = point - right * half_width;
            left_vertex.texCoord = Eigen::Vector2f(0.0f, float(i) / float(points.size() - 1));
            left_vertex.pathWidth = pathWidth;
            left_vertex.pathType = 1.0f;
            left_vertex.colorMode = float(colorMode);
            
            right_vertex.position = point + right * half_width;
            right_vertex.texCoord = Eigen::Vector2f(1.0f, float(i) / float(points.size() - 1));
            right_vertex.pathWidth = pathWidth;
            right_vertex.pathType = 1.0f;
            right_vertex.colorMode = float(colorMode);
            
            vertices.push_back(left_vertex);
            vertices.push_back(right_vertex);
        }
    }
}

} // namespace glk
