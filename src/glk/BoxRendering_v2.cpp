#include <glk/BoxRendering.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace glk {

BoxRenderer::BoxRenderer() 
    : box_vao_(0), box_vbo_(0), box_shader_program_(0) {
}

BoxRenderer::~BoxRenderer() {
    cleanupBoxRendering();
}

bool BoxRenderer::initBoxRendering() {
    // Create VAO and VBO
    glGenVertexArrays(1, &box_vao_);
    glGenBuffers(1, &box_vbo_);
    
    glBindVertexArray(box_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, box_vbo_);
    
    // Allocate buffer for dynamic data
    const size_t max_vertices = 100000; // Adjust based on needs
    glBufferData(GL_ARRAY_BUFFER, 
                 max_vertices * sizeof(BoxVertex), 
                 nullptr, 
                 GL_DYNAMIC_DRAW);
    
    // Set up vertex attributes
    // Position (3 floats)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                         sizeof(BoxVertex), (void*)offsetof(BoxVertex, position));
    
    // Corner index (2 floats) - for compatibility, use texCoord field
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 
                         sizeof(BoxVertex), (void*)offsetof(BoxVertex, texCoord));
    
    // Color mode (1 float)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 
                         sizeof(BoxVertex), (void*)offsetof(BoxVertex, colorMode));
    
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    // Create shaders
    if (!createBoxShaders()) {
        std::cerr << "Failed to create box shaders" << std::endl;
        return false;
    }
    
    return true;
}

void BoxRenderer::addBoundingBox(const std::vector<Eigen::Vector3f>& corners,
                                  const Eigen::Vector3f& color,
                                  float alpha,
                                  int colorMode,
                                  float z_offset) {
    if (corners.size() != 8) {
        std::cerr << "BoundingBox must have exactly 8 corners" << std::endl;
        return;
    }
    
    BoundingBox bbox;
    // Apply z_offset to all corners
    bbox.corners.resize(8);
    for (int i = 0; i < 8; ++i) {
        bbox.corners[i] = corners[i];
        bbox.corners[i].z() += z_offset;
    }
    bbox.color = color;
    bbox.alpha = alpha;
    bbox.colorMode = colorMode;
    bbox.visible = true;
    
    // Generate geometry for this bounding box
    generateBoundingBoxGeometry(bbox);
    
    bounding_boxes_.push_back(bbox);
}

void BoxRenderer::renderBoxes(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) {
    if (bounding_boxes_.empty() || box_shader_program_ == 0) return;

    glUseProgram(box_shader_program_);

    // cache uniform locations
    static GLuint cached_prog = 0;
    static GLint loc_view=-1, loc_proj=-1, loc_color=-1, loc_alpha=-1;

    if (cached_prog != box_shader_program_) {
        cached_prog = box_shader_program_;
        loc_view    = glGetUniformLocation(box_shader_program_, "view");
        loc_proj    = glGetUniformLocation(box_shader_program_, "projection");
        loc_color   = glGetUniformLocation(box_shader_program_, "boxColor");
        loc_alpha   = glGetUniformLocation(box_shader_program_, "boxAlpha");
    }

    // Set global uniforms (same for all boxes)
    if (loc_view >= 0) glUniformMatrix4fv(loc_view, 1, GL_FALSE, view.data());
    if (loc_proj >= 0) glUniformMatrix4fv(loc_proj, 1, GL_FALSE, projection.data());

    // Enable transparency
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);  // Don't write depth for transparent objects
    glDisable(GL_CULL_FACE);  // Show both sides of faces

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // Standard alpha blending

    glBindVertexArray(box_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, box_vbo_);

    // Render bounding boxes
    for (const auto& bbox : bounding_boxes_) {
        if (!bbox.visible || bbox.vertices.empty() || bbox.corners.size() != 8) continue;
        
        // Set per-box uniforms
        if (loc_color >= 0) glUniform3f(loc_color, bbox.color.x(), bbox.color.y(), bbox.color.z());
        if (loc_alpha >= 0) glUniform1f(loc_alpha, bbox.alpha);
        
        // Upload and draw this bounding box
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(bbox.vertices.size() * sizeof(BoxVertex)),
                        bbox.vertices.data());
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(bbox.vertices.size()));
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}


void BoxRenderer::clearBoxes() {
    bounding_boxes_.clear();
}

void BoxRenderer::cleanupBoxRendering() {
    if (box_vao_ != 0) {
        glDeleteVertexArrays(1, &box_vao_);
        box_vao_ = 0;
    }
    if (box_vbo_ != 0) {
        glDeleteBuffers(1, &box_vbo_);
        box_vbo_ = 0;
    }
    if (box_shader_program_ != 0) {
        glDeleteProgram(box_shader_program_);
        box_shader_program_ = 0;
    }
}

bool BoxRenderer::createBoxShaders() {
    const char* vertex_shader = R"(
        #version 460 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec2 texCoord;
        layout (location = 2) in float colorMode;

        uniform mat4 view;
        uniform mat4 projection;

        void main() {
            gl_Position = projection * view * vec4(position, 1.0);
        }
    )";

    const char* fragment_shader = R"(
        #version 460 core
        out vec4 FragColor;

        uniform vec3  boxColor;
        uniform float boxAlpha;

        void main() {
            // Simple transparent bounding box - just use the color and alpha
            FragColor = vec4(boxColor, boxAlpha);
        }
    )";

    box_shader_program_ = compileShaderProgram(vertex_shader, fragment_shader);
    return box_shader_program_ != 0;
}


GLuint BoxRenderer::compileShaderProgram(const char* vertex_source, const char* fragment_source) {
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);
    
    GLint success;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "Box vertex shader compilation failed: " << info_log << std::endl;
        return 0;
    }
    
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);
    
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "Box fragment shader compilation failed: " << info_log << std::endl;
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
        std::cerr << "Box shader program linking failed: " << info_log << std::endl;
        return 0;
    }
    
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    return program;
}

void BoxRenderer::generateBoundingBoxGeometry(BoundingBox& bbox) {
    bbox.vertices.clear();
    
    if (bbox.corners.size() != 8) return;
    
    // Extract bottom and top corners
    std::vector<Eigen::Vector3f> bottom_corners(4);
    std::vector<Eigen::Vector3f> top_corners(4);
    for (int i = 0; i < 4; ++i) {
        bottom_corners[i] = bbox.corners[i];
        top_corners[i] = bbox.corners[i + 4];
    }
    
    // Get base height from first bottom and top corner
    float base_height = (top_corners[0].z() - bottom_corners[0].z()) * 0.3f; // 30% of total height
    float z_base_bottom = bottom_corners[0].z();
    float z_base_top = z_base_bottom + base_height;
    float z_wall_bottom = z_base_top;
    
    // -------------------------------
    // 1) Generate flat base at bottom
    // -------------------------------
    {
        // Top face of base
        BoxVertex t1, t2, t3, t4, t5, t6;
        t1.position = Eigen::Vector3f(bottom_corners[0].x(), bottom_corners[0].y(), z_base_top);
        t1.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        t1.colorMode = float(bbox.colorMode);
        t2.position = Eigen::Vector3f(bottom_corners[1].x(), bottom_corners[1].y(), z_base_top);
        t2.texCoord = Eigen::Vector2f(1.0f, 0.0f);
        t2.colorMode = float(bbox.colorMode);
        t3.position = Eigen::Vector3f(bottom_corners[2].x(), bottom_corners[2].y(), z_base_top);
        t3.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        t3.colorMode = float(bbox.colorMode);
        t4.position = Eigen::Vector3f(bottom_corners[0].x(), bottom_corners[0].y(), z_base_top);
        t4.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        t4.colorMode = float(bbox.colorMode);
        t5.position = Eigen::Vector3f(bottom_corners[2].x(), bottom_corners[2].y(), z_base_top);
        t5.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        t5.colorMode = float(bbox.colorMode);
        t6.position = Eigen::Vector3f(bottom_corners[3].x(), bottom_corners[3].y(), z_base_top);
        t6.texCoord = Eigen::Vector2f(0.0f, 1.0f);
        t6.colorMode = float(bbox.colorMode);
        bbox.vertices.push_back(t1);
        bbox.vertices.push_back(t2);
        bbox.vertices.push_back(t3);
        bbox.vertices.push_back(t4);
        bbox.vertices.push_back(t5);
        bbox.vertices.push_back(t6);
        
        // Bottom face of base
        BoxVertex b1, b2, b3, b4, b5, b6;
        b1.position = Eigen::Vector3f(bottom_corners[0].x(), bottom_corners[0].y(), z_base_bottom);
        b1.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        b1.colorMode = float(bbox.colorMode);
        b2.position = Eigen::Vector3f(bottom_corners[1].x(), bottom_corners[1].y(), z_base_bottom);
        b2.texCoord = Eigen::Vector2f(1.0f, 0.0f);
        b2.colorMode = float(bbox.colorMode);
        b3.position = Eigen::Vector3f(bottom_corners[2].x(), bottom_corners[2].y(), z_base_bottom);
        b3.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        b3.colorMode = float(bbox.colorMode);
        b4.position = Eigen::Vector3f(bottom_corners[0].x(), bottom_corners[0].y(), z_base_bottom);
        b4.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        b4.colorMode = float(bbox.colorMode);
        b5.position = Eigen::Vector3f(bottom_corners[2].x(), bottom_corners[2].y(), z_base_bottom);
        b5.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        b5.colorMode = float(bbox.colorMode);
        b6.position = Eigen::Vector3f(bottom_corners[3].x(), bottom_corners[3].y(), z_base_bottom);
        b6.texCoord = Eigen::Vector2f(0.0f, 1.0f);
        b6.colorMode = float(bbox.colorMode);
        bbox.vertices.push_back(b1);
        bbox.vertices.push_back(b2);
        bbox.vertices.push_back(b3);
        bbox.vertices.push_back(b4);
        bbox.vertices.push_back(b5);
        bbox.vertices.push_back(b6);
        
        // 4 side faces of base
        for (size_t i = 0; i < 4; ++i) {
            size_t next_i = (i + 1) % 4;
            
            BoxVertex v1, v2, v3, v4, v5, v6;
            
            v1.position = Eigen::Vector3f(bottom_corners[i].x(), bottom_corners[i].y(), z_base_bottom);
            v1.texCoord = Eigen::Vector2f(float(i), 0.0f);
            v1.colorMode = float(bbox.colorMode);
            
            v2.position = Eigen::Vector3f(bottom_corners[next_i].x(), bottom_corners[next_i].y(), z_base_bottom);
            v2.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
            v2.colorMode = float(bbox.colorMode);
            
            v3.position = Eigen::Vector3f(bottom_corners[i].x(), bottom_corners[i].y(), z_base_top);
            v3.texCoord = Eigen::Vector2f(float(i), 1.0f);
            v3.colorMode = float(bbox.colorMode);
            
            v4.position = Eigen::Vector3f(bottom_corners[next_i].x(), bottom_corners[next_i].y(), z_base_bottom);
            v4.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
            v4.colorMode = float(bbox.colorMode);
            
            v5.position = Eigen::Vector3f(bottom_corners[next_i].x(), bottom_corners[next_i].y(), z_base_top);
            v5.texCoord = Eigen::Vector2f(float(next_i), 1.0f);
            v5.colorMode = float(bbox.colorMode);
            
            v6.position = Eigen::Vector3f(bottom_corners[i].x(), bottom_corners[i].y(), z_base_top);
            v6.texCoord = Eigen::Vector2f(float(i), 1.0f);
            v6.colorMode = float(bbox.colorMode);
            
            bbox.vertices.push_back(v1);
            bbox.vertices.push_back(v2);
            bbox.vertices.push_back(v3);
            bbox.vertices.push_back(v4);
            bbox.vertices.push_back(v5);
            bbox.vertices.push_back(v6);
        }
    }
    
    // -------------------------------
    // 2) Generate 4 side walls (no top/bottom faces)
    // -------------------------------
    for (size_t i = 0; i < 4; ++i) {
        size_t next_i = (i + 1) % 4;
        
        const Eigen::Vector3f& bottom1 = bottom_corners[i];
        const Eigen::Vector3f& bottom2 = bottom_corners[next_i];
        const Eigen::Vector3f& top1 = top_corners[i];
        const Eigen::Vector3f& top2 = top_corners[next_i];
        
        BoxVertex v1, v2, v3, v4, v5, v6;
        
        // First triangle of wall quad
        v1.position = Eigen::Vector3f(bottom1.x(), bottom1.y(), z_wall_bottom);
        v1.texCoord = Eigen::Vector2f(float(i), 0.0f);
        v1.colorMode = float(bbox.colorMode);
        
        v2.position = Eigen::Vector3f(bottom2.x(), bottom2.y(), z_wall_bottom);
        v2.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
        v2.colorMode = float(bbox.colorMode);
        
        v3.position = Eigen::Vector3f(top1.x(), top1.y(), top1.z());
        v3.texCoord = Eigen::Vector2f(float(i), 1.0f);
        v3.colorMode = float(bbox.colorMode);
        
        // Second triangle of wall quad
        v4.position = Eigen::Vector3f(bottom2.x(), bottom2.y(), z_wall_bottom);
        v4.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
        v4.colorMode = float(bbox.colorMode);
        
        v5.position = Eigen::Vector3f(top2.x(), top2.y(), top2.z());
        v5.texCoord = Eigen::Vector2f(float(next_i), 1.0f);
        v5.colorMode = float(bbox.colorMode);
        
        v6.position = Eigen::Vector3f(top1.x(), top1.y(), top1.z());
        v6.texCoord = Eigen::Vector2f(float(i), 1.0f);
        v6.colorMode = float(bbox.colorMode);
        
        bbox.vertices.push_back(v1);
        bbox.vertices.push_back(v2);
        bbox.vertices.push_back(v3);
        bbox.vertices.push_back(v4);
        bbox.vertices.push_back(v5);
        bbox.vertices.push_back(v6);
    }
}

} // namespace glk

