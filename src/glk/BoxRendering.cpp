#include <glk/BoxRendering.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <stb_image.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace glk {

BoxRenderer::BoxRenderer() 
    : box_vao_(0), box_vbo_(0), box_shader_program_(0),
      icon_texture_(0), icon_vao_(0), icon_vbo_(0), icon_shader_program_(0),
      icon_texture_loaded_(false), icon_size_(0.5f) {
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
                                  float z_offset,
                                  float icon_z_offset) {
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
    bbox.icon_z_offset = icon_z_offset;
    
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
    
    // Render icons on top of boxes
    renderIcons(view, projection);
}


void BoxRenderer::clearBoxes() {
    bounding_boxes_.clear();
}

bool BoxRenderer::loadIconTexture(const std::string& image_path) {
    int width, height, channels;
    // Force loading as RGBA (4 channels) for consistent format
    unsigned char *data = stbi_load(image_path.c_str(), &width, &height, &channels, 4);
    
    if (!data) {
        std::cerr << "Failed to load icon image: " << image_path << std::endl;
        return false;
    }
    
    std::cout << "Loaded icon image: " << image_path << std::endl;
    std::cout << "  Resolution: " << width << "x" << height << std::endl;
    std::cout << "  Original channels: " << channels << ", forced to 4 (RGBA)" << std::endl;
    
    // Delete existing texture if any
    if (icon_texture_ != 0) {
        glDeleteTextures(1, &icon_texture_);
    }
    
    glGenTextures(1, &icon_texture_);
    glBindTexture(GL_TEXTURE_2D, icon_texture_);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Upload texture data - always RGBA now since we forced 4 channels
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    
    stbi_image_free(data);
    
    icon_texture_loaded_ = true;
    
    // Initialize icon VAO/VBO if not already done
    if (icon_vao_ == 0) {
        // Create VAO and VBO for icon quads
        glGenVertexArrays(1, &icon_vao_);
        glGenBuffers(1, &icon_vbo_);
        
        glBindVertexArray(icon_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, icon_vbo_);
        
        // Allocate buffer for icon quads (each icon is 6 vertices = 2 triangles)
        const size_t max_icons = 1000;
        glBufferData(GL_ARRAY_BUFFER, 
                     max_icons * 6 * sizeof(float) * 5, // 6 vertices * (3 pos + 2 tex) floats
                     nullptr, 
                     GL_DYNAMIC_DRAW);
        
        // Position (3 floats)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        
        // Texture coordinates (2 floats)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    
    // Create icon shaders if not already created
    if (icon_shader_program_ == 0) {
        if (!createIconShaders()) {
            std::cerr << "Failed to create icon shaders" << std::endl;
            return false;
        }
    }
    
    std::cout << "Icon texture loaded successfully (ID: " << icon_texture_ << ")" << std::endl;
    return true;
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
    
    // Cleanup icon resources
    if (icon_texture_ != 0) {
        glDeleteTextures(1, &icon_texture_);
        icon_texture_ = 0;
    }
    if (icon_vao_ != 0) {
        glDeleteVertexArrays(1, &icon_vao_);
        icon_vao_ = 0;
    }
    if (icon_vbo_ != 0) {
        glDeleteBuffers(1, &icon_vbo_);
        icon_vbo_ = 0;
    }
    if (icon_shader_program_ != 0) {
        glDeleteProgram(icon_shader_program_);
        icon_shader_program_ = 0;
    }
    icon_texture_loaded_ = false;
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

bool BoxRenderer::createIconShaders() {
    const char* vertex_shader = R"(
        #version 460 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTexCoord;
        
        uniform mat4 view;
        uniform mat4 projection;
        
        out vec2 TexCoord;
        
        void main() {
            gl_Position = projection * view * vec4(aPos, 1.0);
            TexCoord = aTexCoord;
        }
    )";

    const char* fragment_shader = R"(
        #version 460 core
        out vec4 FragColor;
        
        in vec2 TexCoord;
        uniform sampler2D u_texture;
        uniform float u_alpha;
        
        void main() {
            vec4 texColor = texture(u_texture, TexCoord);
            
            // Use the texture color directly with alpha
            FragColor = vec4(texColor.rgb, texColor.a * u_alpha);
        }
    )";

    icon_shader_program_ = compileShaderProgram(vertex_shader, fragment_shader);
    return icon_shader_program_ != 0;
}

void BoxRenderer::renderIcons(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) {
    if (!icon_texture_loaded_ || icon_shader_program_ == 0 || bounding_boxes_.empty()) return;
    
    glUseProgram(icon_shader_program_);
    
    // Cache uniform locations
    static GLuint cached_prog = 0;
    static GLint loc_view = -1, loc_proj = -1, loc_texture = -1, loc_alpha = -1;
    
    if (cached_prog != icon_shader_program_) {
        cached_prog = icon_shader_program_;
        loc_view = glGetUniformLocation(icon_shader_program_, "view");
        loc_proj = glGetUniformLocation(icon_shader_program_, "projection");
        loc_texture = glGetUniformLocation(icon_shader_program_, "u_texture");
        loc_alpha = glGetUniformLocation(icon_shader_program_, "u_alpha");
    }
    
    // Set uniforms
    if (loc_view >= 0) glUniformMatrix4fv(loc_view, 1, GL_FALSE, view.data());
    if (loc_proj >= 0) glUniformMatrix4fv(loc_proj, 1, GL_FALSE, projection.data());
    if (loc_texture >= 0) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, icon_texture_);
        glUniform1i(loc_texture, 0);
    }
    if (loc_alpha >= 0) glUniform1f(loc_alpha, 1.0f);
    
    // Enable transparency
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // First, render circular backgrounds using box shader
    glUseProgram(box_shader_program_);
    
    // Cache uniform locations for box shader
    static GLuint cached_box_prog = 0;
    static GLint loc_box_view = -1, loc_box_proj = -1, loc_box_color = -1, loc_box_alpha = -1;
    
    if (cached_box_prog != box_shader_program_) {
        cached_box_prog = box_shader_program_;
        loc_box_view = glGetUniformLocation(box_shader_program_, "view");
        loc_box_proj = glGetUniformLocation(box_shader_program_, "projection");
        loc_box_color = glGetUniformLocation(box_shader_program_, "boxColor");
        loc_box_alpha = glGetUniformLocation(box_shader_program_, "boxAlpha");
    }
    
    // Set box shader uniforms
    if (loc_box_view >= 0) glUniformMatrix4fv(loc_box_view, 1, GL_FALSE, view.data());
    if (loc_box_proj >= 0) glUniformMatrix4fv(loc_box_proj, 1, GL_FALSE, projection.data());
    
    // Render circular backgrounds
    std::vector<float> circle_vertices;
    
    for (const auto& bbox : bounding_boxes_) {
        if (!bbox.visible || bbox.corners.size() != 8) continue;
        
        // Calculate center of top face
        Eigen::Vector3f top_center = Eigen::Vector3f::Zero();
        for (int i = 4; i < 8; ++i) {
            top_center += bbox.corners[i];
        }
        top_center /= 4.0f;
        
        // Icon size is fixed, not dependent on bounding box size (5% smaller)
        float icon_half_width = icon_size_ * 0.5f * 0.95f;
        float circle_radius = icon_half_width * 1.3f; // Make circle slightly larger than icon (also 5% smaller)
        
        // Calculate forward direction from top face (for orientation)
        Eigen::Vector3f v1 = bbox.corners[5] - bbox.corners[4]; // Right edge of top face
        Eigen::Vector3f forward = v1.normalized();
        
        // Right vector: perpendicular to forward, in XY plane
        Eigen::Vector3f right = Eigen::Vector3f(-forward.y(), forward.x(), 0.0f).normalized();
        if (right.norm() < 1e-6f) {
            right = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        }
        
        // Up vector: always vertical (Z axis) - same as icon
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        
        // Position circle at center of icon (vertically centered), with icon_z_offset applied, slightly behind
        Eigen::Vector3f icon_bottom_center = top_center;
        icon_bottom_center.z() += bbox.icon_z_offset;
        float icon_height = icon_half_width * 1.2f;
        Eigen::Vector3f circle_center = icon_bottom_center + up * (icon_height * 0.5f); // Center of icon
        circle_center -= (right * 0.01f); // Slightly behind icon
        
        // Generate circle geometry (vertical disc in same plane as icon using triangle fan)
        const int circle_segments = 16;
        const float angle_step = 2.0f * M_PI / circle_segments;
        
        // Center vertex (vertical circle)
        circle_vertices.push_back(circle_center.x());
        circle_vertices.push_back(circle_center.y());
        circle_vertices.push_back(circle_center.z());
        circle_vertices.push_back(0.5f); // texCoord (not used, but needed for layout)
        circle_vertices.push_back(0.5f);
        circle_vertices.push_back(0.0f); // colorMode (not used)
        
        // Circle edge vertices (vertical circle in right-up plane)
        for (int i = 0; i <= circle_segments; ++i) {
            float angle = i * angle_step;
            // Generate circle in the plane defined by right and up vectors
            Eigen::Vector3f vertex = circle_center + 
                right * (circle_radius * std::cos(angle)) +
                up * (circle_radius * std::sin(angle));
            
            circle_vertices.push_back(vertex.x());
            circle_vertices.push_back(vertex.y());
            circle_vertices.push_back(vertex.z());
            circle_vertices.push_back(0.5f + 0.5f * std::cos(angle)); // texCoord
            circle_vertices.push_back(0.5f + 0.5f * std::sin(angle));
            circle_vertices.push_back(0.0f); // colorMode
        }
    }
    
    // Render circles if we have any
    if (!circle_vertices.empty()) {
        glBindVertexArray(box_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, box_vbo_);
        
        // Allocate buffer if needed
        static size_t max_circle_vertices = 0;
        size_t circle_vertex_count = circle_vertices.size();
        if (circle_vertex_count > max_circle_vertices) {
            max_circle_vertices = circle_vertex_count;
            glBufferData(GL_ARRAY_BUFFER, max_circle_vertices * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        }
        
        glBufferSubData(GL_ARRAY_BUFFER, 0, circle_vertex_count * sizeof(float), circle_vertices.data());
        
        // Render circles as triangle fans (each circle is one triangle fan)
        const int vertices_per_circle = 16 + 2; // 1 center + 17 edge vertices
        const int floats_per_vertex = 6; // position(3) + texCoord(2) + colorMode(1)
        int vertex_offset = 0;
        
        for (const auto& bbox : bounding_boxes_) {
            if (!bbox.visible || bbox.corners.size() != 8) continue;
            
            // Set color for this circle (alpha always 1.0f for full opacity)
            if (loc_box_color >= 0) glUniform3f(loc_box_color, bbox.color.x(), bbox.color.y(), bbox.color.z());
            if (loc_box_alpha >= 0) glUniform1f(loc_box_alpha, 1.0f);
            
            // Render this circle
            glDrawArrays(GL_TRIANGLE_FAN, vertex_offset, vertices_per_circle);
            vertex_offset += vertices_per_circle;
        }
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    
    // Now render icon textures on top
    glUseProgram(icon_shader_program_);
    
    // Re-set icon shader uniforms
    if (loc_view >= 0) glUniformMatrix4fv(loc_view, 1, GL_FALSE, view.data());
    if (loc_proj >= 0) glUniformMatrix4fv(loc_proj, 1, GL_FALSE, projection.data());
    if (loc_texture >= 0) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, icon_texture_);
        glUniform1i(loc_texture, 0);
    }
    if (loc_alpha >= 0) glUniform1f(loc_alpha, 1.0f);
    
    glBindVertexArray(icon_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, icon_vbo_);
    
    // Render icon for each bounding box
    std::vector<float> icon_vertices;
    
    for (const auto& bbox : bounding_boxes_) {
        if (!bbox.visible || bbox.corners.size() != 8) continue;
        
        // Calculate center of top face
        Eigen::Vector3f top_center = Eigen::Vector3f::Zero();
        for (int i = 4; i < 8; ++i) {
            top_center += bbox.corners[i];
        }
        top_center /= 4.0f;
        
        // Icon size is fixed, not dependent on bounding box size (5% smaller)
        float icon_half_width = icon_size_ * 0.5f * 0.95f;
        float icon_height = icon_half_width * 1.2f; // Make it slightly taller
        
        // Calculate box height for positioning
        float box_height = std::abs(bbox.corners[4].z() - bbox.corners[0].z());
        
        // Calculate forward direction from top face (for billboard orientation)
        Eigen::Vector3f v1 = bbox.corners[5] - bbox.corners[4]; // Right edge of top face
        Eigen::Vector3f forward = v1.normalized();
        
        // Create a vertical standing icon (billboard-style, standing up)
        // Right vector: perpendicular to forward, in XY plane
        Eigen::Vector3f right = Eigen::Vector3f(-forward.y(), forward.x(), 0.0f).normalized();
        if (right.norm() < 1e-6f) {
            // Fallback if forward is vertical
            right = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        }
        
        // Up vector: always vertical (Z axis)
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        
        // Position icon standing on top center, with icon_z_offset applied
        Eigen::Vector3f icon_bottom_center = top_center;
        icon_bottom_center.z() += bbox.icon_z_offset;
        
        // Create vertical quad vertices (standing icon, 2 triangles)
        Eigen::Vector3f corners[4] = {
            icon_bottom_center - right * icon_half_width,                    // bottom-left
            icon_bottom_center + right * icon_half_width,                    // bottom-right
            icon_bottom_center + right * icon_half_width + up * icon_height, // top-right
            icon_bottom_center - right * icon_half_width + up * icon_height  // top-left
        };
        
        // Triangle 1 (flip V coordinate to fix upside-down issue)
        icon_vertices.push_back(corners[0].x()); icon_vertices.push_back(corners[0].y()); icon_vertices.push_back(corners[0].z());
        icon_vertices.push_back(0.0f); icon_vertices.push_back(1.0f); // tex coord (flipped V)
        
        icon_vertices.push_back(corners[1].x()); icon_vertices.push_back(corners[1].y()); icon_vertices.push_back(corners[1].z());
        icon_vertices.push_back(1.0f); icon_vertices.push_back(1.0f);
        
        icon_vertices.push_back(corners[2].x()); icon_vertices.push_back(corners[2].y()); icon_vertices.push_back(corners[2].z());
        icon_vertices.push_back(1.0f); icon_vertices.push_back(0.0f);
        
        // Triangle 2
        icon_vertices.push_back(corners[0].x()); icon_vertices.push_back(corners[0].y()); icon_vertices.push_back(corners[0].z());
        icon_vertices.push_back(0.0f); icon_vertices.push_back(1.0f);
        
        icon_vertices.push_back(corners[2].x()); icon_vertices.push_back(corners[2].y()); icon_vertices.push_back(corners[2].z());
        icon_vertices.push_back(1.0f); icon_vertices.push_back(0.0f);
        
        icon_vertices.push_back(corners[3].x()); icon_vertices.push_back(corners[3].y()); icon_vertices.push_back(corners[3].z());
        icon_vertices.push_back(0.0f); icon_vertices.push_back(0.0f);
    }
    
    if (!icon_vertices.empty()) {
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        icon_vertices.size() * sizeof(float),
                        icon_vertices.data());
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(icon_vertices.size() / 5));
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
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

