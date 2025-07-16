#pragma once

#include <iostream>
#include <vector>
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
    bool initialize()
    {
        if (!createShader())
        {
            std::cerr << "Failed to create thick lines shader" << std::endl;
            return false;
        }

        setupBuffers();
        std::cout << "\033[1;32mThick lines renderer initialized successfully\033[0m" << std::endl;
        return true;
    }

    // Draw coordinate frames with thick lines
    void drawCoordinateFrames(const Eigen::Matrix4f &view,
                              const Eigen::Matrix4f &projection,
                              const Eigen::Vector2i &viewport_size,
                              const std::string &fixed_frame,
                              const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms,
                              float frame_size,
                              float line_thickness)
    {
        if (shader_program_ == 0)
            return;

        // Save current OpenGL state
        GLint previous_program;
        glGetIntegerv(GL_CURRENT_PROGRAM, &previous_program);

        GLint previous_vao;
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previous_vao);

        // Use our shader program
        glUseProgram(shader_program_);

        // Set uniforms
        glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view_matrix"),
                           1, GL_FALSE, view.data());
        glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection_matrix"),
                           1, GL_FALSE, projection.data());
        glUniform1f(glGetUniformLocation(shader_program_, "cylinder_radius"), line_thickness * 0.01f);
        glUniform1f(glGetUniformLocation(shader_program_, "cylinder_length"), frame_size);

        glBindVertexArray(vao_);

        // Draw fixed frame at origin
        if (!fixed_frame.empty())
        {
            Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
            glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model_matrix"),
                               1, GL_FALSE, model.data());
            glDrawArrays(GL_POINTS, 0, 3); // 3 points = 3 arrows (X, Y, Z axes)
        }

        // Draw all other frames
        for (const auto &[frame_name, transform] : frame_transforms)
        {
            Eigen::Matrix4f model = transform.matrix();
            glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model_matrix"),
                               1, GL_FALSE, model.data());
            glDrawArrays(GL_POINTS, 0, 3); // 3 points = 3 arrows (X, Y, Z axes)
        }

        // Restore previous OpenGL state
        glBindVertexArray(previous_vao);
        glUseProgram(previous_program);
    }

    // Cleanup resources
    void cleanup()
    {
        if (vao_ != 0)
        {
            glDeleteVertexArrays(1, &vao_);
            glDeleteBuffers(1, &vbo_);
            vao_ = 0;
            vbo_ = 0;
        }
        if (shader_program_ != 0)
        {
            glDeleteProgram(shader_program_);
            shader_program_ = 0;
        }
    }

private:
    GLuint shader_program_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;

    struct AxisData
    {
        int axis_id; // 0=X, 1=Y, 2=Z
    };

    bool createShader()
    {
        const char *vertex_shader_source = R"(
            #version 330 core
            layout (location = 0) in int axis_id;
            
            uniform mat4 view_matrix;
            uniform mat4 projection_matrix;
            uniform mat4 model_matrix;
            
            flat out int v_axis_id;
            
            void main() {
                v_axis_id = axis_id;
                gl_Position = vec4(0.0, 0.0, 0.0, 1.0); // Will be expanded in geometry shader
            }
        )";

        const char *geometry_shader_source = R"(
            #version 330 core
            layout (points) in;
            layout (triangle_strip, max_vertices = 128) out;
            
            uniform mat4 view_matrix;
            uniform mat4 projection_matrix;
            uniform mat4 model_matrix;
            uniform float cylinder_radius;
            uniform float cylinder_length;
            
            flat in int v_axis_id[];
            out vec3 frag_color;
            out vec3 world_pos;
            out vec3 world_normal;
            
            const int SEGMENTS = 12;
            const float PI = 3.14159265359;
            
            void emitVertex(vec3 pos, vec3 normal, vec3 color) {
                vec4 world_position = model_matrix * vec4(pos, 1.0);
                world_pos = world_position.xyz;
                world_normal = normalize((model_matrix * vec4(normal, 0.0)).xyz);
                frag_color = color;
                gl_Position = projection_matrix * view_matrix * world_position;
                EmitVertex();
            }
            
            void generateCylinder(vec3 start, vec3 end, float radius, vec3 axis_dir, vec3 right, vec3 up, vec3 color) {
                // Generate cylinder wall
                for (int i = 0; i <= SEGMENTS; i++) {
                    float angle = float(i) * 2.0 * PI / float(SEGMENTS);
                    float cos_a = cos(angle);
                    float sin_a = sin(angle);
                    
                    vec3 normal = normalize(right * cos_a + up * sin_a);
                    vec3 offset = normal * radius;
                    
                    emitVertex(start + offset, normal, color);
                    emitVertex(end + offset, normal, color);
                }
                EndPrimitive();
            }
            
            void generateBottomCap(vec3 center, vec3 normal, float radius, vec3 right, vec3 up, vec3 color) {
                // Generate bottom cap as individual triangles
                for (int i = 0; i < SEGMENTS; i++) {
                    float angle1 = float(i) * 2.0 * PI / float(SEGMENTS);
                    float angle2 = float(i + 1) * 2.0 * PI / float(SEGMENTS);
                    
                    float cos1 = cos(angle1);
                    float sin1 = sin(angle1);
                    float cos2 = cos(angle2);
                    float sin2 = sin(angle2);
                    
                    vec3 offset1 = (right * cos1 + up * sin1) * radius;
                    vec3 offset2 = (right * cos2 + up * sin2) * radius;
                    
                    // Create triangle: center -> point1 -> point2
                    emitVertex(center, normal, color);
                    emitVertex(center + offset1, normal, color);
                    emitVertex(center + offset2, normal, color);
                    EndPrimitive();
                }
            }
            
            void generateTopCap(vec3 center, vec3 normal, float radius, vec3 right, vec3 up, vec3 color) {
                // Generate top cap as individual triangles (reverse winding)
                for (int i = 0; i < SEGMENTS; i++) {
                    float angle1 = float(i) * 2.0 * PI / float(SEGMENTS);
                    float angle2 = float(i + 1) * 2.0 * PI / float(SEGMENTS);
                    
                    float cos1 = cos(angle1);
                    float sin1 = sin(angle1);
                    float cos2 = cos(angle2);
                    float sin2 = sin(angle2);
                    
                    vec3 offset1 = (right * cos1 + up * sin1) * radius;
                    vec3 offset2 = (right * cos2 + up * sin2) * radius;
                    
                    // Create triangle: center -> point2 -> point1 (reverse winding for outward normal)
                    emitVertex(center, normal, color);
                    emitVertex(center + offset2, normal, color);
                    emitVertex(center + offset1, normal, color);
                    EndPrimitive();
                }
            }
            
            void main() {
                int axis = v_axis_id[0];
                
                // Define axis direction and color
                vec3 axis_dir;
                vec3 color;
                
                if (axis == 0) { // X-axis
                    axis_dir = vec3(1.0, 0.0, 0.0);
                    color = vec3(1.0, 0.0, 0.0); // Red
                } else if (axis == 1) { // Y-axis
                    axis_dir = vec3(0.0, 1.0, 0.0);
                    color = vec3(0.0, 1.0, 0.0); // Green
                } else { // Z-axis
                    axis_dir = vec3(0.0, 0.0, 1.0);
                    color = vec3(0.0, 0.0, 1.0); // Blue
                }
                
                // Create two perpendicular vectors to axis_dir
                vec3 up = abs(axis_dir.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
                vec3 right = normalize(cross(axis_dir, up));
                up = normalize(cross(right, axis_dir));
                
                // Simple cylinder from origin to full length
                vec3 start = vec3(0.0);
                vec3 end = axis_dir * cylinder_length;
                
                // 1. Generate cylinder wall
                generateCylinder(start, end, cylinder_radius, axis_dir, right, up, color);
                
                // 2. Generate bottom cap (at the beginning/origin)
                generateBottomCap(start, -axis_dir, cylinder_radius, right, up, color);
                
                // 3. Generate top cap (at the end)
                generateTopCap(end, axis_dir, cylinder_radius, right, up, color);
            }
        )";

        const char *fragment_shader_source = R"(
            #version 330 core
            in vec3 frag_color;
            in vec3 world_pos;
            in vec3 world_normal;
            out vec4 FragColor;
            
            void main() {
                // Enhanced lighting with specular highlights
                vec3 light_dir = normalize(vec3(1.0, 1.0, 1.0));
                vec3 view_dir = normalize(-world_pos); // Assuming camera at origin for simplicity
                vec3 normal = normalize(world_normal);
                
                // Ambient
                float ambient = 0.2;
                
                // Diffuse
                float diffuse = max(dot(normal, light_dir), 0.0) * 0.7;
                
                // Specular
                vec3 reflect_dir = reflect(-light_dir, normal);
                float specular = pow(max(dot(view_dir, reflect_dir), 0.0), 32.0) * 0.3;
                
                vec3 lit_color = frag_color * (ambient + diffuse) + vec3(1.0) * specular;
                FragColor = vec4(lit_color, 1.0);
            }
        )";

        // Compile vertex shader
        GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
        glCompileShader(vertex_shader);

        if (!checkShaderCompilation(vertex_shader, "Vertex"))
        {
            return false;
        }

        // Compile geometry shader
        GLuint geometry_shader = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry_shader, 1, &geometry_shader_source, NULL);
        glCompileShader(geometry_shader);

        if (!checkShaderCompilation(geometry_shader, "Geometry"))
        {
            glDeleteShader(vertex_shader);
            return false;
        }

        // Compile fragment shader
        GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
        glCompileShader(fragment_shader);

        if (!checkShaderCompilation(fragment_shader, "Fragment"))
        {
            glDeleteShader(vertex_shader);
            glDeleteShader(geometry_shader);
            return false;
        }

        // Create and link program
        shader_program_ = glCreateProgram();
        glAttachShader(shader_program_, vertex_shader);
        glAttachShader(shader_program_, geometry_shader);
        glAttachShader(shader_program_, fragment_shader);
        glLinkProgram(shader_program_);

        // Check linking
        GLint success;
        glGetProgramiv(shader_program_, GL_LINK_STATUS, &success);
        if (!success)
        {
            char info_log[512];
            glGetProgramInfoLog(shader_program_, 512, NULL, info_log);
            std::cerr << "Shader program linking failed: " << info_log << std::endl;
            glDeleteShader(vertex_shader);
            glDeleteShader(geometry_shader);
            glDeleteShader(fragment_shader);
            return false;
        }

        // Clean up individual shaders
        glDeleteShader(vertex_shader);
        glDeleteShader(geometry_shader);
        glDeleteShader(fragment_shader);

        return true;
    }

    bool checkShaderCompilation(GLuint shader, const std::string &type)
    {
        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            char info_log[512];
            glGetShaderInfoLog(shader, 512, NULL, info_log);
            std::cerr << type << " shader compilation failed: " << info_log << std::endl;
            return false;
        }
        return true;
    }

    void setupBuffers()
    {
        // Create axis data for X, Y, Z axes
        std::vector<AxisData> axes;
        axes.push_back({0}); // X-axis
        axes.push_back({1}); // Y-axis
        axes.push_back({2}); // Z-axis

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, axes.size() * sizeof(AxisData), axes.data(), GL_STATIC_DRAW);

        // Axis ID attribute (location = 0)
        glEnableVertexAttribArray(0);
        glVertexAttribIPointer(0, 1, GL_INT, sizeof(AxisData), (void *)0);

        glBindVertexArray(0);
    }
};