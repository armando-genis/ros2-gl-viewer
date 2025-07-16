#pragma once

#include <iostream>
#include <vector>
#include <GL/gl3w.h>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ft2build.h>
#include FT_FREETYPE_H

struct Character
{
    unsigned int textureID;  // ID handle of the glyph texture
    Eigen::Vector2i size;    // Size of glyph
    Eigen::Vector2i bearing; // Offset from baseline to left/top of glyph
    unsigned int advance;    // Horizontal offset to advance to next glyph
};

struct TextQuad
{
    Eigen::Vector3f position;
    Eigen::Vector2f size;
    std::string text;
    Eigen::Vector3f color;
    float scale;           // Individual scale for each text
    float baseline_offset; // Offset for proper centering
    float corner_radius = 0.0f;
    bool visible;
};
class TextRenderer
{
public:
    TextRenderer();
    ~TextRenderer();

    // Initialize text rendering system
    bool initTextRendering(const std::string &font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");

    // Add text to be rendered at world position
    void addWorldText(const std::string &text, const Eigen::Vector3f &world_pos,
                      const Eigen::Vector3f &color = Eigen::Vector3f(1.0f, 1.0f, 1.0f),
                      float scale = 1.0f,
                      float corner_radius = 0.0f);

    // Render all text quads
    void renderWorldText(const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection);

    // Clear all text quads
    void clearWorldText();
    // Set text scale
    void setTextScale(float scale);

    // Cleanup text rendering resources
    void cleanupTextRendering();

private:
    // FreeType and text rendering
    FT_Library ft_library_;
    FT_Face ft_face_;
    std::map<GLchar, Character> characters_;

    // Text rendering OpenGL objects
    GLuint text_vao_ = 0;
    GLuint text_vbo_ = 0;
    GLuint text_shader_program_ = 0;

    // Background quad for text
    GLuint bg_vao_ = 0;
    GLuint bg_vbo_ = 0;
    GLuint bg_shader_program_ = 0;

    // Text management
    std::vector<TextQuad> text_quads_;
    float text_scale_ = 1.0f;

    // Create text and background shaders
    bool createTextShaders();

    // Helper function to compile shader program
    GLuint compileShaderProgram(const char *vertex_source, const char *fragment_source);

    // Render text background
    void renderTextBackground(const TextQuad &quad, const Eigen::Matrix4f &model,
                              const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection);

    // Render text characters
    void renderText(const TextQuad &quad, const Eigen::Matrix4f &model,
                    const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection);
};