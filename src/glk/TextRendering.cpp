#include <glk/TextRendering.hpp>
#include <iostream>

// Constructor/Destructor
TextRenderer::TextRenderer() = default;
TextRenderer::~TextRenderer() { cleanupTextRendering(); }

bool TextRenderer::initTextRendering(const std::string &font_path)
{
    // Initialize FreeType library
    if (FT_Init_FreeType(&ft_library_))
    {
        std::cerr << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        return false;
    }

    // Load font
    if (FT_New_Face(ft_library_, font_path.c_str(), 0, &ft_face_))
    {
        std::cerr << "ERROR::FREETYPE: Failed to load font" << std::endl;
        return false;
    }

    // Set font size (smaller values = smaller text)
    FT_Set_Pixel_Sizes(ft_face_, 0, 30);

    // Disable byte-alignment restriction
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Load first 128 characters of ASCII set
    for (unsigned char c = 0; c < 128; c++)
    {
        // Load character glyph
        if (FT_Load_Char(ft_face_, c, FT_LOAD_RENDER))
        {
            std::cerr << "ERROR::FREETYPE: Failed to load Glyph" << std::endl;
            continue;
        }

        // Generate texture
        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            ft_face_->glyph->bitmap.width,
            ft_face_->glyph->bitmap.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            ft_face_->glyph->bitmap.buffer);

        // Set texture options
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Store character for later use
        Character character = {
            texture,
            Eigen::Vector2i(ft_face_->glyph->bitmap.width, ft_face_->glyph->bitmap.rows),
            Eigen::Vector2i(ft_face_->glyph->bitmap_left, ft_face_->glyph->bitmap_top),
            static_cast<unsigned int>(ft_face_->glyph->advance.x)};
        characters_.insert(std::pair<char, Character>(c, character));
    }

    glBindTexture(GL_TEXTURE_2D, 0);

    // Clean up FreeType resources
    FT_Done_Face(ft_face_);
    FT_Done_FreeType(ft_library_);

    // Create text rendering VAO/VBO
    glGenVertexArrays(1, &text_vao_);
    glGenBuffers(1, &text_vbo_);
    glBindVertexArray(text_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, text_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Create background quad VAO/VBO
    // glGenVertexArrays(1, &bg_vao_);
    // glGenBuffers(1, &bg_vbo_);
    // glBindVertexArray(bg_vao_);
    // glBindBuffer(GL_ARRAY_BUFFER, bg_vbo_);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 3, NULL, GL_DYNAMIC_DRAW);
    // glEnableVertexAttribArray(0);
    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    // glBindBuffer(GL_ARRAY_BUFFER, 0);
    // glBindVertexArray(0);

    glGenVertexArrays(1, &bg_vao_);
    glGenBuffers(1, &bg_vbo_);
    glBindVertexArray(bg_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, bg_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 5, NULL, GL_DYNAMIC_DRAW); // 5 components per vertex
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Create shaders
    if (!createTextShaders())
    {
        std::cerr << "Failed to create text shaders" << std::endl;
        return false;
    }

    return true;
}

void TextRenderer::addWorldText(const std::string &text, const Eigen::Vector3f &world_pos,
                                const Eigen::Vector3f &color,
                                float scale,
                                float corner_radius)
{
    TextQuad quad;
    quad.text = text;
    quad.position = world_pos;
    quad.color = color;
    quad.scale = scale;
    quad.corner_radius = corner_radius;
    quad.visible = true;

    // Calculate text size and baseline info for background using individual scale
    float text_width = 0.0f;
    float max_bearing_y = 0.0f;
    float min_bearing_y = 0.0f;
    float effective_scale = text_scale_ * scale;

    for (char c : text)
    {
        Character ch = characters_[c];
        text_width += (ch.advance >> 6) * effective_scale;

        // Track the highest and lowest points of the text
        float top = ch.bearing.y() * effective_scale;
        float bottom = (ch.bearing.y() - ch.size.y()) * effective_scale;

        max_bearing_y = std::max(max_bearing_y, top);
        min_bearing_y = std::min(min_bearing_y, bottom);
    }

    float text_height = max_bearing_y - min_bearing_y;

    // Store both size and baseline offset for proper centering
    quad.size = Eigen::Vector2f(text_width, text_height);
    quad.baseline_offset = (max_bearing_y + min_bearing_y) / 2.0f; // Center offset

    text_quads_.push_back(quad);
}

void TextRenderer::renderWorldText(const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (const auto &quad : text_quads_)
    {
        if (!quad.visible)
            continue;

        // Calculate billboard transformation (always face camera)
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

        // Extract camera position from inverse view matrix
        Eigen::Matrix4f inv_view = view.inverse();
        Eigen::Vector3f camera_pos = inv_view.block<3, 1>(0, 3);

        // Calculate direction from text to camera
        Eigen::Vector3f to_camera = (camera_pos - quad.position).normalized();

        // Use world up vector (adjust if the world uses different up axis)
        Eigen::Vector3f world_up(0.0f, 0.0f, 1.0f);

        // Calculate right vector
        Eigen::Vector3f right = world_up.cross(to_camera).normalized();

        // Recalculate up vector to ensure orthogonality
        Eigen::Vector3f up = to_camera.cross(right).normalized();

        // Build the billboard rotation matrix
        model.block<3, 1>(0, 0) = right;
        model.block<3, 1>(0, 1) = up;
        model.block<3, 1>(0, 2) = to_camera;
        model.block<3, 1>(0, 3) = quad.position;

        // Render background first (offset backwards)
        Eigen::Matrix4f bg_model = model;
        float background_offset = 0.02f; // Move background slightly away from the text
        bg_model.block<3, 1>(0, 3) = quad.position - to_camera * background_offset;
        renderTextBackground(quad, bg_model, view, projection);

        // Then render text
        renderText(quad, model, view, projection);
    }

    glDisable(GL_BLEND);
}
void TextRenderer::clearWorldText()
{
    text_quads_.clear();
}

void TextRenderer::setTextScale(float scale)
{
    text_scale_ = scale;
}

void TextRenderer::cleanupTextRendering()
{
    if (text_vao_ != 0)
    {
        glDeleteVertexArrays(1, &text_vao_);
        glDeleteBuffers(1, &text_vbo_);
    }
    if (bg_vao_ != 0)
    {
        glDeleteVertexArrays(1, &bg_vao_);
        glDeleteBuffers(1, &bg_vbo_);
    }
    if (text_shader_program_ != 0)
    {
        glDeleteProgram(text_shader_program_);
    }
    if (bg_shader_program_ != 0)
    {
        glDeleteProgram(bg_shader_program_);
    }

    // Clean up character textures
    for (auto &pair : characters_)
    {
        glDeleteTextures(1, &pair.second.textureID);
    }
    characters_.clear();
}

bool TextRenderer::createTextShaders()
{
    // Text vertex shader
    const char *text_vertex_shader = R"(
        #version 460 core
        layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
        out vec2 TexCoords;
        
        uniform mat4 projection;
        uniform mat4 view;
        uniform mat4 model;
        
        void main() {
            gl_Position = projection * view * model * vec4(vertex.xy, 0.0, 1.0);
            TexCoords = vertex.zw;
        }
    )";

    // Text fragment shader
    const char *text_fragment_shader = R"(
        #version 460 core
        in vec2 TexCoords;
        out vec4 color;
        
        uniform sampler2D text;
        uniform vec3 textColor;
        
        void main() {
            vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
            color = vec4(textColor, 1.0) * sampled;
        }
    )";

    // Background vertex shader
    // const char *bg_vertex_shader = R"(
    //     #version 460 core
    //     layout (location = 0) in vec3 aPos;

    //     uniform mat4 projection;
    //     uniform mat4 view;
    //     uniform mat4 model;

    //     void main() {
    //         gl_Position = projection * view * model * vec4(aPos, 1.0);
    //     }
    // )";

    const char *bg_vertex_shader = R"(
        #version 460 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aUV;  // Add UV coordinates
        
        uniform mat4 projection;
        uniform mat4 view;
        uniform mat4 model;
        
        out vec2 uv;
        
        void main() {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            uv = aUV;
        }
    )";

    // Background fragment shader
    // const char *bg_fragment_shader = R"(
    //     #version 460 core
    //     out vec4 FragColor;

    //     uniform vec3 bgColor;

    //     void main() {
    //         FragColor = vec4(bgColor, 0.8); // Semi-transparent background
    //     }
    // )";

    const char *bg_fragment_shader = R"(
        #version 460 core
        in vec2 uv;
        out vec4 FragColor;
        
        uniform vec3 bgColor;
        uniform float cornerRadius;
        
        void main() {
            vec2 pos = uv * 2.0 - 1.0;  // Convert UV to [-1, 1] range
            
            // Calculate distance to rounded rectangle corners
            vec2 d = abs(pos) - (1.0 - cornerRadius);
            float distance = min(max(d.x, d.y), 0.0) + length(max(d, 0.0)) - cornerRadius;
            
            float alpha = 1.0 - smoothstep(0.0, 0.02, distance);
            
            FragColor = vec4(bgColor, 0.8 * alpha);
        }
    )";

    // Compile and link text shader
    text_shader_program_ = compileShaderProgram(text_vertex_shader, text_fragment_shader);
    if (text_shader_program_ == 0)
        return false;

    // Compile and link background shader
    bg_shader_program_ = compileShaderProgram(bg_vertex_shader, bg_fragment_shader);
    if (bg_shader_program_ == 0)
        return false;

    return true;
}

GLuint TextRenderer::compileShaderProgram(const char *vertex_source, const char *fragment_source)
{
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);

    GLint success;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "Vertex shader compilation failed: " << info_log << std::endl;
        return 0;
    }

    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);

    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "Fragment shader compilation failed: " << info_log << std::endl;
        return 0;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetProgramInfoLog(program, 512, NULL, info_log);
        std::cerr << "Shader program linking failed: " << info_log << std::endl;
        return 0;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

void TextRenderer::renderTextBackground(const TextQuad &quad, const Eigen::Matrix4f &model,
                                        const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection)
{
    glUseProgram(bg_shader_program_);

    // Set uniforms
    glUniformMatrix4fv(glGetUniformLocation(bg_shader_program_, "projection"), 1, GL_FALSE, projection.data());
    glUniformMatrix4fv(glGetUniformLocation(bg_shader_program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(bg_shader_program_, "model"), 1, GL_FALSE, model.data());
    glUniform3f(glGetUniformLocation(bg_shader_program_, "bgColor"), 0.0f, 0.0f, 0.0f);

    glUniform1f(glGetUniformLocation(bg_shader_program_, "cornerRadius"), quad.corner_radius);

    // Create background quad slightly larger than text (use individual scale)
    float padding = quad.scale;
    float width = quad.size.x() + padding * 2.0f;
    float height = quad.size.y() + padding * 2.0f;

    // float vertices[] = {
    //     -width / 2.0f, -height / 2.0f, 0.0f,
    //     width / 2.0f, -height / 2.0f, 0.0f,
    //     width / 2.0f, height / 2.0f, 0.0f,
    //     -width / 2.0f, -height / 2.0f, 0.0f,
    //     width / 2.0f, height / 2.0f, 0.0f,
    //     -width / 2.0f, height / 2.0f, 0.0f};

    float vertices[] = {
        // Position         // UV
        -width / 2.0f, -height / 2.0f, 0.0f, 0.0f, 0.0f,
        width / 2.0f, -height / 2.0f, 0.0f, 1.0f, 0.0f,
        width / 2.0f, height / 2.0f, 0.0f, 1.0f, 1.0f,
        -width / 2.0f, -height / 2.0f, 0.0f, 0.0f, 0.0f,
        width / 2.0f, height / 2.0f, 0.0f, 1.0f, 1.0f,
        -width / 2.0f, height / 2.0f, 0.0f, 0.0f, 1.0f};

    glBindVertexArray(bg_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, bg_vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}
void TextRenderer::renderText(const TextQuad &quad, const Eigen::Matrix4f &model,
                              const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection)
{
    glUseProgram(text_shader_program_);

    // Set uniforms
    glUniformMatrix4fv(glGetUniformLocation(text_shader_program_, "projection"), 1, GL_FALSE, projection.data());
    glUniformMatrix4fv(glGetUniformLocation(text_shader_program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(text_shader_program_, "model"), 1, GL_FALSE, model.data());
    glUniform3f(glGetUniformLocation(text_shader_program_, "textColor"),
                quad.color.x(), quad.color.y(), quad.color.z());

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(text_vao_);

    // Use individual scale for this text
    float effective_scale = text_scale_ * quad.scale;

    // Calculate starting position (centered)
    float text_width = 0.0f;
    for (char c : quad.text)
    {
        Character ch = characters_[c];
        text_width += (ch.advance >> 6) * effective_scale;
    }

    float x = -text_width / 2.0f;
    float y = -quad.baseline_offset;

    // Iterate through all characters
    for (char c : quad.text)
    {
        Character ch = characters_[c];

        float xpos = x + ch.bearing.x() * effective_scale;
        float ypos = y - (ch.size.y() - ch.bearing.y()) * effective_scale;

        float w = ch.size.x() * effective_scale;
        float h = ch.size.y() * effective_scale;

        // Update VBO for each character
        float vertices[6][4] = {
            {xpos, ypos + h, 0.0f, 0.0f},
            {xpos, ypos, 0.0f, 1.0f},
            {xpos + w, ypos, 1.0f, 1.0f},

            {xpos, ypos + h, 0.0f, 0.0f},
            {xpos + w, ypos, 1.0f, 1.0f},
            {xpos + w, ypos + h, 1.0f, 0.0f}};

        // Render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, ch.textureID);
        glBindBuffer(GL_ARRAY_BUFFER, text_vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // Advance cursors for next glyph
        x += (ch.advance >> 6) * effective_scale;
    }

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}