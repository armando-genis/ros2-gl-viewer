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

void BoxRenderer::addBox(const std::vector<Eigen::Vector2f>& corners,
                         float z_offset,
                         float z_height,
                         const Eigen::Vector3f& color,
                         float alpha,
                         int colorMode) {
    if (corners.size() != 4) {
        std::cerr << "Box must have exactly 4 corners" << std::endl;
        return;
    }
    
    Box box;
    box.corners = corners;
    box.z_offset = z_offset;
    box.z_height = z_height;
    box.color = color;
    box.alpha = alpha;
    box.colorMode = colorMode;
    box.visible = true;
    
    // Generate geometry for this box
    generateBoxGeometry(box);
    
    boxes_.push_back(box);
}

void BoxRenderer::renderBoxes(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection) {
    if (boxes_.empty() || box_shader_program_ == 0) return;

    glUseProgram(box_shader_program_);

    // cache uniform locations
    static GLuint cached_prog = 0;
    static GLint loc_view=-1, loc_proj=-1, loc_color=-1, loc_alpha=-1;
    static GLint loc_zbottom=-1, loc_zheight=-1, loc_radius=-1, loc_corners0=-1, loc_solid_frac=-1;

    if (cached_prog != box_shader_program_) {
        cached_prog      = box_shader_program_;
        loc_view         = glGetUniformLocation(box_shader_program_, "view");
        loc_proj         = glGetUniformLocation(box_shader_program_, "projection");
        loc_color        = glGetUniformLocation(box_shader_program_, "boxColor");
        loc_alpha        = glGetUniformLocation(box_shader_program_, "boxAlpha");
        loc_zbottom      = glGetUniformLocation(box_shader_program_, "z_bottom");
        loc_zheight      = glGetUniformLocation(box_shader_program_, "z_height");
        loc_radius       = glGetUniformLocation(box_shader_program_, "corner_radius");
        loc_corners0     = glGetUniformLocation(box_shader_program_, "corners[0]");
        loc_solid_frac   = glGetUniformLocation(box_shader_program_, "solid_frac");
    }

    // global uniforms
    if (loc_view >= 0) glUniformMatrix4fv(loc_view, 1, GL_FALSE, view.data());
    if (loc_proj >= 0) glUniformMatrix4fv(loc_proj, 1, GL_FALSE, projection.data());
    if (loc_solid_frac >= 0) glUniform1f(loc_solid_frac, 0.4f); // 40% solid, then fade

    // Translucent pass with premultiplied alpha
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);                  // don't write depth (avoid self-occlusion artifacts)
    glDisable(GL_CULL_FACE);                // show both sides of thin walls if needed

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // PREMULTIPLIED ALPHA

    glBindVertexArray(box_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, box_vbo_);

    for (const auto& box : boxes_) {
        if (!box.visible || box.vertices.empty()) continue;

        // corner radius heuristic
        float avg_len = 0.0f;
        for (int i = 0; i < 4; ++i) {
            int ni = (i + 1) & 3;
            avg_len += (box.corners[ni] - box.corners[i]).norm();
        }
        avg_len *= 0.25f;
        float r = std::min(std::max(avg_len * 0.15f, 0.1f), 0.5f);

        // per-box uniforms
        if (loc_color   >= 0) glUniform3f(loc_color,  box.color.x(), box.color.y(), box.color.z());
        if (loc_alpha   >= 0) glUniform1f(loc_alpha,  box.alpha);
        if (loc_zbottom >= 0) glUniform1f(loc_zbottom, box.z_offset);
        if (loc_zheight >= 0) glUniform1f(loc_zheight, box.z_height);
        if (loc_radius  >= 0) glUniform1f(loc_radius,  r);

        // corners array (xy + radius in .z for future per-corner control)
        float cdata[16];
        for (int i = 0; i < 4; ++i) {
            cdata[i*4 + 0] = box.corners[i].x();
            cdata[i*4 + 1] = box.corners[i].y();
            cdata[i*4 + 2] = r;      // put the same r for now
            cdata[i*4 + 3] = 0.0f;
        }
        if (loc_corners0 >= 0) glUniform4fv(loc_corners0, 4, cdata);

        // upload and draw this box
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(box.vertices.size() * sizeof(BoxVertex)),
                        box.vertices.data());
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(box.vertices.size()));
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}


void BoxRenderer::clearBoxes() {
    boxes_.clear();
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
        layout (location = 1) in vec2 cornerIndex;   // unused, for compatibility
        layout (location = 2) in float colorMode;

        out vec2 WorldPos;     // world XY (for SDF)
        out float ZCoord;      // world Z (for vertical profile)
        out float ColorModeV;

        uniform mat4 view;
        uniform mat4 projection;

        void main() {
            gl_Position = projection * view * vec4(position, 1.0);
            WorldPos    = position.xy;
            ZCoord      = position.z;
            ColorModeV  = colorMode;
        }
    )";

    const char* fragment_shader = R"(
        #version 460 core
        in vec2 WorldPos;
        in float ZCoord;
        in float ColorModeV;
        out vec4 FragColor;

        uniform vec3  boxColor;
        uniform float boxAlpha;
        uniform vec4  corners[4];   // xy = footprint corners; z can carry per-corner radius (unused here)
        uniform float z_bottom;
        uniform float z_height;
        uniform float corner_radius;
        uniform float solid_frac;   // 0..1: solid up to this fraction of height, then fade to 0 at top

        // Signed distance to rounded rectangle centered at origin (half-size b, radius r)
        float sdRoundedBox(vec2 p, vec2 b, float r) {
            vec2 q = abs(p) - (b - vec2(r));
            return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - r;
        }

        // Build a local frame + half-sizes from arbitrary 4-point quad (robust to winding/skew)
        void boxLocalFrame(out vec2 center, out vec2 ex, out vec2 ey, out vec2 halfSize, out float r) {
            vec2 c0 = corners[0].xy;
            vec2 c1 = corners[1].xy;
            vec2 c2 = corners[2].xy;
            vec2 c3 = corners[3].xy;

            center = 0.25 * (c0 + c1 + c2 + c3);

            // Edges
            vec2 e01 = c1 - c0;
            vec2 e12 = c2 - c1;
            vec2 e23 = c3 - c2;
            vec2 e30 = c0 - c3;

            float l01 = length(e01);
            float l12 = length(e12);
            float l23 = length(e23);
            float l30 = length(e30);

            // Longest edge defines X axis; opposite edge refines scale
            int idx = 0;
            float lmax = l01;
            if (l12 > lmax) { lmax = l12; idx = 1; }
            if (l23 > lmax) { lmax = l23; idx = 2; }
            if (l30 > lmax) { lmax = l30; idx = 3; }

            vec2 ex_edge, ex_opp, ey_edge, ey_opp;
            float lx, lxo, ly, lyo;
            if (idx == 0) { ex_edge=e01; ex_opp=e23; lx=l01; lxo=l23; ey_edge=e12; ey_opp=e30; ly=l12; lyo=l30; }
            else if (idx == 1) { ex_edge=e12; ex_opp=e30; lx=l12; lxo=l30; ey_edge=e23; ey_opp=e01; ly=l23; lyo=l01; }
            else if (idx == 2) { ex_edge=e23; ex_opp=e01; lx=l23; lxo=l01; ey_edge=e30; ey_opp=e12; ly=l30; lyo=l12; }
            else { ex_edge=e30; ex_opp=e12; lx=l30; lxo=l12; ey_edge=e01; ey_opp=e23; ly=l01; lyo=l23; }

            ex = (lx > 1e-6) ? ex_edge / lx : vec2(1.0, 0.0);
            // Gram–Schmidt to get ey orthonormal to ex
            vec2 ey_proj = ey_edge - ex * dot(ey_edge, ex);
            float eyl = length(ey_proj);
            ey = (eyl > 1e-6) ? ey_proj / eyl : vec2(-ex.y, ex.x);

            halfSize = 0.5 * vec2((lx + lxo) * 0.5, (ly + lyo) * 0.5);

            // final radius: uniform or avg of per-corner .z if you ever pass them
            float r0 = max(corners[0].z, 0.0);
            float r1 = max(corners[1].z, 0.0);
            float r2 = max(corners[2].z, 0.0);
            float r3 = max(corners[3].z, 0.0);
            float rAvg = 0.25 * (r0 + r1 + r2 + r3);
            r = (rAvg > 0.0) ? rAvg : corner_radius;
            r = min(r, min(halfSize.x, halfSize.y) * 0.999);
        }

        void main() {
            vec2 center, ex, ey, halfSize;
            float r;
            boxLocalFrame(center, ex, ey, halfSize, r);

            // box-aligned coordinates
            vec2 pLocal = vec2(dot(WorldPos - center, ex), dot(WorldPos - center, ey));

            // SDF to rounded rectangle footprint
            float dist = sdRoundedBox(pLocal, halfSize, r);

            // Stable AA band across triangles
            float aa = max(1e-6, 0.75 * length(vec2(dFdx(dist), dFdy(dist))));
            float inside = 1.0 - smoothstep(-aa, aa, dist);
            if (inside <= 0.0) discard;

            // Vertical profile: solid up to solid_frac of height, then fade to 0 at top
            float h = clamp((ZCoord - z_bottom) / max(z_height, 1e-6), 0.0, 1.0);
            float plateauAlpha = 1.0 - smoothstep(solid_frac, 1.0, h);

            float finalAlpha = boxAlpha * plateauAlpha * inside;
            if (finalAlpha < 0.01) discard;

            // PREMULTIPLIED alpha output
            FragColor = vec4(boxColor * finalAlpha, finalAlpha);
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

void BoxRenderer::generateBoxGeometry(Box& box) {
    box.vertices.clear();

    const float z_bottom = box.z_offset;
    const float z_top    = box.z_offset + box.z_height;

    // Calculate corner radius based on box size
    float avg_length = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
        size_t next_i = (i + 1) % 4;
        avg_length += (box.corners[next_i] - box.corners[i]).norm();
    }
    avg_length /= 4.0f;
    float corner_radius = std::min(avg_length * 0.15f, 0.5f);
    corner_radius = std::max(corner_radius, 0.1f);

    // -------------------------------
    // 1) Four walls only - no top face
    // -------------------------------
    for (size_t i = 0; i < 4; ++i) {
        size_t next_i = (i + 1) % 4;

        const Eigen::Vector2f& c1 = box.corners[i];
        const Eigen::Vector2f& c2 = box.corners[next_i];

        BoxVertex v1, v2, v3, v4, v5, v6;

        // First triangle of wall quad
        v1.position = Eigen::Vector3f(c1.x(), c1.y(), z_bottom);
        v1.texCoord = Eigen::Vector2f(float(i), 0.0f);
        v1.colorMode = float(box.colorMode);

        v2.position = Eigen::Vector3f(c2.x(), c2.y(), z_bottom);
        v2.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
        v2.colorMode = float(box.colorMode);

        v3.position = Eigen::Vector3f(c1.x(), c1.y(), z_top);
        v3.texCoord = Eigen::Vector2f(float(i), 1.0f);
        v3.colorMode = float(box.colorMode);

        // Second triangle of wall quad
        v4.position = Eigen::Vector3f(c2.x(), c2.y(), z_bottom);
        v4.texCoord = Eigen::Vector2f(float(next_i), 0.0f);
        v4.colorMode = float(box.colorMode);

        v5.position = Eigen::Vector3f(c2.x(), c2.y(), z_top);
        v5.texCoord = Eigen::Vector2f(float(next_i), 1.0f);
        v5.colorMode = float(box.colorMode);

        v6.position = Eigen::Vector3f(c1.x(), c1.y(), z_top);
        v6.texCoord = Eigen::Vector2f(float(i), 1.0f);
        v6.colorMode = float(box.colorMode);

        box.vertices.push_back(v1);
        box.vertices.push_back(v2);
        box.vertices.push_back(v3);
        box.vertices.push_back(v4);
        box.vertices.push_back(v5);
        box.vertices.push_back(v6);
    }

    // -------------------------------
    // 2) Bottom face only (two triangles)
    // -------------------------------
    {
        // Use the actual corners for the bottom face
        BoxVertex b1, b2, b3, b4, b5, b6;

        // First triangle
        b1.position = Eigen::Vector3f(box.corners[0].x(), box.corners[0].y(), z_bottom);
        b1.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        b1.colorMode = float(box.colorMode);

        b2.position = Eigen::Vector3f(box.corners[1].x(), box.corners[1].y(), z_bottom);
        b2.texCoord = Eigen::Vector2f(1.0f, 0.0f);
        b2.colorMode = float(box.colorMode);

        b3.position = Eigen::Vector3f(box.corners[2].x(), box.corners[2].y(), z_bottom);
        b3.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        b3.colorMode = float(box.colorMode);

        // Second triangle
        b4.position = Eigen::Vector3f(box.corners[0].x(), box.corners[0].y(), z_bottom);
        b4.texCoord = Eigen::Vector2f(0.0f, 0.0f);
        b4.colorMode = float(box.colorMode);

        b5.position = Eigen::Vector3f(box.corners[2].x(), box.corners[2].y(), z_bottom);
        b5.texCoord = Eigen::Vector2f(1.0f, 1.0f);
        b5.colorMode = float(box.colorMode);

        b6.position = Eigen::Vector3f(box.corners[3].x(), box.corners[3].y(), z_bottom);
        b6.texCoord = Eigen::Vector2f(0.0f, 1.0f);
        b6.colorMode = float(box.colorMode);

        box.vertices.push_back(b1);
        box.vertices.push_back(b2);
        box.vertices.push_back(b3);
        box.vertices.push_back(b4);
        box.vertices.push_back(b5);
        box.vertices.push_back(b6);
    }

    appendRoundedCornerStrips(box, /*segments=*/8);
}

void BoxRenderer::appendRoundedCornerStrips(Box& box, int segments) {
    // --- 1) Build local frame (ex, ey), center and half-sizes ---
    auto makeFrame = [&](Eigen::Vector2f& center, Eigen::Vector2f& ex, Eigen::Vector2f& ey,
                         Eigen::Vector2f& halfSize, float& radius) {
        const auto& c0 = box.corners[0];
        const auto& c1 = box.corners[1];
        const auto& c2 = box.corners[2];
        const auto& c3 = box.corners[3];

        center = 0.25f * (c0 + c1 + c2 + c3);

        Eigen::Vector2f e01 = c1 - c0;
        Eigen::Vector2f e12 = c2 - c1;
        Eigen::Vector2f e23 = c3 - c2;
        Eigen::Vector2f e30 = c0 - c3;

        float l01 = e01.norm(), l12 = e12.norm(), l23 = e23.norm(), l30 = e30.norm();

        // choose longest edge for x-axis, opposite edge helps scale
        int idx = 0; float lmax = l01;
        if (l12 > lmax) { lmax = l12; idx = 1; }
        if (l23 > lmax) { lmax = l23; idx = 2; }
        if (l30 > lmax) { lmax = l30; idx = 3; }

        Eigen::Vector2f x_edge, x_opp, y_edge, y_opp;
        float lx=0, lxo=0, ly=0, lyo=0;
        if (idx == 0) { x_edge=e01; x_opp=e23; lx=l01; lxo=l23; y_edge=e12; y_opp=e30; ly=l12; lyo=l30; }
        else if (idx == 1){ x_edge=e12; x_opp=e30; lx=l12; lxo=l30; y_edge=e23; y_opp=e01; ly=l23; lyo=l01; }
        else if (idx == 2){ x_edge=e23; x_opp=e01; lx=l23; lxo=l01; y_edge=e30; y_opp=e12; ly=l30; lyo=l12; }
        else { x_edge=e30; x_opp=e12; lx=l30; lxo=l12; y_edge=e01; y_opp=e23; ly=l01; lyo=l23; }

        ex = (lx > 1e-6f) ? x_edge / lx : Eigen::Vector2f(1,0);
        // Gram–Schmidt to make ey orthonormal to ex
        Eigen::Vector2f y_proj = y_edge - ex * (y_edge.dot(ex));
        float lyp = y_proj.norm();
        ey = (lyp > 1e-6f) ? y_proj / lyp : Eigen::Vector2f(-ex.y(), ex.x());

        halfSize = 0.5f * Eigen::Vector2f( (lx + lxo)*0.5f, (ly + lyo)*0.5f );

        // radius consistent with your rule, then clamped
        float avg_length = 0.0f;
        for (int i=0;i<4;++i) {
            int ni = (i+1) & 3;
            avg_length += (box.corners[ni] - box.corners[i]).norm();
        }
        avg_length *= 0.25f;
        radius = std::min(std::max(avg_length * 0.15f, 0.1f), 0.5f);
        radius = std::min(radius, std::min(halfSize.x(), halfSize.y())*0.999f);
    };

    Eigen::Vector2f center, ex, ey, half; float r;
    makeFrame(center, ex, ey, half, r);

    // --- 2) Corner centers (in world XY), using local offsets (±half.x - r, ±half.y - r) ---
    auto localToWorld = [&](float lx, float ly) -> Eigen::Vector2f {
        return center + lx * ex + ly * ey;
    };

    Eigen::Vector2f C[4] = {
        localToWorld(-half.x() + r, -half.y() + r), // c0: bottom-left in local frame
        localToWorld( half.x() - r, -half.y() + r), // c1: bottom-right
        localToWorld( half.x() - r,  half.y() - r), // c2: top-right
        localToWorld(-half.x() + r,  half.y() - r)  // c3: top-left
    };

    // --- 3) Angle spans per corner (outward, CCW in local space) ---
    const float a0[4] = { float(M_PI), 1.5f*float(M_PI), 0.0f, 0.5f*float(M_PI) };
    const float a1[4] = { 1.5f*float(M_PI), 2.0f*float(M_PI), 0.5f*float(M_PI), float(M_PI) };

    const float z0 = box.z_offset;
    const float z1 = box.z_offset + box.z_height;

    auto pushQuad = [&](const Eigen::Vector2f& p0, const Eigen::Vector2f& p1) {
        // vertical quad (p0,z0)-(p1,z0)-(p1,z1)-(p0,z1)
        BoxVertex v[6];
        auto V = [&](const Eigen::Vector2f& p, float z) {
            BoxVertex t; t.position = Eigen::Vector3f(p.x(), p.y(), z);
            t.texCoord = Eigen::Vector2f(0.0f, 0.0f);
            t.colorMode = float(box.colorMode);
            return t;
        };
        v[0] = V(p0, z0);
        v[1] = V(p1, z0);
        v[2] = V(p1, z1);
        v[3] = V(p0, z0);
        v[4] = V(p1, z1);
        v[5] = V(p0, z1);
        box.vertices.insert(box.vertices.end(), v, v+6);
    };

    // --- 4) Build quarter-cylinder per corner with 'segments' quads ---
    for (int c = 0; c < 4; ++c) {
        float aStart = a0[c];
        float aEnd   = a1[c];
        for (int s = 0; s < segments; ++s) {
            float t0 = float(s)   / segments;
            float t1 = float(s+1) / segments;
            float th0 = aStart + (aEnd - aStart) * t0;
            float th1 = aStart + (aEnd - aStart) * t1;

            Eigen::Vector2f p0 = C[c] + r * ( std::cos(th0) * ex + std::sin(th0) * ey );
            Eigen::Vector2f p1 = C[c] + r * ( std::cos(th1) * ex + std::sin(th1) * ey );

            pushQuad(p0, p1);
        }
    }
}




} // namespace glk

