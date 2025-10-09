#include <glk/CloudRendering.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <stb_image.h>

namespace glk {

CloudRenderer::CloudRenderer() 
    : cloud_vao_(0), cloud_vbo_(0), cloud_ebo_(0), cloud_shader_program_(0), perlin_texture_(0) {
}

CloudRenderer::~CloudRenderer() {
    cleanupCloudRendering();
}

bool CloudRenderer::initCloudRendering() {
    // Create VAO, VBO, and EBO
    glGenVertexArrays(1, &cloud_vao_);
    glGenBuffers(1, &cloud_vbo_);
    glGenBuffers(1, &cloud_ebo_);
    
    // Load or create Perlin noise texture
    if (!loadPerlinTexture("perlin257.png")) {
        std::cout << "Failed to load Perlin texture, creating default..." << std::endl;
        perlin_texture_ = createDefaultPerlinTexture();
    }
    
    // Create shaders
    if (!createCloudShaders()) {
        std::cerr << "Failed to create cloud shaders" << std::endl;
        return false;
    }
    
    // Create default sky sphere
    createSkySphere();
    
    return true;
}

bool CloudRenderer::loadPerlinTexture(const std::string& texturePath) {
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        std::cerr << "Failed to load Perlin texture: " << texturePath << std::endl;
        return false;
    }
    
    glGenTextures(1, &perlin_texture_);
    glBindTexture(GL_TEXTURE_2D, perlin_texture_);
    
    // Set texture parameters for seamless tiling
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload texture data
    GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    
    stbi_image_free(data);
    
    std::cout << "Loaded Perlin texture: " << texturePath << " (" << width << "x" << height << ")" << std::endl;
    return true;
}

GLuint CloudRenderer::createDefaultPerlinTexture() {
    const int size = 256;
    std::vector<unsigned char> data(size * size * 4);
    
    // Generate simple noise pattern
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            float noise = (sin(x * 0.1f) * cos(y * 0.1f) + 1.0f) * 0.5f;
            unsigned char value = static_cast<unsigned char>(noise * 255);
            
            int index = (y * size + x) * 4;
            data[index] = value;
            data[index + 1] = value;
            data[index + 2] = value;
            data[index + 3] = 255;
        }
    }
    
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    
    std::cout << "Created default Perlin texture (256x256)" << std::endl;
    return texture;
}

void CloudRenderer::createSkySphere(float radius, int widthSegments, int heightSegments) {
    vertices_.clear();
    indices_.clear();
    
    generateSphereGeometry(radius, widthSegments, heightSegments);
    
    // Upload to GPU
    glBindVertexArray(cloud_vao_);
    
    glBindBuffer(GL_ARRAY_BUFFER, cloud_vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(CloudVertex), 
                 vertices_.data(), GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cloud_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), 
                 indices_.data(), GL_STATIC_DRAW);
    
    // Set up vertex attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CloudVertex), 
                         (void*)offsetof(CloudVertex, position));
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(CloudVertex), 
                         (void*)offsetof(CloudVertex, uv));
    
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(CloudVertex), 
                         (void*)offsetof(CloudVertex, normal));
    
    glBindVertexArray(0);
    
    std::cout << "Created sky sphere with " << vertices_.size() << " vertices and " 
              << indices_.size() << " indices" << std::endl;
}

void CloudRenderer::generateSphereGeometry(float radius, int widthSegments, int heightSegments) {
    const float PI = 3.14159265359f;
    
    for (int y = 0; y <= heightSegments; ++y) {
        for (int x = 0; x <= widthSegments; ++x) {
            float u = static_cast<float>(x) / widthSegments;
            float v = static_cast<float>(y) / heightSegments;
            
            float theta = u * 2.0f * PI;
            float phi = v * PI;
            
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);
            
            CloudVertex vertex;
            vertex.position = Eigen::Vector3f(
                radius * sinPhi * cosTheta,
                radius * cosPhi,
                radius * sinPhi * sinTheta
            );
            vertex.uv = Eigen::Vector2f(u, v);
            vertex.normal = vertex.position.normalized();
            
            vertices_.push_back(vertex);
        }
    }
    
    // Generate indices
    for (int y = 0; y < heightSegments; ++y) {
        for (int x = 0; x < widthSegments; ++x) {
            int a = y * (widthSegments + 1) + x;
            int b = a + 1;
            int c = a + widthSegments + 1;
            int d = c + 1;
            
            // Two triangles per quad
            indices_.push_back(a);
            indices_.push_back(c);
            indices_.push_back(b);
            
            indices_.push_back(b);
            indices_.push_back(c);
            indices_.push_back(d);
        }
    }
}

void CloudRenderer::updateTime(float time) {
    uniforms_.uTime = time;
}

void CloudRenderer::updateSunDirection(const Eigen::Vector3f& sunDirection) {
    uniforms_.uSunDirection = sunDirection.normalized();
}

void CloudRenderer::updateCloudParameters(float coverage, float height, float thickness, 
                                         float absorption, float windX, float windZ, float maxDistance) {
    uniforms_.uCloudCoverage = coverage;
    uniforms_.uCloudHeight = height;
    uniforms_.uCloudThickness = thickness;
    uniforms_.uCloudAbsorption = absorption;
    uniforms_.uWindSpeedX = windX;
    uniforms_.uWindSpeedZ = windZ;
    uniforms_.uMaxCloudDistance = maxDistance;
}


void CloudRenderer::cleanupCloudRendering() {
    if (cloud_vao_ != 0) {
        glDeleteVertexArrays(1, &cloud_vao_);
        cloud_vao_ = 0;
    }
    if (cloud_vbo_ != 0) {
        glDeleteBuffers(1, &cloud_vbo_);
        cloud_vbo_ = 0;
    }
    if (cloud_ebo_ != 0) {
        glDeleteBuffers(1, &cloud_ebo_);
        cloud_ebo_ = 0;
    }
    if (cloud_shader_program_ != 0) {
        glDeleteProgram(cloud_shader_program_);
        cloud_shader_program_ = 0;
    }
    if (perlin_texture_ != 0) {
        glDeleteTextures(1, &perlin_texture_);
        perlin_texture_ = 0;
    }
}

bool CloudRenderer::createCloudShaders() {
    // Vertex shader
    const char* vertex_shader = R"(
        #version 460 core
        layout(location=0) in vec3 position;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        out vec3 vPosWS;   // world-space position of the vertex
        out vec3 vPosLS;   // local-space (box) position

        void main() {
            vec4 world = model * vec4(position, 1.0);
            vPosWS = world.xyz;
            vPosLS = position;
            gl_Position = projection * view * world;
        }
    )";
    
    // Fragment shader - converted from the Three.js version
    // replace your fragment_shader string with this one:
    const char* fragment_shader = R"(
        #version 460 core

        in vec3 vPosWS;   // world-space position of the vertex (front face)
        in vec3 vPosLS;   // local-space position inside the box
        out vec4 FragColor;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        // Params
        uniform float uTime;
        uniform vec3  uSunDirection;
        uniform sampler2D t_PerlinNoise;
        uniform vec2  uResolution;          // not used
        uniform float uCloudCoverage;
        uniform float uCloudHeight;
        uniform float uCloudThickness;
        uniform float uCloudAbsorption;
        uniform float uWindSpeedX;
        uniform float uWindSpeedZ;
        uniform float uMaxCloudDistance;    // not used
        uniform vec3  cameraPosition;

        uniform vec3 uHalfExtents;

        // 0 = Y-up, 1 = Z-up (your scene)
        uniform int  uZUp;

        #define STEPS          22
        #define LIGHT_STEPS     5

        // Choose height axis and ground plane depending on up-axis
        float getHeightCoord(vec3 p) { return (uZUp == 1) ? p.z : p.y; }
        vec2  groundCoord (vec3 p)  { return (uZUp == 1) ? p.xy : p.xz; }

        float noise3D(in vec3 p) {
            // 2D perlin lookup over ground plane
            vec2 uv = groundCoord(p) * 0.01;
            return texture(t_PerlinNoise, uv).x;
        }

        const mat3 m = 1.21 * mat3( 0.00,  0.80,  0.60,
                                -0.80,  0.36, -0.48,
                                -0.60, -0.48,  0.64);

        float fbm(vec3 p) {
            float t;
            float mult = 2.76434;
            t  = 0.51749673 * noise3D(p); p = m * p * mult;
            t += 0.25584929 * noise3D(p); p = m * p * mult;
            t += 0.12527603 * noise3D(p); p = m * p * mult;
            t += 0.06255931 * noise3D(p);
            return t;
        }

        float cloud_density(vec3 posWS, vec3 windOffset, float h01) {
            vec3 p = posWS * 0.0212242 + windOffset;

            float dens = fbm(p);

            // Coverage gate
            float cov = 1.0 - uCloudCoverage;
            dens *= smoothstep(cov, cov + 0.05, dens);

            // Vertical band along the scene's up axis
            float height    = getHeightCoord(posWS) - uCloudHeight;
            float heightAtt = 1.0 - clamp(height / uCloudThickness, 0.0, 1.0);
            heightAtt *= heightAtt;

            dens *= heightAtt;
            return clamp(dens, 0.0, 1.0);
        }

        // Keep 4-arg signature to match your C++ call
        float cloud_light(vec3 posWS, vec3 dir_stepWS, vec3 windOffset, float cov) {
            // “Use” cov to avoid unused warnings (no effect on result)
            float covNoOp = cov * 0.0;

            float T = 1.0 + covNoOp;
            vec3 pos = posWS;
            for (int i = 0; i < LIGHT_STEPS; ++i) {
                float dens = cloud_density(pos, windOffset, 0.0);
                float T_i = exp(-uCloudAbsorption * dens);
                T *= T_i;
                pos += dir_stepWS;
                if (T < 0.01) break;
            }
            return T; // transmittance (0 = dark, 1 = full light)
        }

        // AABB ray-box (LOCAL space)
        bool intersectBoxLS(vec3 ro, vec3 rd, vec3 bmin, vec3 bmax, out float t0, out float t1) {
            vec3 inv = 1.0 / rd;
            vec3 tbot = (bmin - ro) * inv;
            vec3 ttop = (bmax - ro) * inv;
            vec3 tmin = min(ttop, tbot);
            vec3 tmax = max(ttop, tbot);
            t0 = max(max(tmin.x, tmin.y), tmin.z);
            t1 = min(min(tmax.x, tmax.y), tmax.z);
            return t1 > max(t0, 0.0);
        }

        void main() {
            // Camera in LOCAL space
            mat4 invModel = inverse(model);
            vec3 camLS = (invModel * vec4(cameraPosition, 1.0)).xyz;

            // Ray toward this fragment (LOCAL space)
            vec3 dirLS = normalize(vPosLS - camLS);

            // AABB intersection (LOCAL)
            vec3 bmin = -uHalfExtents;
            vec3 bmax =  uHalfExtents;
            float tEnter, tExit;
            if (!intersectBoxLS(camLS, dirLS, bmin, bmax, tEnter, tExit)) {
                discard;
            }
            tEnter = max(tEnter, 0.0);

            // March
            float marchLen = (tExit - tEnter) / float(STEPS);
            vec3  posLS = camLS + dirLS * tEnter;

            // Convert LS step to WORLD distance (Beer-Lambert)
            vec3 stepWSvec = (model * vec4(dirLS * marchLen, 0.0)).xyz;
            float stepWorldDist = length(stepWSvec);

            vec3 posWS = (model * vec4(posLS, 1.0)).xyz;

            // Wind on ground plane (XY if Z-up, XZ if Y-up)
            vec3 windOffset = (uZUp == 1)
                ? vec3(uTime * -uWindSpeedX, uTime * -uWindSpeedZ, 0.0)
                : vec3(uTime * -uWindSpeedX, 0.0,                uTime * -uWindSpeedZ);

            vec3 C = vec3(0.0);
            float T = 1.0;

            for (int i = 0; i < STEPS; ++i) {
                float dens = cloud_density(posWS, windOffset, float(i) / float(STEPS));
                if (dens > 0.01) {
                    float T_i = exp(-uCloudAbsorption * dens * stepWorldDist);
                    float lightTrans = cloud_light(posWS, normalize(uSunDirection) * 5.0, windOffset, uCloudCoverage);

                    vec3 base = vec3(0.15, 0.15, 0.2);
                    vec3 sunC = vec3(1.0, 0.9, 0.7);
                    vec3 col = mix(base, sunC, lightTrans);

                    C += T * col * dens * stepWorldDist;
                    T *= T_i;
                    if (T < 0.01) break;
                }
                posLS += dirLS * marchLen;
                posWS = (model * vec4(posLS, 1.0)).xyz;
            }

            float alpha = clamp(1.0 - T, 0.0, 1.0);
            FragColor = vec4(C, alpha);
        }
    )";


    
    cloud_shader_program_ = compileShaderProgram(vertex_shader, fragment_shader);
    return cloud_shader_program_ != 0;
}

GLuint CloudRenderer::compileShaderProgram(const char* vertex_source, const char* fragment_source) {
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_source, NULL);
    glCompileShader(vertex_shader);
    
    GLint success;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "Cloud vertex shader compilation failed: " << info_log << std::endl;
        return 0;
    }
    
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_source, NULL);
    glCompileShader(fragment_shader);
    
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "Cloud fragment shader compilation failed: " << info_log << std::endl;
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
        std::cerr << "Cloud shader program linking failed: " << info_log << std::endl;
        return 0;
    }
    
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    
    return program;
}

void CloudRenderer::renderCloudVolume(const Eigen::Matrix4f& view,
                                      const Eigen::Matrix4f& projection,
                                      const Eigen::Vector3f& cameraPosition) {
    if (!vol_vao_ || cloud_shader_program_ == 0) return;

    // Save state
    GLint prevProgram=0; glGetIntegerv(GL_CURRENT_PROGRAM, &prevProgram);
    GLboolean wasBlend = glIsEnabled(GL_BLEND);
    GLboolean wasDepth = glIsEnabled(GL_DEPTH_TEST);
    GLint prevDepthFunc=0; glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);
    GLboolean prevDepthMask; glGetBooleanv(GL_DEPTH_WRITEMASK, &prevDepthMask);
    GLboolean wasCull = glIsEnabled(GL_CULL_FACE);

    // Use a dedicated volume program (compile like cloud_shader_program_, or reuse it with extra uniforms)
    GLuint prog = cloud_shader_program_; // if you merged the volume FS into this program
    glUseProgram(prog);

    // Uniforms (reuse your locations or query once & cache)
    glUniformMatrix4fv(glGetUniformLocation(prog,"model"), 1, GL_FALSE, volume_model_.data());
    glUniformMatrix4fv(glGetUniformLocation(prog,"view"),  1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(prog,"projection"), 1, GL_FALSE, projection.data());
    glUniform3fv(glGetUniformLocation(prog,"cameraPosition"), 1, cameraPosition.data());
    glUniform3fv(glGetUniformLocation(prog,"uHalfExtents"), 1, half_extents_.data());

    // cloud params (same as in your sky code)
    glUniform1f(glGetUniformLocation(prog,"uTime"), uniforms_.uTime);
    glUniform3fv(glGetUniformLocation(prog,"uSunDirection"), 1, uniforms_.uSunDirection.data());
    glUniform1f(glGetUniformLocation(prog,"uCloudCoverage"), uniforms_.uCloudCoverage);
    glUniform1f(glGetUniformLocation(prog,"uCloudHeight"),   uniforms_.uCloudHeight);
    glUniform1f(glGetUniformLocation(prog,"uCloudThickness"),uniforms_.uCloudThickness);
    glUniform1f(glGetUniformLocation(prog,"uCloudAbsorption"),uniforms_.uCloudAbsorption);
    glUniform1f(glGetUniformLocation(prog,"uWindSpeedX"),    uniforms_.uWindSpeedX);
    glUniform1f(glGetUniformLocation(prog,"uWindSpeedZ"),    uniforms_.uWindSpeedZ);
    glUniform1f(glGetUniformLocation(prog,"uMaxCloudDistance"),uniforms_.uMaxCloudDistance);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, perlin_texture_);
    glUniform1i(glGetUniformLocation(prog, "uZUp"), 1); // Z-up

    // Blend clouds over opaque scene
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);

    glDisable(GL_CULL_FACE);

    glBindVertexArray(vol_vao_);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    // Restore
    if (!wasBlend) glDisable(GL_BLEND);
    if (wasCull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (wasDepth) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    glDepthFunc(prevDepthFunc ? prevDepthFunc : GL_LESS);
    glDepthMask(prevDepthMask);
    glUseProgram(prevProgram);
}

void CloudRenderer::createVolumeBox(const Eigen::Vector3f& halfExtents) {
    volume_half_extents_ = halfExtents;

    // 24 unique verts (4 per face) so it can have correct face interpolation
    struct V { float x,y,z; };
    const float hx = halfExtents.x(), hy = halfExtents.y(), hz = halfExtents.z();

    // positions for 6 faces (CCW, triangles)
    const V verts[] = {
        // +X face
        {+hx,-hy,-hz}, {+hx,+hy,-hz}, {+hx,+hy,+hz}, {+hx,-hy,+hz},
        // -X face
        {-hx,-hy,+hz}, {-hx,+hy,+hz}, {-hx,+hy,-hz}, {-hx,-hy,-hz},
        // +Y face
        {-hx,+hy,-hz}, {-hx,+hy,+hz}, {+hx,+hy,+hz}, {+hx,+hy,-hz},
        // -Y face
        {-hx,-hy,+hz}, {-hx,-hy,-hz}, {+hx,-hy,-hz}, {+hx,-hy,+hz},
        // +Z face
        {-hx,-hy,+hz}, {+hx,-hy,+hz}, {+hx,+hy,+hz}, {-hx,+hy,+hz},
        // -Z face
        {+hx,-hy,-hz}, {-hx,-hy,-hz}, {-hx,+hy,-hz}, {+hx,+hy,-hz},
    };

    const GLuint idx[] = {
        0,1,2, 0,2,3,      // +X
        4,5,6, 4,6,7,      // -X
        8,9,10, 8,10,11,   // +Y
        12,13,14, 12,14,15,// -Y
        16,17,18, 16,18,19,// +Z
        20,21,22, 20,22,23 // -Z
    };

    if (vol_vao_ == 0) glGenVertexArrays(1, &vol_vao_);
    if (vol_vbo_ == 0) glGenBuffers(1, &vol_vbo_);
    if (vol_ebo_ == 0) glGenBuffers(1, &vol_ebo_);

    glBindVertexArray(vol_vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vol_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vol_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0); // position -> location 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(V), (void*)0);

    glBindVertexArray(0);

    std::cout << "Created cloud volume box VAO (half-extents: "
              << hx << "," << hy << "," << hz << "), 36 indices\n";
}



} // namespace glk
