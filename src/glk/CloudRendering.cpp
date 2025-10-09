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
    if (!loadPerlinTexture("perlin256.png")) {
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

void CloudRenderer::renderSky(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, 
                             const Eigen::Vector3f& cameraPosition) {
    if (cloud_shader_program_ == 0 || vertices_.empty()) return;
    
    glUseProgram(cloud_shader_program_);
    
    // Set uniforms
    glUniformMatrix4fv(glGetUniformLocation(cloud_shader_program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(cloud_shader_program_, "projection"), 1, GL_FALSE, projection.data());
    glUniform3fv(glGetUniformLocation(cloud_shader_program_, "cameraPosition"), 1, cameraPosition.data());
    
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uTime"), uniforms_.uTime);
    glUniform3fv(glGetUniformLocation(cloud_shader_program_, "uSunDirection"), 1, uniforms_.uSunDirection.data());
    glUniform2fv(glGetUniformLocation(cloud_shader_program_, "uResolution"), 1, uniforms_.uResolution.data());
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uCloudCoverage"), uniforms_.uCloudCoverage);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uCloudHeight"), uniforms_.uCloudHeight);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uCloudThickness"), uniforms_.uCloudThickness);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uCloudAbsorption"), uniforms_.uCloudAbsorption);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uWindSpeedX"), uniforms_.uWindSpeedX);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uWindSpeedZ"), uniforms_.uWindSpeedZ);
    glUniform1f(glGetUniformLocation(cloud_shader_program_, "uMaxCloudDistance"), uniforms_.uMaxCloudDistance);
    
    // Bind Perlin noise texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, perlin_texture_);
    glUniform1i(glGetUniformLocation(cloud_shader_program_, "t_PerlinNoise"), 0);
    
    // Enable depth testing but render back faces
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glCullFace(GL_FRONT); // Render inside of sphere
    
    glBindVertexArray(cloud_vao_);
    glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    
    // Restore culling
    glCullFace(GL_BACK);
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
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec2 uv;
        layout (location = 2) in vec3 normal;
        
        out vec3 vWorldPosition;
        out vec2 vUv;
        
        uniform mat4 view;
        uniform mat4 projection;
        
        void main() {
            vUv = uv;
            vec4 worldPosition = vec4(position, 1.0);
            vWorldPosition = worldPosition.xyz;
            gl_Position = projection * view * worldPosition;
        }
    )";
    
    // Fragment shader - converted from the Three.js version
    const char* fragment_shader = R"(
        #version 460 core
        precision highp float;
        precision highp int;
        
        uniform float uTime;
        uniform vec3 uSunDirection;
        uniform sampler2D t_PerlinNoise;
        uniform vec2 uResolution;
        uniform float uCloudCoverage;
        uniform float uCloudHeight;
        uniform float uCloudThickness;
        uniform float uCloudAbsorption;
        uniform float uWindSpeedX;
        uniform float uWindSpeedZ;
        uniform float uMaxCloudDistance;
        uniform vec3 cameraPosition;
        
        in vec3 vWorldPosition;
        in vec2 vUv;
        
        out vec4 FragColor;
        
        #define TWO_PI 6.28318530718
        #define STEPS          22
        #define LIGHT_STEPS    5
        
        vec3 Get_Sky_Color(vec3 rayDir) {
            float sunAmount = max(0.0, dot(rayDir, uSunDirection));
            float skyGradient = pow(max(0.0, rayDir.y), 0.5);
            vec3 skyColor = mix(
                vec3(0.1, 0.2, 0.4),
                vec3(0.8, 0.7, 0.5),
                pow(sunAmount, 12.0)
            );
            vec3 horizonColor = vec3(0.7, 0.75, 0.8);
            skyColor = mix(horizonColor, skyColor, skyGradient);
            if (sunAmount > 0.999) {
                skyColor += vec3(1.0, 0.7, 0.3) * pow(sunAmount, 1000.0) * 3.0;
            }
            return skyColor;
        }
        
        float noise3D(in vec3 p) {
            vec2 uv = p.xz * 0.01;
            return texture(t_PerlinNoise, uv).x;
        }
        
        const mat3 m = 1.21 * mat3(0.00, 0.80, 0.60,
                                  -0.80, 0.36, -0.48,
                                  -0.60, -0.48, 0.64);
        
        float fbm(vec3 p) {
            float t;
            float mult = 2.76434;
            t  = 0.51749673 * noise3D(p); p = m * p * mult;
            t += 0.25584929 * noise3D(p); p = m * p * mult;
            t += 0.12527603 * noise3D(p); p = m * p * mult;
            t += 0.06255931 * noise3D(p);
            return t;
        }
        
        float cloud_density(vec3 pos, vec3 offset, float h) {
            vec3 p = pos * 0.0212242 + offset;
            float dens = fbm(p);
            float cov = 1.0 - uCloudCoverage;
            dens *= smoothstep(cov, cov + 0.05, dens);
            float height = pos.y - uCloudHeight;
            float heightAttenuation = 1.0 - clamp(height / uCloudThickness, 0.0, 1.0);
            heightAttenuation = heightAttenuation * heightAttenuation;
            dens *= heightAttenuation;
            return clamp(dens, 0.0, 1.0);
        }
        
        float cloud_light(vec3 pos, vec3 dir_step, vec3 offset, float cov) {
            float T = 1.0;
            for (int i = 0; i < LIGHT_STEPS; i++) {
                float dens = cloud_density(pos, offset, 0.0);
                float T_i = exp(-uCloudAbsorption * dens);
                T *= T_i;
                pos += dir_step;
            }
            return T;
        }
        
        vec4 render_clouds(vec3 rayOrigin, vec3 rayDirection) {
            float t = (uCloudHeight - rayOrigin.y) / rayDirection.y;
            if (t < 0.0) return vec4(0.0);
            if (t > uMaxCloudDistance) return vec4(0.0);
            float distanceFade = 1.0 - smoothstep(uMaxCloudDistance * 0.6, uMaxCloudDistance, t);
            vec3 startPos = rayOrigin + rayDirection * t;
            vec3 windOffset = vec3(uTime * -uWindSpeedX, 0.0, uTime * -uWindSpeedZ);
            vec3 pos = startPos;
            float march_step = uCloudThickness / float(STEPS);
            vec3 dir_step = rayDirection * march_step;
            vec3 light_step = uSunDirection * 5.0;
            float covAmount = (sin(mod(uTime * 0.02, TWO_PI))) * 0.1 + 0.5;
            float coverage = mix(0.4, 0.6, clamp(covAmount, 0.0, 1.0));
            float T = 1.0;
            vec3 C = vec3(0);
            float alpha = 0.0;
            for (int i = 0; i < STEPS; i++) {
                if (pos.y < uCloudHeight || pos.y > uCloudHeight + uCloudThickness) {
                    pos += dir_step;
                    continue;
                }
                float h = float(i) / float(STEPS);
                float dens = cloud_density(pos, windOffset, h);
                if (dens > 0.01) {
                    float T_i = exp(-uCloudAbsorption * dens * march_step);
                    T *= T_i;
                    float cloudLight = cloud_light(pos, light_step, windOffset, coverage);
                    float lightFactor = (exp(h) / 1.75);
                    float sunContribution = pow(max(0.0, dot(rayDirection, uSunDirection)), 2.0);
                    vec3 edgeColor = mix(vec3(1.0), vec3(1.0, 0.8, 0.5), sunContribution);
                    vec3 cloudColor = mix(
                        vec3(0.15, 0.15, 0.2),
                        edgeColor,
                        cloudLight * lightFactor
                    );
                    C += T * cloudColor * dens * march_step * 1.5;
                    alpha += (1.0 - T_i) * (1.0 - alpha);
                }
                pos += dir_step;
                if (T < 0.01) break;
            }
            vec3 sunColor = vec3(0.9, 0.7, 0.5);
            vec3 skyColor = vec3(0.4, 0.5, 0.6);
            C = C * mix(skyColor, sunColor, 0.5 * pow(max(0.0, dot(rayDirection, uSunDirection)), 2.0));
            alpha *= distanceFade;
            C *= distanceFade;
            return vec4(C, alpha);
        }
        
        void main() {
            vec3 rayDirection = normalize(vWorldPosition - cameraPosition);
            vec3 skyColor = Get_Sky_Color(rayDirection);
            vec4 clouds = vec4(0.0);
            if (rayDirection.y > 0.0) {
                clouds = render_clouds(cameraPosition, rayDirection);
            }
            vec3 finalColor = mix(skyColor, clouds.rgb, clouds.a);
            float t = pow(1.0 - max(0.0, rayDirection.y), 5.0);
            finalColor = mix(finalColor, vec3(0.65, 0.7, 0.75), 0.5 * t);
            finalColor = finalColor / (finalColor + vec3(1.0));
            FragColor = vec4(finalColor, 1.0);
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

} // namespace glk
