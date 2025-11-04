#include <glk/modelUpload.hpp>
#include <iostream>

// GLB support libraries
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

// Image loading library
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

modelUpload::modelUpload(/* args */)
{
}

modelUpload::~modelUpload()
{
}

void modelUpload::cleanupMesh(PlyMesh &mesh)
{
    std::cout << "Cleaning up mesh - VAO: " << mesh.vao
              << ", VBO: " << mesh.vbo
              << ", EBO: " << mesh.ebo
              << ", Texture: " << mesh.texture_id << std::endl;

    // Clear any existing errors first
    while (glGetError() != GL_NO_ERROR)
    { /* Clear error queue */
    }

    // Delete VAO
    if (mesh.vao != 0)
    {
        glDeleteVertexArrays(1, &mesh.vao);
        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            std::cout << "Error deleting VAO " << mesh.vao << ": " << error << std::endl;
        }
        mesh.vao = 0;
    }

    // Delete VBO
    if (mesh.vbo != 0)
    {
        glDeleteBuffers(1, &mesh.vbo);
        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            std::cout << "Error deleting VBO " << mesh.vbo << ": " << error << std::endl;
        }
        mesh.vbo = 0;
    }

    // Delete EBO
    if (mesh.ebo != 0)
    {
        glDeleteBuffers(1, &mesh.ebo);
        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            std::cout << "Error deleting EBO " << mesh.ebo << ": " << error << std::endl;
        }
        mesh.ebo = 0;
    }

    // Delete texture
    if (mesh.texture_id != 0)
    {
        GLboolean is_texture = glIsTexture(mesh.texture_id);
        if (is_texture)
        {
            glDeleteTextures(1, &mesh.texture_id);
            GLenum error = glGetError();
            if (error != GL_NO_ERROR)
            {
                std::cout << "Error deleting texture " << mesh.texture_id << ": " << error << std::endl;
            }
            else
            {
                std::cout << "Deleted texture ID: " << mesh.texture_id << std::endl;
            }
        }
        else
        {
            std::cout << "Warning: Texture ID " << mesh.texture_id << " is not valid" << std::endl;
        }
        mesh.texture_id = 0;
    }

    // Reset other properties...
    mesh.vertex_count = 0;
    mesh.index_count = 0;
    mesh.has_texture = false;
    mesh.base_color[0] = mesh.base_color[1] = mesh.base_color[2] = mesh.base_color[3] = 1.0f;
    mesh.metallic_factor = 0.0f;
    mesh.roughness_factor = 1.0f;
    mesh.frame_id = "map";
    mesh.type = -1;

    std::cout << "Mesh cleanup completed" << std::endl;
}

bool modelUpload::createGLBShader(GLuint &shader_program)
{
    const char *vertex_shader_source = R"(
        #version 330 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec3 normal;
        layout (location = 2) in vec2 tex_coord;
        
        uniform mat4 view_matrix;
        uniform mat4 projection_matrix;
        uniform mat4 model_matrix;
        
        out vec3 frag_pos;
        out vec3 frag_normal;
        out vec2 frag_tex_coord;
        
        void main() {
            vec4 world_pos = model_matrix * vec4(position, 1.0);
            frag_pos = world_pos.xyz;
            
            // Transform normal to world space
            mat3 normal_matrix = mat3(transpose(inverse(model_matrix)));
            frag_normal = normalize(normal_matrix * normal);
            
            frag_tex_coord = tex_coord;
            gl_Position = projection_matrix * view_matrix * world_pos;
        }
    )";

    const char *fragment_shader_source = R"(
        #version 330 core
        in vec3 frag_pos;
        in vec3 frag_normal;
        in vec2 frag_tex_coord;
        
        uniform vec4 base_color;
        uniform bool has_texture;
        uniform sampler2D base_color_texture;
        uniform float metallic_factor;
        uniform float roughness_factor;
        
        out vec4 FragColor;
        
        void main() {
            // Start with base color
            vec4 albedo = base_color;
            
            // Sample texture if available
            if (has_texture) {
                vec4 tex_color = texture(base_color_texture, frag_tex_coord);
                albedo = albedo * tex_color;
            }
            
            // Calculate lighting
            vec3 normal = normalize(frag_normal);
            
            // Main light source
            vec3 light_dir = normalize(vec3(0.5, 1.0, 0.8));
            vec3 light_color = vec3(1.0, 1.0, 1.0);
            
            // Ambient lighting
            float ambient_strength = 0.3;
            vec3 ambient = ambient_strength * light_color;
            
            // Diffuse lighting
            float diff = max(dot(normal, light_dir), 0.0);
            vec3 diffuse = diff * light_color * 0.6;
            
            // Simple specular (reduced for more realistic look)
            float specular_strength = 0.2 * (1.0 - roughness_factor);
            vec3 view_dir = normalize(-frag_pos);
            vec3 reflect_dir = reflect(-light_dir, normal);
            float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
            vec3 specular = specular_strength * spec * light_color;
            
            // Combine lighting with albedo
            vec3 result = (ambient + diffuse + specular) * albedo.rgb;
            
            FragColor = vec4(result, albedo.a);
        }
    )";

    // ... rest of your shader compilation code remains the same ...

    // Compile vertex shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_source, NULL);
    glCompileShader(vertex_shader);

    // Check vertex shader compilation
    GLint success;
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cerr << "GLB Vertex shader compilation failed: " << info_log << std::endl;
        return false;
    }

    // Compile fragment shader
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_source, NULL);
    glCompileShader(fragment_shader);

    // Check fragment shader compilation
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cerr << "GLB Fragment shader compilation failed: " << info_log << std::endl;
        glDeleteShader(vertex_shader);
        return false;
    }

    // Create and link program
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader);
    glAttachShader(shader_program, fragment_shader);
    glLinkProgram(shader_program);

    // Check linking
    glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetProgramInfoLog(shader_program, 512, NULL, info_log);
        std::cerr << "GLB Shader program linking failed: " << info_log << std::endl;
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        return false;
    }

    // Clean up individual shaders
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    std::cout << "Successfully created GLB shader program: " << shader_program << std::endl;
    return true;
}

// Helper function to validate GLB file format
bool modelUpload::validateGlbFile(const std::string &path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    // Check file size
    file.seekg(0, std::ios::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file_size < 12)
    { // GLB header is at least 12 bytes
        return false;
    }

    // Check magic number (first 4 bytes should be "glTF")
    char magic[4];
    file.read(magic, 4);

    return std::string(magic, 4) == "glTF";
}

// Helper function to get detailed cgltf error message
std::string modelUpload::getCgltfErrorMessage(int result)
{
    switch (result)
    {
    case cgltf_result_success:
        return "Success";
    case cgltf_result_data_too_short:
        return "Data too short";
    case cgltf_result_unknown_format:
        return "Unknown format";
    case cgltf_result_invalid_json:
        return "Invalid JSON";
    case cgltf_result_invalid_gltf:
        return "Invalid glTF structure";
    case cgltf_result_invalid_options:
        return "Invalid options";
    case cgltf_result_file_not_found:
        return "File not found";
    case cgltf_result_io_error:
        return "IO error";
    case cgltf_result_out_of_memory:
        return "Out of memory";
    default:
        return "Unknown error (code: " + std::to_string(result) + ")";
    }
}

PlyMesh modelUpload::loadModel(const std::string &path)
{
    // determine file extension
    auto pos = path.find_last_of('.') + 1;
    std::string ext = (pos != std::string::npos) ? path.substr(pos) : "";
    for (auto &c : ext)
        c = ::tolower(c);

    if (ext == "glb" || ext == "gltf")
    {
        return loadGlb(path);
    }
    else if (ext == "ply")
    {
        return loadPlyBinaryLE(path);
    }
    else
    {
        throw std::runtime_error("Unsupported model format: " + path);
    }
}

GLuint modelUpload::loadTextureFromImage(const cgltf_image *image, const cgltf_data * /* gltf_data */)
{
    if (!image)
        return 0;

    GLuint texture_id = 0;
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    // Load image data
    unsigned char *image_data = nullptr;
    int width = 0, height = 0, channels = 0;

    if (image->buffer_view)
    {
        // Image is embedded in GLB
        const cgltf_buffer_view *view = image->buffer_view;
        const cgltf_buffer *buffer = view->buffer;

        if (buffer->data)
        {
            const unsigned char *buffer_data = (const unsigned char *)buffer->data + view->offset;
            image_data = stbi_load_from_memory(buffer_data, (int)view->size, &width, &height, &channels, 4);
        }
    }
    else if (image->uri)
    {
        // External image file (less common in GLB)
        image_data = stbi_load(image->uri, &width, &height, &channels, 4);
    }

    if (image_data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
        glGenerateMipmap(GL_TEXTURE_2D);

        // Set texture parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(image_data);
        std::cout << "Loaded texture: " << width << "x" << height << " channels: " << channels << std::endl;
    }
    else
    {
        std::cerr << "Failed to load texture image" << std::endl;
        glDeleteTextures(1, &texture_id);
        texture_id = 0;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    return texture_id;
}

// ====================================
// bgl mesh loading & rendering
// ====================================

PlyMesh modelUpload::loadGlb(const std::string &path)
{
    std::cout << "Attempting to load GLB file: " << path << std::endl;

    // ✅ Clear any existing OpenGL errors at the start
    while (glGetError() != GL_NO_ERROR)
    {
        std::cout << "Clearing pre-existing OpenGL error" << std::endl;
    }

    // Check if file exists using filesystem
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("File does not exist: " + path);
    }

    // Check file size
    std::error_code ec;
    auto file_size = std::filesystem::file_size(path, ec);
    if (ec)
    {
        throw std::runtime_error("Cannot get file size: " + path + " (" + ec.message() + ")");
    }

    if (file_size == 0)
    {
        throw std::runtime_error("File is empty: " + path);
    }

    std::cout << "File exists, size: " << file_size << " bytes" << std::endl;

    // Validate GLB format
    if (!validateGlbFile(path))
    {
        throw std::runtime_error("File is not a valid GLB format: " + path);
    }

    std::cout << "File appears to be valid GLB format" << std::endl;

    // Initialize cgltf options - simplified version
    cgltf_options options = {};
    options.type = cgltf_file_type_invalid; // Auto-detect
    options.json_token_count = 0;           // Use default

    cgltf_data *data = nullptr;

    // Parse the file
    cgltf_result result = cgltf_parse_file(&options, path.c_str(), &data);
    if (result != cgltf_result_success)
    {
        std::string error_msg = getCgltfErrorMessage(result);
        throw std::runtime_error("Failed to parse glTF (" + error_msg + "): " + path);
    }

    std::cout << "Successfully parsed glTF file" << std::endl;

    // Load buffers
    result = cgltf_load_buffers(&options, data, path.c_str());
    if (result != cgltf_result_success)
    {
        std::string error_msg = getCgltfErrorMessage(result);
        cgltf_free(data);
        throw std::runtime_error("Failed to load glTF buffers (" + error_msg + "): " + path);
    }

    // Validate the data
    result = cgltf_validate(data);
    if (result != cgltf_result_success)
    {
        std::string error_msg = getCgltfErrorMessage(result);
        cgltf_free(data);
        throw std::runtime_error("Invalid glTF data (" + error_msg + "): " + path);
    }

    std::cout << "glTF validation successful" << std::endl;

    // Check for meshes
    if (data->meshes_count == 0)
    {
        cgltf_free(data);
        throw std::runtime_error("No meshes in glTF: " + path);
    }

    std::cout << "Found " << data->meshes_count << " mesh(es)" << std::endl;

    // Extract transform from first root node (contains object's transform from Blender)
    // This is stored in mesh.local_transform and applied during rendering
    Eigen::Matrix4f object_transform = Eigen::Matrix4f::Identity();
    
    if (data->scenes_count > 0)
    {
        const cgltf_scene *scene = &data->scenes[0];
        if (scene->nodes_count > 0)
        {
            // Extract ONLY the first root node's transform (the object's transform)
            // Don't multiply multiple root nodes together (that was the bug for multi-object scenes)
            const cgltf_node *first_node = scene->nodes[0];
            object_transform = extractNodeTransform(first_node);
            std::cout << "Extracted object transform from first root node" << std::endl;
        }
        else
        {
            std::cout << "No root nodes, using identity transform" << std::endl;
        }
    }
    else
    {
        std::cout << "No scenes, using identity transform" << std::endl;
    }

    // Define Vertex structure first
    struct Vertex
    {
        float px, py, pz;
        float nx, ny, nz;
        float u, v;
    };

    // Combine all meshes into a single mesh
    std::vector<Vertex> all_vertices;
    std::vector<uint32_t> all_indices;
    GLuint texture_id = 0;
    bool has_texture = false;
    float base_color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float metallic_factor = 0.0f;
    float roughness_factor = 1.0f;

    // Process each mesh
    for (size_t mesh_idx = 0; mesh_idx < data->meshes_count; ++mesh_idx)
    {
        auto &gltfMesh = data->meshes[mesh_idx];
        std::cout << "Processing mesh " << mesh_idx << std::endl;

        if (gltfMesh.primitives_count == 0)
        {
            std::cout << "Mesh " << mesh_idx << " has no primitives, skipping" << std::endl;
            continue;
        }

        std::cout << "Found " << gltfMesh.primitives_count << " primitive(s) in mesh " << mesh_idx << std::endl;

        // Process each primitive in the mesh
        for (size_t prim_idx = 0; prim_idx < gltfMesh.primitives_count; ++prim_idx)
        {
            auto &prim = gltfMesh.primitives[prim_idx];

            // Find accessors
            cgltf_accessor *posAcc = nullptr;
            cgltf_accessor *normAcc = nullptr;
            cgltf_accessor *uvAcc = nullptr;

            std::cout << "Processing " << prim.attributes_count << " attributes for primitive " << prim_idx << std::endl;

            for (size_t i = 0; i < prim.attributes_count; ++i)
            {
                auto &attr = prim.attributes[i];
                if (attr.type == cgltf_attribute_type_position)
                {
                    posAcc = attr.data;
                    std::cout << "Found POSITION attribute" << std::endl;
                }
                else if (attr.type == cgltf_attribute_type_normal)
                {
                    normAcc = attr.data;
                    std::cout << "Found NORMAL attribute" << std::endl;
                }
                else if (attr.type == cgltf_attribute_type_texcoord)
                {
                    uvAcc = attr.data;
                    std::cout << "Found TEXCOORD attribute" << std::endl;
                }
            }

            if (!posAcc)
            {
                std::cout << "Missing POSITION accessor in primitive " << prim_idx << ", skipping" << std::endl;
                continue;
            }

            if (!prim.indices)
            {
                std::cout << "Missing indices in primitive " << prim_idx << ", skipping" << std::endl;
                continue;
            }

            // Extract material and texture information (use first material found)
            if (prim.material && mesh_idx == 0 && prim_idx == 0)
            {
                const cgltf_material *material = prim.material;
                std::cout << "Found material for primitive" << std::endl;

                // Access PBR metallic roughness
                const cgltf_pbr_metallic_roughness &pbr = material->pbr_metallic_roughness;

                // Copy base color factor
                for (int i = 0; i < 4; ++i)
                {
                    base_color[i] = pbr.base_color_factor[i];
                }

                metallic_factor = pbr.metallic_factor;
                roughness_factor = pbr.roughness_factor;

                std::cout << "Base color: (" << base_color[0] << ", " << base_color[1]
                          << ", " << base_color[2] << ", " << base_color[3] << ")" << std::endl;
                std::cout << "Metallic: " << metallic_factor << ", Roughness: " << roughness_factor << std::endl;

                // Load base color texture if available
                if (pbr.base_color_texture.texture)
                {
                    const cgltf_texture *texture = pbr.base_color_texture.texture;
                    if (texture && texture->image)
                    {
                        std::cout << "Loading base color texture..." << std::endl;
                        texture_id = loadTextureFromImage(texture->image, data);
                        if (texture_id != 0)
                        {
                            has_texture = true;
                            std::cout << "Successfully loaded texture with ID: " << texture_id << std::endl;
                        }
                    }
                }
            }

            // Read vertices for this primitive
            size_t vertexCount = posAcc->count;
            std::cout << "Processing " << vertexCount << " vertices for primitive " << prim_idx << std::endl;

            size_t vertex_offset = all_vertices.size();
            all_vertices.resize(vertex_offset + vertexCount);
            cgltf_float tmp[4] = {};

            for (size_t i = 0; i < vertexCount; ++i)
            {
                size_t vert_idx = vertex_offset + i;

                // Read position
                if (cgltf_accessor_read_float(posAcc, i, tmp, 3) != 1)
                {
                    cgltf_free(data);
                    throw std::runtime_error("Failed to read position data at vertex " + std::to_string(i));
                }
                all_vertices[vert_idx].px = tmp[0];
                all_vertices[vert_idx].py = tmp[1];
                all_vertices[vert_idx].pz = tmp[2];

                // Read normal if available
                if (normAcc)
                {
                    if (cgltf_accessor_read_float(normAcc, i, tmp, 3) != 1)
                    {
                        cgltf_free(data);
                        throw std::runtime_error("Failed to read normal data at vertex " + std::to_string(i));
                    }
                    all_vertices[vert_idx].nx = tmp[0];
                    all_vertices[vert_idx].ny = tmp[1];
                    all_vertices[vert_idx].nz = tmp[2];
                }
                else
                {
                    all_vertices[vert_idx].nx = all_vertices[vert_idx].ny = all_vertices[vert_idx].nz = 0.0f;
                }

                // Read UV if available
                if (uvAcc)
                {
                    if (cgltf_accessor_read_float(uvAcc, i, tmp, 2) != 1)
                    {
                        cgltf_free(data);
                        throw std::runtime_error("Failed to read UV data at vertex " + std::to_string(i));
                    }
                    all_vertices[vert_idx].u = tmp[0];
                    all_vertices[vert_idx].v = tmp[1];
                }
                else
                {
                    all_vertices[vert_idx].u = all_vertices[vert_idx].v = 0.0f;
                }
            }

            // Read indices for this primitive
            auto *idxAcc = prim.indices;
            size_t indexCount = idxAcc->count;
            std::cout << "Processing " << indexCount << " indices for primitive " << prim_idx << std::endl;

            size_t index_offset = all_indices.size();
            all_indices.resize(index_offset + indexCount);

            // Check the component type to determine how to read indices
            if (idxAcc->component_type == cgltf_component_type_r_8u)
            {
                // 8-bit indices
                for (size_t i = 0; i < indexCount; ++i)
                {
                    cgltf_uint idx;
                    if (cgltf_accessor_read_uint(idxAcc, i, &idx, 1) != 1)
                    {
                        cgltf_free(data);
                        throw std::runtime_error("Failed to read index at position " + std::to_string(i));
                    }
                    all_indices[index_offset + i] = static_cast<uint32_t>(idx) + static_cast<uint32_t>(vertex_offset);
                }
            }
            else if (idxAcc->component_type == cgltf_component_type_r_16u)
            {
                // 16-bit indices
                for (size_t i = 0; i < indexCount; ++i)
                {
                    cgltf_uint idx;
                    if (cgltf_accessor_read_uint(idxAcc, i, &idx, 1) != 1)
                    {
                        cgltf_free(data);
                        throw std::runtime_error("Failed to read index at position " + std::to_string(i));
                    }
                    all_indices[index_offset + i] = static_cast<uint32_t>(idx) + static_cast<uint32_t>(vertex_offset);
                }
            }
            else if (idxAcc->component_type == cgltf_component_type_r_32u)
            {
                // 32-bit indices
                for (size_t i = 0; i < indexCount; ++i)
                {
                    cgltf_uint idx;
                    if (cgltf_accessor_read_uint(idxAcc, i, &idx, 1) != 1)
                    {
                        cgltf_free(data);
                        throw std::runtime_error("Failed to read index at position " + std::to_string(i));
                    }
                    all_indices[index_offset + i] = idx + static_cast<uint32_t>(vertex_offset);
                }
            }
            else
            {
                cgltf_free(data);
                throw std::runtime_error("Unsupported index component type: " + std::to_string(idxAcc->component_type));
            }
        }
    }

    std::cout << "Combined all meshes: " << all_vertices.size() << " total vertices, " << all_indices.size() << " total indices" << std::endl;

    // Calculate and display bounding box (for debugging, shows untransformed vertex data)
    if (!all_vertices.empty())
    {
        float min_x = all_vertices[0].px, max_x = all_vertices[0].px;
        float min_y = all_vertices[0].py, max_y = all_vertices[0].py;
        float min_z = all_vertices[0].pz, max_z = all_vertices[0].pz;
        
        for (const auto& v : all_vertices)
        {
            min_x = std::min(min_x, v.px); max_x = std::max(max_x, v.px);
            min_y = std::min(min_y, v.py); max_y = std::max(max_y, v.py);
            min_z = std::min(min_z, v.pz); max_z = std::max(max_z, v.pz);
        }
        
        float center_x = (min_x + max_x) / 2.0f;
        float center_y = (min_y + max_y) / 2.0f;
        float center_z = (min_z + max_z) / 2.0f;
        
        float size_x = max_x - min_x;
        float size_y = max_y - min_y;
        float size_z = max_z - min_z;
        
        std::cout << "Model bounding box (preserving original Blender position):" << std::endl;
        std::cout << "  X: [" << min_x << ", " << max_x << "] (size: " << size_x << ")" << std::endl;
        std::cout << "  Y: [" << min_y << ", " << max_y << "] (size: " << size_y << ")" << std::endl;
        std::cout << "  Z: [" << min_z << ", " << max_z << "] (size: " << size_z << ")" << std::endl;
        std::cout << "  Center: (" << center_x << ", " << center_y << ", " << center_z << ")" << std::endl;
    }

    // Upload combined mesh to OpenGL with detailed error checking
    std::cout << "Uploading combined mesh to OpenGL..." << std::endl;

    PlyMesh mesh;
    mesh.vertex_count = all_vertices.size();
    mesh.index_count = all_indices.size();

    // Check for any existing OpenGL errors first
    GLenum error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cout << "Warning: OpenGL error before mesh upload: " << error << std::endl;
    }

    // Generate VAO, VBO, EBO
    glGenVertexArrays(1, &mesh.vao);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error generating VAO: " + std::to_string(error));
    }

    glGenBuffers(1, &mesh.vbo);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error generating VBO: " + std::to_string(error));
    }

    glGenBuffers(1, &mesh.ebo);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error generating EBO: " + std::to_string(error));
    }

    std::cout << "Generated VAO: " << mesh.vao << ", VBO: " << mesh.vbo << ", EBO: " << mesh.ebo << std::endl;

    // Bind VAO
    glBindVertexArray(mesh.vao);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error binding VAO: " + std::to_string(error));
    }

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error binding VBO: " + std::to_string(error));
    }

    size_t vertex_data_size = all_vertices.size() * sizeof(Vertex);
    std::cout << "Uploading vertex data: " << all_vertices.size() << " vertices, " << vertex_data_size << " bytes" << std::endl;

    glBufferData(GL_ARRAY_BUFFER, vertex_data_size, all_vertices.data(), GL_STATIC_DRAW);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error uploading vertex data: " + std::to_string(error));
    }

    // Setup vertex attributes
    // Position attribute (location 0)
    glEnableVertexAttribArray(0);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error enabling vertex attribute 0: " + std::to_string(error));
    }

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, px));
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error setting vertex attribute 0 pointer: " + std::to_string(error));
    }

    // Normal attribute (location 1)
    glEnableVertexAttribArray(1);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error enabling vertex attribute 1: " + std::to_string(error));
    }

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, nx));
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error setting vertex attribute 1 pointer: " + std::to_string(error));
    }

    // UV attribute (location 2)
    glEnableVertexAttribArray(2);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error enabling vertex attribute 2: " + std::to_string(error));
    }

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, u));
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error setting vertex attribute 2 pointer: " + std::to_string(error));
    }

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error binding EBO: " + std::to_string(error));
    }

    size_t index_data_size = all_indices.size() * sizeof(uint32_t);
    std::cout << "Uploading index data: " << all_indices.size() << " indices, " << index_data_size << " bytes" << std::endl;

    // Validate indices before uploading
    uint32_t max_index = 0;
    for (const auto &idx : all_indices)
    {
        if (idx >= all_vertices.size())
        {
            cgltf_free(data);
            throw std::runtime_error("Invalid index found: " + std::to_string(idx) + " >= " + std::to_string(all_vertices.size()));
        }
        max_index = std::max(max_index, idx);
    }
    std::cout << "Index validation passed. Max index: " << max_index << " (vertex count: " << all_vertices.size() << ")" << std::endl;

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_data_size, all_indices.data(), GL_STATIC_DRAW);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error uploading index data: " + std::to_string(error));
    }

    // Unbind VAO
    glBindVertexArray(0);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        cgltf_free(data);
        throw std::runtime_error("OpenGL error unbinding VAO: " + std::to_string(error));
    }

    // Store material and texture data in the mesh
    mesh.texture_id = texture_id;
    mesh.has_texture = has_texture;
    mesh.base_color[0] = base_color[0];
    mesh.base_color[1] = base_color[1];
    mesh.base_color[2] = base_color[2];
    mesh.base_color[3] = base_color[3];
    mesh.metallic_factor = metallic_factor;
    mesh.roughness_factor = roughness_factor;
    mesh.local_transform = object_transform; // Store object transform to apply during rendering
    mesh.type = 0;

    std::cout << "Stored in mesh - Texture ID: " << mesh.texture_id
              << ", Has texture: " << mesh.has_texture
              << ", Base color: (" << mesh.base_color[0] << ", " << mesh.base_color[1]
              << ", " << mesh.base_color[2] << ", " << mesh.base_color[3] << ")" << std::endl;

    cgltf_free(data);
    std::cout << "Successfully loaded GLB mesh with " << all_vertices.size() << " vertices and " << all_indices.size() << " indices" << std::endl;

    return mesh;
}

Eigen::Matrix4f modelUpload::extractNodeTransform(const cgltf_node *node)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    if (node->has_matrix)
    {
        // Node has a direct 4x4 matrix
        // cgltf matrices are column-major, same as Eigen
        std::cout << "Node has direct matrix transform" << std::endl;
        for (int col = 0; col < 4; ++col)
        {
            for (int row = 0; row < 4; ++row)
            {
                transform(row, col) = node->matrix[col * 4 + row];
            }
        }
    }
    else
    {
        // Node has separate translation, rotation, scale
        Eigen::Vector3f translation(0, 0, 0);
        Eigen::Quaternionf rotation(1, 0, 0, 0); // w, x, y, z
        Eigen::Vector3f scale(1, 1, 1);

        if (node->has_translation)
        {
            translation = Eigen::Vector3f(
                node->translation[0],
                node->translation[1],
                node->translation[2]);
            std::cout << "Node translation: (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")" << std::endl;
        }

        if (node->has_rotation)
        {
            rotation = Eigen::Quaternionf(
                node->rotation[3], // w
                node->rotation[0], // x
                node->rotation[1], // y
                node->rotation[2]  // z
            );
            std::cout << "Node rotation: (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")" << std::endl;
        }

        if (node->has_scale)
        {
            scale = Eigen::Vector3f(
                node->scale[0],
                node->scale[1],
                node->scale[2]);
            std::cout << "Node scale: (" << scale.x() << ", " << scale.y() << ", " << scale.z() << ")" << std::endl;
        }

        // Build transform matrix: T * R * S
        Eigen::Affine3f affine = Eigen::Affine3f::Identity();
        affine.translate(translation);
        affine.rotate(rotation);
        affine.scale(scale);

        transform = affine.matrix();
    }

    // Process child nodes recursively if needed
    for (size_t i = 0; i < node->children_count; ++i)
    {
        std::cout << "Processing child node " << i << std::endl;
        Eigen::Matrix4f child_transform = extractNodeTransform(node->children[i]);
        transform = transform * child_transform;
    }

    return transform;
}

void modelUpload::setMatrices(const Eigen::Matrix4f &view_matrix, const Eigen::Matrix4f &projection_matrix)
{
    // Convert Eigen to GLM (column-major)
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            current_view_matrix[i][j] = view_matrix(j, i);
            current_projection_matrix[i][j] = projection_matrix(j, i);
        }
    }
}

void modelUpload::renderGLBMesh(const PlyMesh &mesh,
                                GLuint shader_program,
                                std::mutex &tf_mutex,
                                const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms)
{
    // Get model transform
    Eigen::Isometry3f tf_transform = Eigen::Isometry3f::Identity();
    {
        std::lock_guard<std::mutex> lk(tf_mutex);
        auto it = frame_transforms.find(mesh.frame_id);
        if (it != frame_transforms.end())
            tf_transform = it->second;
    }

    Eigen::Matrix4f final_model_matrix = tf_transform.matrix() * mesh.local_transform;

    // Use the GLB shader
    glUseProgram(shader_program);

    // Set uniforms with correct names
    GLint model_loc = glGetUniformLocation(shader_program, "model_matrix");
    GLint base_color_loc = glGetUniformLocation(shader_program, "base_color");
    GLint view_loc = glGetUniformLocation(shader_program, "view_matrix");
    GLint proj_loc = glGetUniformLocation(shader_program, "projection_matrix");
    GLint has_texture_loc = glGetUniformLocation(shader_program, "has_texture");
    GLint texture_loc = glGetUniformLocation(shader_program, "base_color_texture");
    GLint metallic_loc = glGetUniformLocation(shader_program, "metallic_factor");
    GLint roughness_loc = glGetUniformLocation(shader_program, "roughness_factor");

    if (model_loc != -1)
    {
        glUniformMatrix4fv(model_loc, 1, GL_FALSE, final_model_matrix.data());
    }
    if (view_loc != -1)
    {
        glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(current_view_matrix));
    }
    if (proj_loc != -1)
    {
        glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(current_projection_matrix));
    }
    if (base_color_loc != -1)
    {
        glUniform4fv(base_color_loc, 1, mesh.base_color);
    }
    if (has_texture_loc != -1)
    {
        glUniform1i(has_texture_loc, mesh.has_texture ? 1 : 0);
    }
    if (metallic_loc != -1)
    {
        glUniform1f(metallic_loc, mesh.metallic_factor);
    }
    if (roughness_loc != -1)
    {
        glUniform1f(roughness_loc, mesh.roughness_factor);
    }

    // Bind texture if available
    if (mesh.has_texture && mesh.texture_id != 0)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mesh.texture_id);
        if (texture_loc != -1)
        {
            glUniform1i(texture_loc, 0); // Texture unit 0
        }
    }

    // Render the mesh
    glBindVertexArray(mesh.vao);
    glDrawElements(GL_TRIANGLES, GLsizei(mesh.index_count), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    // Unbind texture
    if (mesh.has_texture && mesh.texture_id != 0)
    {
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    glUseProgram(0);
}

// ====================================
// ply mesh loading & rendering
// ====================================
void modelUpload::renderMesh(const PlyMesh &mesh,
                             glk::GLSLShader &shader,
                             std::mutex &tf_mutex,
                             const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms)
{
    Eigen::Isometry3f model = Eigen::Isometry3f::Identity();
    {
        std::lock_guard<std::mutex> lk(tf_mutex);
        auto it = frame_transforms.find(mesh.frame_id);
        if (it != frame_transforms.end())
            model = it->second;
    }

    shader.set_uniform("color_mode", 4);
    shader.set_uniform("model_matrix", model.matrix());

    glBindVertexArray(mesh.vao);
    glDrawElements(
        GL_TRIANGLES,
        GLsizei(mesh.index_count),
        GL_UNSIGNED_INT,
        nullptr);
    glBindVertexArray(0);
}

PlyMesh modelUpload::loadPlyBinaryLE(const std::string &path)
{
    std::ifstream in{path, std::ios::binary};
    if (!in)
        throw std::runtime_error("Failed to open PLY: " + path);

    bool binary_le = false;
    size_t vertex_count = 0, face_count = 0;
    struct Property
    {
        std::string name, type;
    };
    std::vector<Property> vertexProps;
    bool inVertexElement = false;
    std::string line;

    size_t idxX = -1, idxY = -1, idxZ = -1, idxR = -1, idxG = -1, idxB = -1;

    while (std::getline(in, line))
    {
        if (line.rfind("format", 0) == 0 && line.find("binary_little_endian") != std::string::npos)
            binary_le = true;
        if (line.rfind("element vertex", 0) == 0)
        {
            std::istringstream ss(line);
            std::string tmp;
            ss >> tmp >> tmp >> vertex_count;
            inVertexElement = true;
        }
        else if (line.rfind("element face", 0) == 0)
        {
            std::istringstream ss(line);
            std::string tmp;
            ss >> tmp >> tmp >> face_count;
            inVertexElement = false;
        }
        else if (inVertexElement && line.rfind("property", 0) == 0)
        {
            std::istringstream ss(line);
            std::string prop, type, name;
            ss >> prop >> type >> name;
            vertexProps.push_back({name, type});
            if (name == "x")
                idxX = vertexProps.size() - 1;
            if (name == "y")
                idxY = vertexProps.size() - 1;
            if (name == "z")
                idxZ = vertexProps.size() - 1;
            if (name == "red")
                idxR = vertexProps.size() - 1;
            if (name == "green")
                idxG = vertexProps.size() - 1;
            if (name == "blue")
                idxB = vertexProps.size() - 1;
        }
        if (line == "end_header")
            break;
    }

    if (!binary_le)
        throw std::runtime_error("Only binary_little_endian PLY supported");
    if (vertexProps.size() < 3)
        throw std::runtime_error("PLY header: need at least x,y,z properties");

    struct Vertex
    {
        float x, y, z;
        uint8_t r, g, b;
    };
    std::vector<Vertex> verts(vertex_count);

    // --- Read vertex data with correct types ---
    for (size_t i = 0; i < vertex_count; ++i)
    {
        float x = 0, y = 0, z = 0;
        uint8_t r = 255, g = 255, b = 255;
        for (size_t j = 0; j < vertexProps.size(); ++j)
        {
            if (vertexProps[j].type == "float")
            {
                float v;
                in.read(reinterpret_cast<char *>(&v), sizeof(float));
                if (j == idxX)
                    x = v;
                if (j == idxY)
                    y = v;
                if (j == idxZ)
                    z = v;
            }
            else if (vertexProps[j].type == "uchar")
            {
                uint8_t v;
                in.read(reinterpret_cast<char *>(&v), sizeof(uint8_t));
                if (j == idxR)
                    r = v;
                if (j == idxG)
                    g = v;
                if (j == idxB)
                    b = v;
            }
            else
            {
                // skip other types (e.g. alpha, s, t)
                if (vertexProps[j].type == "uint")
                {
                    uint32_t dummy;
                    in.read(reinterpret_cast<char *>(&dummy), sizeof(uint32_t));
                }
                else
                {
                    // Add more types if needed
                    throw std::runtime_error("Unsupported property type: " + vertexProps[j].type);
                }
            }
        }
        verts[i] = {x, y, z, r, g, b};
    }

    // --- Read faces (same as before) ---
    std::vector<uint32_t> indices;
    indices.reserve(face_count * 3);
    for (size_t f = 0; f < face_count; ++f)
    {
        uint8_t nVerts = 0;
        in.read(reinterpret_cast<char *>(&nVerts), sizeof(nVerts));
        if (!in)
            throw std::runtime_error("Unexpected EOF in face data");

        std::vector<uint32_t> faceIdx(nVerts);
        in.read(reinterpret_cast<char *>(faceIdx.data()), nVerts * sizeof(uint32_t));
        if (!in)
            throw std::runtime_error("Unexpected EOF in face data");

        if (nVerts == 3)
        {
            indices.insert(indices.end(), faceIdx.begin(), faceIdx.end());
        }
        else
        {
            for (uint8_t k = 1; k + 1 < nVerts; ++k)
            {
                indices.push_back(faceIdx[0]);
                indices.push_back(faceIdx[k]);
                indices.push_back(faceIdx[k + 1]);
            }
        }
    }

    // --- Upload to OpenGL (same as before) ---
    PlyMesh mesh;
    mesh.vertex_count = vertex_count;
    mesh.index_count = indices.size();

    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glGenBuffers(1, &mesh.ebo);

    glBindVertexArray(mesh.vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), (void *)(3 * sizeof(float)));

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);

    mesh.type = 1; // Set type to indicate this is a PLY mesh
    return mesh;
}
