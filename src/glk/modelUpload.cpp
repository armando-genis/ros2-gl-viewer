#include <glk/modelUpload.hpp>
#include <iostream>

modelUpload::modelUpload(/* args */)
{
}

modelUpload::~modelUpload()
{
}

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
    return mesh;
}
