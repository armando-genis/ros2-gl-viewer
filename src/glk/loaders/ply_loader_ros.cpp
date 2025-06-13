#include <glk/loaders/ply_loader_ros.hpp>
#include <iostream>

namespace glk
{

    AssimpLoaderROS::AssimpLoaderROS(const std::string &path,
                                     const std::string &tf_frame_in,
                                     std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                     const std::string &fixed_frame)
        : tf_frame(tf_frame_in)
    {
        std::cout << "[AssimpLoaderROS] Loading mesh from '" << path << "' (frame='"
                  << tf_frame << "')" << std::endl;

        // 1) Import scene with Assimp
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(
            path,
            aiProcess_Triangulate |
                aiProcess_GenSmoothNormals |
                aiProcess_JoinIdenticalVertices |
                aiProcess_ImproveCacheLocality);
        if (!scene || !scene->HasMeshes())
        {
            std::cerr << "[AssimpLoaderROS] ERROR: Failed to load mesh: '" << path << "'"
                      << std::endl;
            return;
        }

        aiMesh *mesh = scene->mMeshes[0];
        std::cout << "[AssimpLoaderROS] Mesh has "
                  << mesh->mNumVertices << " vertices and "
                  << mesh->mNumFaces << " faces" << std::endl;

        // 2) Extract first mesh
        std::vector<float> verts;
        std::vector<unsigned int> inds;
        verts.reserve(mesh->mNumVertices * 6);
        for (unsigned i = 0; i < mesh->mNumVertices; ++i)
        {
            aiVector3D p = mesh->mVertices[i];
            aiVector3D n = mesh->mNormals[i];
            verts.push_back(p.x);
            verts.push_back(p.y);
            verts.push_back(p.z);
            verts.push_back(n.x);
            verts.push_back(n.y);
            verts.push_back(n.z);
        }
        for (unsigned i = 0; i < mesh->mNumFaces; ++i)
        {
            aiFace &f = mesh->mFaces[i];
            if (f.mNumIndices == 3)
            {
                inds.push_back(f.mIndices[0]);
                inds.push_back(f.mIndices[1]);
                inds.push_back(f.mIndices[2]);
            }
        }
        std::cout << "[AssimpLoaderROS] Prepared " << inds.size() / 3 << " triangles for rendering" << std::endl;

        // 3) Setup OpenGL buffers
        setupBuffers(verts, inds);
        std::cout << "[AssimpLoaderROS] OpenGL VAO=" << vao
                  << ", VBO=" << vbo
                  << ", EBO=" << ebo
                  << ", index_count=" << index_count
                  << std::endl;

        // 4) Initial TF lookup
        try
        {
            auto tfst = tf_buffer->lookupTransform(
                fixed_frame, tf_frame, tf2::TimePointZero);
            Eigen::Quaternionf q(
                tfst.transform.rotation.w,
                tfst.transform.rotation.x,
                tfst.transform.rotation.y,
                tfst.transform.rotation.z);
            Eigen::Vector3f t(
                tfst.transform.translation.x,
                tfst.transform.translation.y,
                tfst.transform.translation.z);
            transform = Eigen::Isometry3f::Identity();
            transform.rotate(q);
            transform.pretranslate(t);
            std::cout << "[AssimpLoaderROS] Initial TF: t=("
                      << t.transpose() << ")" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[AssimpLoaderROS] TF lookup failed for '" << tf_frame << "': "
                      << e.what() << std::endl;
        }
    }

    void AssimpLoaderROS::setupBuffers(const std::vector<float> &verts,
                                       const std::vector<unsigned int> &inds)
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     verts.size() * sizeof(float),
                     verts.data(),
                     GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        index_count = static_cast<GLsizei>(inds.size());
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     inds.size() * sizeof(unsigned int),
                     inds.data(),
                     GL_STATIC_DRAW);

        // position (loc=0)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              6 * sizeof(float), (void *)0);
        // normal (loc=1)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                              6 * sizeof(float), (void *)(3 * sizeof(float)));
        glBindVertexArray(0);
    }

} // namespace glk