#include <glk/MapboxRenderer.hpp>
#include <iostream>
#include <cmath>
#include <cstdlib> // for setenv

// Add required MapOptions include
#include <mbgl/map/map_options.hpp>

MapboxRenderer::MapboxRenderer(int width,
                               int height,
                               const std::string &accessToken,
                               float pixelRatio)
    : width_(width), height_(height), pixelRatio_(pixelRatio)
{
    std::cout << "Initializing MapboxRenderer with dimensions: " << width << "x" << height << std::endl;

    try
    {
        runLoop_ = std::make_unique<mbgl::util::RunLoop>();

        // Set environment variable for access token (most reliable method)
        setenv("MAPBOX_ACCESS_TOKEN", accessToken.c_str(), 1);
        std::cout << "✅ Access token set via environment variable" << std::endl;

        // Create headless frontend first
        frontend_ = std::make_unique<mbgl::HeadlessFrontend>(
            mbgl::Size{static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)},
            pixelRatio_);
        std::cout << "✅ Headless frontend created" << std::endl;

        // Create MapOptions
        mbgl::MapOptions mapOptions;
        mapOptions.withMapMode(mbgl::MapMode::Static)
            .withConstrainMode(mbgl::ConstrainMode::None)
            .withViewportMode(mbgl::ViewportMode::Default)
            .withCrossSourceCollisions(true)
            .withNorthOrientation(mbgl::NorthOrientation::Upwards)
            .withSize(mbgl::Size{static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)});
        std::cout << "✅ Map options configured" << std::endl;

        // Create Map using the only available 4-parameter constructor
        std::cout << "🔄 Creating Map instance..." << std::endl;

        // Create ResourceOptions using default constructor only (no configuration to avoid copy issues)
        mbgl::ResourceOptions resourceOptions;
        resourceOptions.withAccessToken(accessToken);

        map_ = std::make_unique<mbgl::Map>(
            *frontend_,                        // RendererFrontend&
            mbgl::MapObserver::nullObserver(), // MapObserver&
            std::move(mapOptions),             // const MapOptions&
            std::move(resourceOptions)         // const ResourceOptions&
        );
        std::cout << "✅ Map created successfully" << std::endl;

        // ✅ REMOVE THESE LINES:
        // // Load a default style
        // std::cout << "🔄 Loading default map style..." << std::endl;
        // map_->getStyle().loadURL("mapbox://styles/mapbox/streets-v11");
        // std::cout << "✅ Map style loaded" << std::endl;

        // Generate OpenGL texture for map pixels
        std::cout << "🔄 Creating OpenGL texture..." << std::endl;
        glGenTextures(1, &texture_);
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA,
            width_, height_, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);
        std::cout << "✅ OpenGL texture created (ID: " << texture_ << ")" << std::endl;

        // Setup quad geometry
        std::cout << "🔄 Setting up quad geometry..." << std::endl;
        setupQuadGeometry();

        // Create shader program manually
        std::cout << "🔄 Creating map shader..." << std::endl;
        if (!createMapShader())
        {
            throw std::runtime_error("Failed to create map shader");
        }

        // ✅ REMOVE: All the commented style loading, wait code, and map parameter setting

        mapReady_ = true;
        std::cout << "🎉 MapboxRenderer initialized successfully!" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Failed to initialize MapboxRenderer: " << e.what() << std::endl;
        mapReady_ = false;
    }
}

MapboxRenderer::~MapboxRenderer()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (quadVAO_ != 0)
    {
        glDeleteVertexArrays(1, &quadVAO_);
    }
    if (quadVBO_ != 0)
    {
        glDeleteBuffers(1, &quadVBO_);
    }
    if (quadEBO_ != 0)
    {
        glDeleteBuffers(1, &quadEBO_);
    }
    if (shaderProgram_ != 0)
    {
        glDeleteProgram(shaderProgram_);
    }
    if (texture_ != 0)
    {
        glDeleteTextures(1, &texture_);
    }

    // Destroy mbgl objects before the run loop
    map_.reset();
    frontend_.reset();
    runLoop_.reset();

    std::cout << "MapboxRenderer destroyed" << std::endl;
}

void MapboxRenderer::setupQuadGeometry()
{
    // Fullscreen quad vertices: position(x,y) + texcoord(u,v)
    float quadVertices[] = {
        // positions   // texCoords
        -1.0f, 1.0f, 0.0f, 1.0f,  // top left
        -1.0f, -1.0f, 0.0f, 0.0f, // bottom left
        1.0f, -1.0f, 1.0f, 0.0f,  // bottom right
        1.0f, 1.0f, 1.0f, 1.0f    // top right
    };

    unsigned int indices[] = {
        0, 1, 2, // first triangle
        0, 2, 3  // second triangle
    };

    glGenVertexArrays(1, &quadVAO_);
    glGenBuffers(1, &quadVBO_);
    glGenBuffers(1, &quadEBO_);

    glBindVertexArray(quadVAO_);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadEBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Position attribute (location = 0)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Texture coordinate attribute (location = 1)
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    std::cout << "✅ Quad geometry setup complete (VAO: " << quadVAO_ << ")" << std::endl;
}

bool MapboxRenderer::createMapShader()
{
    const char *vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec2 aPos;
        layout(location = 1) in vec2 aTex;
        
        uniform mat4 uMVP;
        
        out vec2 vTex;
        
        void main() {
            vTex = aTex;
            gl_Position = uMVP * vec4(aPos, 0.0, 1.0);
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        in vec2 vTex;
        out vec4 FragColor;
        
        uniform sampler2D uMap;
        uniform float uAlpha;
        
        void main() {
            vec4 mapColor = texture(uMap, vTex);
            FragColor = vec4(mapColor.rgb, mapColor.a * uAlpha);
        }
    )";

    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // Check vertex shader compilation
    GLint success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "❌ Map vertex shader compilation failed: " << infoLog << std::endl;
        glDeleteShader(vertexShader);
        return false;
    }

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // Check fragment shader compilation
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "❌ Map fragment shader compilation failed: " << infoLog << std::endl;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return false;
    }

    // Create and link shader program
    shaderProgram_ = glCreateProgram();
    glAttachShader(shaderProgram_, vertexShader);
    glAttachShader(shaderProgram_, fragmentShader);
    glLinkProgram(shaderProgram_);

    // Check linking
    glGetProgramiv(shaderProgram_, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram_, 512, NULL, infoLog);
        std::cerr << "❌ Map shader program linking failed: " << infoLog << std::endl;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(shaderProgram_);
        shaderProgram_ = 0;
        return false;
    }

    // Clean up individual shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    std::cout << "✅ Map shader program created successfully (ID: " << shaderProgram_ << ")" << std::endl;
    return true;
}

void MapboxRenderer::render()
{
    if (!mapReady_)
    {
        std::cout << "[Mapbox] ❌ Map not ready, skipping render" << std::endl;
        return;
    }

    // ✅ CHECK IF ALREADY RENDERING
    if (isRendering_.load(std::memory_order_acquire))
    {
        std::cout << "[Mapbox] ⚠️ Already rendering, skipping" << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    try
    {
        // ✅ SET RENDERING FLAG
        isRendering_.store(true, std::memory_order_release);

        static int render_counter = 0;
        render_counter++;
        std::cout << "[Mapbox] 🎬 Starting render #" << render_counter << std::endl;

        // ✅ ADD: Get current map info
        try
        {
            auto center = map_->getCameraOptions().center;
            auto zoom = map_->getCameraOptions().zoom;
            if (center && zoom)
            {
                std::cout << "[Mapbox] 🗺️ Current map: lat=" << center->latitude()
                          << ", lon=" << center->longitude() << ", zoom=" << *zoom << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cout << "[Mapbox] ⚠️ Could not get camera info: " << e.what() << std::endl;
        }

        // ✅ CLEAR THE FLAG IMMEDIATELY AFTER renderStill CALL
        map_->renderStill([&](std::exception_ptr ep)
                          {
            if (ep) { 
                try { 
                    std::rethrow_exception(ep); 
                } catch (const std::exception& e) {
                    std::cerr << "[Mapbox] ❌ Map render error: " << e.what() << std::endl;
                }
            } else {
                lastSuccessfulRender_ = std::chrono::steady_clock::now();
                std::cout << "[Mapbox] ✅ Render callback completed successfully" << std::endl;
            } });

        // ✅ CLEAR FLAG IMMEDIATELY (don't wait for callback)
        isRendering_.store(false, std::memory_order_release);
        std::cout << "[Mapbox] 🔓 Rendering flag cleared" << std::endl;

        auto image = frontend_->readStillImage();
        if (image.data && image.size.width > 0 && image.size.height > 0)
        {
            const size_t nbytes = static_cast<size_t>(image.size.width) *
                                  static_cast<size_t>(image.size.height) * 4;
            pendingPixels_.assign(image.data.get(), image.data.get() + nbytes);
            hasNewPixels_.store(true, std::memory_order_release);

            // ✅ ADD: More detailed pixel debugging
            static int pixel_debug_count = 0;
            if (++pixel_debug_count <= 5)
            {
                std::cout << "[Mapbox] 🔍 First 32 bytes of pixel data: ";
                for (size_t i = 0; i < 32 && i < nbytes; i++)
                { // ✅ Fixed: use size_t
                    printf("%02x ", pendingPixels_[i]);
                }
                std::cout << std::endl;

                // Check middle and end of image too
                size_t mid = nbytes / 2;
                if (mid + 16 < nbytes)
                {
                    std::cout << "[Mapbox] 🔍 Middle 16 bytes: ";
                    for (size_t i = 0; i < 16; i++)
                    {
                        printf("%02x ", pendingPixels_[mid + i]);
                    }
                    std::cout << std::endl;
                }

                // Check if all pixels are the same
                bool all_same = true;
                unsigned char first = pendingPixels_[0];
                for (size_t i = 1; i < std::min((size_t)1000, nbytes); i++)
                { // ✅ Fixed: use size_t
                    if (pendingPixels_[i] != first)
                    {
                        all_same = false;
                        break;
                    }
                }

                std::cout << "[Mapbox] 🎨 Pixel analysis: "
                          << (all_same ? "ALL SAME VALUE" : "VARIED VALUES")
                          << " (first byte: 0x" << std::hex << (int)first << std::dec << ")" << std::endl;
            }

            std::cout << "[Mapbox] 📸 Got " << nbytes << " bytes of map data ("
                      << image.size.width << "x" << image.size.height << ")" << std::endl;
        }
        else
        {
            std::cout << "[Mapbox] ⚠️ No image data received from frontend" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        isRendering_.store(false, std::memory_order_release);
        std::cerr << "[Mapbox] ❌ Error in render(): " << e.what() << std::endl;
    }
}

void MapboxRenderer::uploadLatestPixelsToTexture()
{
    if (!mapReady_ || shaderProgram_ == 0)
    {
        static int skip_counter = 0;
        if (++skip_counter % 60 == 0)
        { // Log every second at 60fps
            std::cout << "[Mapbox] ⚠️ Upload skipped - not ready (ready:" << mapReady_
                      << ", shader:" << shaderProgram_ << ")" << std::endl;
        }
        return;
    }

    if (!hasNewPixels_.load(std::memory_order_acquire))
    {
        static int no_pixels_counter = 0;
        if (++no_pixels_counter % 60 == 0)
        { // Log every second at 60fps
            std::cout << "[Mapbox] ⏳ No new pixels to upload" << std::endl;
        }
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (pendingPixels_.empty())
    {
        std::cout << "[Mapbox] ⚠️ hasNewPixels=true but pendingPixels empty!" << std::endl;
        return;
    }

    std::cout << "[Mapbox] 🔄 Uploading " << pendingPixels_.size() << " bytes to texture " << texture_ << std::endl;

    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                    width_, height_, GL_RGBA, GL_UNSIGNED_BYTE,
                    pendingPixels_.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    hasNewPixels_.store(false, std::memory_order_release);

    std::cout << "[Mapbox] ✅ Texture upload completed" << std::endl;
}

void MapboxRenderer::drawMap(const Eigen::Matrix4f &view_matrix,
                             const Eigen::Matrix4f &projection_matrix,
                             const Eigen::Vector3f &position,
                             float size,
                             float rotation_z)
{
    if (!mapReady_ || shaderProgram_ == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Create model matrix for map position in 3D space
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();

    // Apply translation
    model_matrix.block<3, 1>(0, 3) = position;

    // Apply Z rotation
    if (std::abs(rotation_z) > 1e-6f)
    {
        float cos_r = std::cos(rotation_z);
        float sin_r = std::sin(rotation_z);
        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation(0, 0) = cos_r;
        rotation(0, 1) = -sin_r;
        rotation(1, 0) = sin_r;
        rotation(1, 1) = cos_r;
        model_matrix = model_matrix * rotation;
    }

    // Apply scale
    model_matrix(0, 0) *= size;
    model_matrix(1, 1) *= size;

    // Calculate MVP matrix
    Eigen::Matrix4f mvp = projection_matrix * view_matrix * model_matrix;

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Use shader and set uniforms
    glUseProgram(shaderProgram_);

    // Set MVP matrix uniform
    GLint mvpLoc = glGetUniformLocation(shaderProgram_, "uMVP");
    if (mvpLoc != -1)
    {
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.data());
    }

    // Set texture uniform
    GLint mapLoc = glGetUniformLocation(shaderProgram_, "uMap");
    if (mapLoc != -1)
    {
        glUniform1i(mapLoc, 0); // Texture unit 0
    }

    // Set alpha uniform
    GLint alphaLoc = glGetUniformLocation(shaderProgram_, "uAlpha");
    if (alphaLoc != -1)
    {
        glUniform1f(alphaLoc, 1.0f); // Full opacity
    }

    // ✅ SWITCH BACK TO MAP TEXTURE WITH DEBUG
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_);

    // ✅ DEBUG: Check texture parameters
    static int debug_count = 0;
    if (++debug_count % 60 == 0)
    { // Every second
        GLint width, height, format;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &format);

        std::cout << "[Mapbox] 🖼️ Map texture info: "
                  << width << "x" << height
                  << " format=" << format
                  << " texture_id=" << texture_ << std::endl;
    }

    // Render quad
    glBindVertexArray(quadVAO_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // Cleanup
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    glDisable(GL_BLEND);
}

void MapboxRenderer::resize(int width, int height)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    width_ = width;
    height_ = height;

    try
    {
        // Resize frontend and map
        frontend_->setSize(mbgl::Size{static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)});
        map_->setSize(mbgl::Size{static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)});

        // Resize OpenGL texture
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA,
            width_, height_, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindTexture(GL_TEXTURE_2D, 0);

        std::cout << "MapboxRenderer resized to: " << width << "x" << height << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error resizing MapboxRenderer: " << e.what() << std::endl;
    }
}

GLuint MapboxRenderer::getTexture() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return texture_;
}

void MapboxRenderer::setMapCenter(double latitude, double longitude)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        mbgl::CameraOptions camera;
        camera.center = mbgl::LatLng{latitude, longitude};
        map_->jumpTo(camera);
        std::cout << "Map center set to: " << latitude << ", " << longitude << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting map center: " << e.what() << std::endl;
    }
}

void MapboxRenderer::setZoom(double zoom)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        mbgl::CameraOptions camera;
        camera.zoom = zoom;
        map_->jumpTo(camera);
        std::cout << "Map zoom set to: " << zoom << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting zoom: " << e.what() << std::endl;
    }
}

void MapboxRenderer::setStyle(const std::string &styleURL)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        map_->getStyle().loadURL(styleURL);
        std::cout << "Map style set to: " << styleURL << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting map style: " << e.what() << std::endl;
    }
}

void MapboxRenderer::setBearing(double bearing)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        mbgl::CameraOptions camera;
        camera.bearing = bearing;
        map_->jumpTo(camera);
        std::cout << "Map bearing set to: " << bearing << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting bearing: " << e.what() << std::endl;
    }
}

void MapboxRenderer::setPitch(double pitch)
{
    if (!mapReady_)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        mbgl::CameraOptions camera;
        camera.pitch = pitch;
        map_->jumpTo(camera);
        std::cout << "Map pitch set to: " << pitch << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error setting pitch: " << e.what() << std::endl;
    }
}

mbgl::LatLng MapboxRenderer::getMapCenter() const
{
    if (!mapReady_)
    {
        return mbgl::LatLng{0.0, 0.0};
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        auto options = map_->getCameraOptions();
        if (options.center)
        {
            return *options.center;
        }
        return mbgl::LatLng{0.0, 0.0};
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting map center: " << e.what() << std::endl;
        return mbgl::LatLng{0.0, 0.0};
    }
}

double MapboxRenderer::getZoom() const
{
    if (!mapReady_)
    {
        return 0.0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
        auto options = map_->getCameraOptions();
        if (options.zoom)
        {
            return *options.zoom;
        }
        return 0.0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting zoom: " << e.what() << std::endl;
        return 0.0;
    }
}

bool MapboxRenderer::isMapReady() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return mapReady_;
}