// MapboxRenderer.hpp
#pragma once

#include <memory>
#include <string>
#include <GL/gl3w.h>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>

#include <mbgl/util/run_loop.hpp>

// Mapbox includes
#include <mbgl/map/map.hpp>
#include <mbgl/gfx/headless_frontend.hpp>
#include <mbgl/storage/resource_options.hpp>
#include <mbgl/style/style.hpp>
#include <mbgl/util/size.hpp>
#include <mbgl/util/image.hpp>
#include <mbgl/util/geo.hpp>
#include <mbgl/map/map_options.hpp>

class MapboxRenderer
{
public:
    /**
     * Constructor
     * @param width        Framebuffer width in pixels
     * @param height       Framebuffer height in pixels
     * @param accessToken  Mapbox access token for API authentication
     * @param pixelRatio   Device pixel ratio (default: 1.0f)
     */
    MapboxRenderer(int width,
                   int height,
                   const std::string &accessToken,
                   float pixelRatio = 1.0f);
    ~MapboxRenderer();

    // Render the map into the texture (call each frame before drawing quad)
    void render();

    // Draw the map as a 3D quad in the scene
    void drawMap(const Eigen::Matrix4f &view_matrix,
                 const Eigen::Matrix4f &projection_matrix,
                 const Eigen::Vector3f &position,
                 float size = 100.0f,
                 float rotation_z = 0.0f);

    // Resize framebuffer, frontend, map, and texture
    void resize(int width, int height);

    // Get OpenGL texture ID containing the rendered map
    GLuint getTexture() const;

    // Map control methods
    void setMapCenter(double latitude, double longitude);
    void setZoom(double zoom);
    void setStyle(const std::string &styleURL);
    void setBearing(double bearing);
    void setPitch(double pitch);

    // Get current map state
    mbgl::LatLng getMapCenter() const;
    double getZoom() const;

    // Check if map is ready to render
    bool isMapReady() const;

    std::unique_ptr<mbgl::util::RunLoop> runLoop_;

    std::vector<uint8_t> pendingPixels_;
    std::atomic<bool> hasNewPixels_{false};

    void uploadLatestPixelsToTexture();

    std::atomic<bool> isRendering_{false};
    std::chrono::steady_clock::time_point lastSuccessfulRender_;

private:
    // Setup quad geometry for rendering
    void setupQuadGeometry();

    // Create shader program for map rendering
    bool createMapShader();

    // Mapbox headless frontend and map instance
    std::unique_ptr<mbgl::HeadlessFrontend> frontend_;
    std::unique_ptr<mbgl::Map> map_;

    // OpenGL resources
    GLuint texture_ = 0;
    GLuint quadVAO_ = 0;
    GLuint quadVBO_ = 0;
    GLuint quadEBO_ = 0;
    GLuint shaderProgram_ = 0; // OpenGL shader program

    // Current framebuffer dimensions & DPR
    int width_ = 0;
    int height_ = 0;
    float pixelRatio_ = 1.0f;

    // Map state
    bool mapReady_ = false;

    // Protect concurrent access
    mutable std::mutex mutex_;
};