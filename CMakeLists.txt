
cmake_minimum_required(VERSION 3.10)
project(ros2-gl-viewer)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Freetype REQUIRED)
find_package(GLUT   REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ICU REQUIRED icu-uc icu-i18n)
pkg_check_modules(LIBUV REQUIRED libuv)

# ROS2 dependencies - without using ament
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# load 3d graphics libraries (Assimp)
find_package(assimp REQUIRED) 
include_directories(${ASSIMP_INCLUDE_DIRS})

# Find gl3w
set(GL3W_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gl3w)
include_directories(${GL3W_DIR}/include)

# Set ImGui directory
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
include_directories(${IMGUI_DIR})
set(IMGUI_IMPL_DIR ${IMGUI_DIR}/backends)
include_directories(${IMGUI_IMPL_DIR})

# Lanelet2
set(LANELET2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Rosless-Lanelet2")
list(APPEND CMAKE_MODULE_PATH "${LANELET2_ROOT}/cmake")
add_subdirectory("${LANELET2_ROOT}" "${CMAKE_BINARY_DIR}/lanelet2_build")

# Cgltf library
set(CGLTF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cgltf)
set(STB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
include_directories(${CGLTF_DIR})
include_directories(${STB_DIR})

# Mapbox GL Native - Use pre-built libraries
set(MAPBOX_GL_NATIVE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mapbox-gl-native)
set(MAPBOX_GL_NATIVE_BUILD_DIR ${MAPBOX_GL_NATIVE_DIR}/build)

# Include Mapbox GL Native headers
include_directories(${MAPBOX_GL_NATIVE_DIR}/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/platform/default/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/platform/default/include/mbgl/storage)

# Include Mapbox vendor dependencies that are needed for headers
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/optional)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/variant/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/geometry.hpp/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/geojson.hpp/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/rapidjson/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/expected-lite/include)

# Find Mapbox GL Native library and its vendor dependencies
find_library(MAPBOX_CORE_LIB
    NAMES mbgl-core
    PATHS ${MAPBOX_GL_NATIVE_BUILD_DIR}
    NO_DEFAULT_PATH
)

# Find Mapbox vendor libraries
find_library(MAPBOX_CSSCOLORPARSER_LIB
    NAMES mbgl-vendor-csscolorparser
    PATHS ${MAPBOX_GL_NATIVE_BUILD_DIR}
    NO_DEFAULT_PATH
)

find_library(MAPBOX_NUNICODE_LIB
    NAMES mbgl-vendor-nunicode
    PATHS ${MAPBOX_GL_NATIVE_BUILD_DIR}
    NO_DEFAULT_PATH
)

find_library(MAPBOX_PARSEDATE_LIB
    NAMES mbgl-vendor-parsedate
    PATHS ${MAPBOX_GL_NATIVE_BUILD_DIR}
    NO_DEFAULT_PATH
)

find_library(MAPBOX_SQLITE_LIB
    NAMES mbgl-vendor-sqlite
    PATHS ${MAPBOX_GL_NATIVE_BUILD_DIR}
    NO_DEFAULT_PATH
)

# Check if library was found
if(NOT MAPBOX_CORE_LIB)
    message(FATAL_ERROR "Mapbox GL Native core library not found in ${MAPBOX_GL_NATIVE_BUILD_DIR}")
endif()

message(STATUS "Found Mapbox GL Native: ${MAPBOX_CORE_LIB}")


# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${std_msgs_LIBRARIES}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${FREETYPE_INCLUDE_DIRS}
)

set(GLK_SOURCES
  src/glk/colormap.cpp
  src/glk/frame_buffer.cpp
  src/glk/glsl_shader.cpp
  src/glk/lines.cpp
  src/glk/mesh.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/loaders/ply_loader.cpp
  src/glk/primitives/primitives.cpp
  src/glk/TextRendering.cpp
  src/glk/PclLoader.cpp
  src/glk/modelUpload.cpp
  src/glk/LaneletLoader.cpp
  src/glk/ThickLines.cpp
  src/glk/MapboxRenderer.cpp
)

# List all source files in guik directory
set(GUIK_SOURCES
  src/guik/camera_control.cpp
  src/guik/gl_canvas.cpp
  src/guik/imgui_application.cpp
  src/guik/model_control.cpp
  src/guik/projection_control.cpp
)

add_executable(pointcloud_viewer
  src/main.cpp
  ${GLK_SOURCES}
  ${GUIK_SOURCES}
  ${GL3W_DIR}/src/gl3w.c
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Add compile definitions for the libraries
target_compile_definitions(pointcloud_viewer PRIVATE
    CGLTF_IMPLEMENTATION
    # STB_IMAGE_IMPLEMENTATION
)

target_include_directories(pointcloud_viewer PRIVATE
  ${LIBUV_INCLUDE_DIRS}
)


# Link libraries
target_link_libraries(pointcloud_viewer
  ${OPENGL_LIBRARIES}
  glfw
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  dl  # For gl3w dynamic loading
  ${FREETYPE_LIBRARIES}
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_traffic_rules
  lanelet2_routing
  lanelet2_validation
  ${GLUT_LIBRARY}
  # Mapbox GL Native libraries
  ${MAPBOX_CORE_LIB}
  ${MAPBOX_CSSCOLORPARSER_LIB}
  ${MAPBOX_NUNICODE_LIB}
  ${MAPBOX_PARSEDATE_LIB}
  ${MAPBOX_SQLITE_LIB}

  ${LIBUV_LIBRARIES}
  # System dependencies for Mapbox
  ${JPEG_LIBRARIES}
  ${PNG_LIBRARIES}
  ${ZLIB_LIBRARIES}
  ${ICU_LIBRARIES}
  curl
  GL
  pthread
  m
)


# Make sure the application finds the shader files
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/data
  ${CMAKE_BINARY_DIR}/data
)

# Copy the icon file from src to build directory
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${CMAKE_SOURCE_DIR}/src/appicon.png
  ${CMAKE_BINARY_DIR}/appicon.png
  COMMENT "Copying appicon.png to build directory"
)

# Install the executable
install(TARGETS pointcloud_viewer
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

