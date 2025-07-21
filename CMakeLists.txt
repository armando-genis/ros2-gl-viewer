
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
  # src/glk/ThickLines.cpp
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
    STB_IMAGE_IMPLEMENTATION
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
)


# Make sure the application finds the shader files
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/data
  ${CMAKE_BINARY_DIR}/data
)

# Install the executable
install(TARGETS pointcloud_viewer
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)