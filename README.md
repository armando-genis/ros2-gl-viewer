# ros-gl-viewer
A lightweight, OpenGL-powered 3D visualization tool for ROS 2, ideal for SLAM, perception debugging and rapid prototyping


![gl Screenshot](https://github.com/armando-genis/ros2-gl-viewer/blob/main/img/img2.png)

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-tf2-eigen
sudo apt install -y libglm-dev libglfw3-dev libeigen3-dev
sudo apt update
sudo apt install libfreetype6-dev
sudo apt install libglfw3-dev libglew-dev libglm-dev libglu1-mesa-dev freeglut3-dev
sudo apt-get install libassimp-dev


cd /ros2-gl-viewer/third_party

# Clone ImGui
git clone https://github.com/ocornut/imgui.git

# Clone and generate gl3w
git clone https://github.com/skaslev/gl3w.git

cd /ros2-gl-viewer/third_party/gl3w
python3 gl3w_gen.py

cd /ros2-gl-viewer

mkdir build 
cd build 

cmake ..
make 

./pointcloud_viewer

```

- https://github.com/rtryan98/OpenGL
- https://glm.g-truc.net/0.9.9/index.html
- https://github.com/EdoardoLuciani/OpenGL-4.5-Freetype-Example/blob/master/main.cpp



in the /stdafx.h

#include "FastNoise/FastNoise.h"
#include "FastNoise/Metadata.h"


