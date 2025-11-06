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
sudo apt-get install libuv1-dev pkg-config
sudo apt-get install libjpeg-dev libpng-dev libz-dev libcurl4-openssl-dev
sudo apt-get install libicu-dev pkg-config

cd /ros2-gl-viewer/third_party

# Clone ImGui
git clone https://github.com/ocornut/imgui.git

# Clone and generate gl3w
git clone https://github.com/skaslev/gl3w.git

git clone https://github.com/embedded-software-laboratory/Rosless-Lanelet2.git
cd Rosless-Lanelet2

mkdir build && cd build
cmake ..
cmake --build . -- -j$(nproc)
sudo cmake --install .

git clone https://github.com/jkuhlmann/cgltf.git
wget https://raw.githubusercontent.com/nothings/stb/master/stb_image.h

cd /ros2-gl-viewer/third_party/gl3w
python3 gl3w_gen.py

git clone https://github.com/mapbox/mapbox-gl-native.git
cd mapbox-gl-native
git submodule update --init --recursive
cmake . -B build
cmake --build build


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

Found frame: zedd with parent: base_link
Found frame: vectornav with parent: base_link
Found frame: velodyne with parent: base_link
Found frame: base_footprint with parent: base_link

Found frame: 'zedd' with parent: 'base_link'
Found frame: 'velodyne' with parent: 'base_link'
Found frame: 'base_footprint' with parent: 'base_link'
Found frame: 'vectornav' with parent: 'base_link'

/workspace/models/map_2.ply

/workspace/models/map_tec.ply

/workspace/models/sdv.ply

/workspace/models/open_house.pcd

/workspace/oms/open_house_v2.osm

/workspace/oms/map_homework.osm

/workspace/oms/me_4.osm

/workspace/oms/me_5.osm

/workspace/models/LOAM/cloudGlobal.pcd

/workspace/models/sdv.glb
/workspace/models/sombrero.glb
/workspace/models/spooky.glb
/workspace/models/ghost.glb

/workspace/models/concept_rot.glb
/workspace/models/hdl64_rot.glb
