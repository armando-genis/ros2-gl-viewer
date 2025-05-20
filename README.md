# ros-gl-viewer
A lightweight, OpenGL-powered 3D visualization tool for ROS 2, ideal for SLAM, perception debugging and rapid prototyping


![rviz2 Screenshot](https://github.com/armando-genis/rviz_dark_custom_theme/blob/main/img/rviz_img.png)

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-tf2-eigen
sudo apt install -y libglm-dev libglfw3-dev libeigen3-dev

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