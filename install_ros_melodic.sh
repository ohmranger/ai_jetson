#!/bin/bash
# Apache License 2.0
# Copyright (c) 2020, ROBOTIS CO., LTD.
#sudo nmcli device wifi connect 'ohmranger' password '55555555'
#sudo timedatectl set-ntp false && timedatectl set-time "2025-01-13 22:30:00"
#sudo timedatectl set-time "2025-01-09 15:30:00"
#export DISPLAY:=0.0
#/usr/lib/vino/vino-server

sudo rm -rf /usr/local/cuda/samples \
/usr/src/cudnn_samples_* \
/usr/src/tensorrt/data \
/usr/src/tensorrt/samples \
/usr/share/visionworks* ~/VisionWorks-SFM*Samples \
/opt/nvidia/deepstream/deepstream*/samples
sudo apt install nano -y
echo ""
echo "[Note] Target OS version  >>> Ubuntu 20.04.x (Focal Fossa) or Linux Mint 21.x"
echo "[Note] Target ROS version >>> ROS $name_ros_version Ninjemys"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read
echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="bionic"}
name_ros_version=${name_ros_version:="melodic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}
echo "[Update the package lists]"
sudo apt update -y
echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com
echo "[Add the ROS repository]"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 

echo "[Update the package lists]"
sudo apt update -y
echo "[Install ros-desktop-full version of $name_ros_version"
sudo apt install -y ros-$name_ros_version-desktop-full
echo "[Install RQT & Gazebo]"
sudo apt install -y ros-$name_ros_version-rqt-* ros-$name_ros_version-gazebo-*
echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git
echo "[Install rosdep and Update]"
sudo apt install python3-rosdep -y
echo "[Initialize rosdep and Update]"
sudo sh -c "rosdep init"
rosdep update
echo "[Make the catkin workspace and test the catkin_make]"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin_make
echo "[Set the ROS evironment]"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_catkin_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_catkin_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cm='cd ~/$name_catkin_workspace && catkin_make'\" >> ~/.bashrc"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export TURTLEBOT3_MODEL=burger\" >> ~/.bashrc"
source $HOME/.bashrc
sudo apt-get install ros-$name_ros_version-joy ros-$name_ros_version-teleop-twist-joy \
  ros-$name_ros_version-teleop-twist-keyboard ros-$name_ros_version-laser-proc \
  ros-$name_ros_version-rgbd-launch ros-$name_ros_version-rosserial-arduino \
  ros-$name_ros_version-rosserial-python ros-$name_ros_version-rosserial-client \
  ros-$name_ros_version-rosserial-msgs ros-$name_ros_version-amcl ros-$name_ros_version-map-server \
  ros-$name_ros_version-move-base ros-$name_ros_version-urdf ros-$name_ros_version-xacro \
  ros-$name_ros_version-compressed-image-transport ros-$name_ros_version-rqt* ros-$name_ros_version-rviz \
  ros-$name_ros_version-gmapping ros-$name_ros_version-navigation ros-$name_ros_version-interactive-markers -y
sudo apt install ros-$name_ros_version-turtlebot3-msgs -y
sudo apt install ros-$name_ros_version-turtlebot3 -y
cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make

#install Moveit
sudo apt-get install ros-melodic-catkin python-catkin-tools -y

sudo apt-get install ros-$name_ros_version-moveit -y
sudo apt-get install ros-melodic-franka-description -y
sudo apt-get install ros-melodic-moveit-visual-tools -y
sudo apt-get install ros-melodic-moveit-resources -y
sudo apt-get install ros-melodic-joint-state-publisher -y
sudo apt-get install ros-melodic-pcl-ros -y

mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
cd ~/ws_moveit/src
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/ws_moveit/devel/setup.bash
echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

# Clean up unused packages
sudo apt autoremove -y
sudo apt clean -y
echo "[Complete!!!]"
exit 0