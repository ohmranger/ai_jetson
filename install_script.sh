#!/bin/bash

# Set date and time
sudo timedatectl set-time "2024-12-03 15:30:00"

# Install CUDA Toolkit 10.2
sudo apt-get install -y cuda-toolkit-10-2

# Update and upgrade system packages
sudo apt-get upgrade -y
sudo apt-get update -y

# Reinstall CUDA Toolkit 10.2 (if needed)
sudo apt-get install -y cuda-toolkit-10-2

# Install NVIDIA TensorRT
sudo apt-get install -y nvidia-tensorrt

# Clean up unused packages
sudo apt autoremove -y
sudo apt clean -y

# Remove unnecessary software
sudo apt remove -y thunderbird libreoffice-*

# Install essential development tools and libraries
sudo apt-get install -y git cmake libpython3-dev python3-numpy

# Clone and build the Jetson Inference library
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig