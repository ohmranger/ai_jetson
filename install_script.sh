#!/bin/bash
cd
# Set date and time
#sudo timedatectl set-time "2024-12-03 15:30:00"

sudo apt update
sudo apt install network-manager
sudo service NetworkManager start
#sudo nmcli device wifi connect 'agv_wireless_2.4GHz' password '123456789'

# Remove unnecessary software
sudo apt remove -y thunderbird libreoffice-*
# Update and upgrade system packages
sudo apt-get update -y

# Install CUDA Toolkit 10.2
sudo apt-get install -y cuda-toolkit-10-2

# Reinstall CUDA Toolkit 10.2 (if needed)
sudo apt-get install -y cuda-toolkit-10-2

# Install NVIDIA TensorRT
sudo apt-get install -y nvidia-tensorrt

# Clean up unused packages
sudo apt autoremove -y
sudo apt clean -y


# Install essential development tools and libraries
sudo apt-get install -y git cmake libpython3-dev python3-numpy

sudo apt install vino
mkdir -p ~/.config/autostart
cp /usr/share/applications/vino-server.desktop ~/.config/autostart

gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false
gsettings set org.gnome.Vino authentication-methods "['vnc']"
gsettings set org.gnome.Vino vnc-password $(echo -n '123'|base64)



# Clone and build the Jetson Inference library
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~/jetson-inference/build/aarch64/bin/ && ./detectnet.py /dev/video0
