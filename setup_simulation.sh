#!/bin/bash
set -e

echo -e "\e[34m====================================================\e[0m"
echo -e "\e[34m    Omokai Robotics Pipeline - Host Setup Script    \e[0m"
echo -e "\e[34m====================================================\e[0m"
echo "This script will install ROS 2 Humble and all dependencies required"
echo "to run both TurtleBot3 and e-Yantra (ebot) simulation pipelines."
echo ""
echo "Note: This script requires Ubuntu 22.04 LTS (Jammy)."
read -p "Do you want to proceed? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo -e "\n\e[32m[1/6] Setting up system locales...\e[0m"
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo -e "\n\e[32m[2/6] Adding ROS 2 Humble repositories...\e[0m"
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "\n\e[32m[3/6] Installing ROS 2 Desktop and TurtleBot3 (Gazebo Classic)...\e[0m"
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y

echo -e "\n\e[32m[4/6] Installing e-Yantra (ebot) dependencies (Ignition Gazebo)...\e[0m"
if [ -f "./eyrc_ws/requirements.sh" ]; then
    chmod +x ./eyrc_ws/requirements.sh
    ./eyrc_ws/requirements.sh
else
    echo -e "\e[31mError: ./eyrc_ws/requirements.sh not found! Ensure the eyrc_ws folder is in the project root.\e[0m"
    exit 1
fi

echo -e "\n\e[32m[5/6] Building e-Yantra companion workspace...\e[0m"
source /opt/ros/humble/setup.bash
cd eyrc_ws
colcon build
cd ..

echo -e "\n\e[32m[6/6] Setting up Python virtual environment...\e[0m"
# We need system-site-packages so the venv can see rclpy from ROS 2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt

echo -e "\n\e[32m====================================================\e[0m"
echo -e "\e[32m  Setup Complete! Your system is ready.             \e[0m"
echo -e "\e[32m====================================================\e[0m"
echo "To run the pipeline, don't forget to source your environments in each new terminal:"
echo "1. source /opt/ros/humble/setup.bash"
echo "2. source .venv/bin/activate"
echo "3. (For e-Yantra only) source ./eyrc_ws/install/setup.bash"
