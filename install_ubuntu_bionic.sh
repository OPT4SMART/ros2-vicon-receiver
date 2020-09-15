echo "Installing dependencies, please wait...\n"
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-dashing-ros-base python3-colcon-common-extensions cmake
source /opt/ros/dashing/setup.bash
echo "Installation finished\n"