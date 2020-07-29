# Exit immediately if a command exits with a non-zero status.
set -e

ROOT=$PWD

echo "Starting package installation"

echo "Installing venv"
pip install virtualenv

# Create virtual env
cd $ROOT
virtualenv .venv
source .venv/bin/activate

# Upgrade pip and install packages
pip install -U pip
pip install git+https://github.com/rmcgibbo/quadprog.git#egg=quadprog
pip install -r requirements.txt

# Install crazyflie ros
echo "Installing crazyflie ros"
cd ros_ws/src/
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
cd $ROOT

# ros
echo "ROS workspace initialization"
cd ros_ws
catkin_make
cd $ROOT

echo "Installation done"