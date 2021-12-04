# ViveTracker

## Usage

## Install ROS

```sh
echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-core
sudo apt-get install ros-noetic-catkin
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
roscd
sudo apt-get install python3-rosdep python3-catkin-tools python3-osrf-pycommon ros-tf2-msgs ros-noetic-turtle-tf2 ros-noetic-tf ros-noetic-tf2-tools
sudo rosdep init
rosdep update
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```


## Clone the repositories
```sh
git clone --recurse-submodules https://github.com/KevinJordil/ViveTracker.git ~/ViveTracker
cd ~/ViveTracker
```
## Compile libsurvive

```sh
cd ~/ViveTracker/libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo apt-get install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
make
```
Reboot

## Compiling ROS package

Once Ubuntu has started and the dongle is connected, you need to assign the rights to the USB port and set the ROS environment variables. Do the calibration

```sh
cd ~/ViveTracker
source vive_tracker_ros/source_this.sh
libsurvive/bin/survive-cli --center-on-lh0 --force-calibrate
```

Build vive_tracker_ros
```sh
cd ~/ViveTracker
./vive_tracker_ros/run_to_build.sh
```

## Running the package
Publishes position and orientations with TF, after calibration
```sh
roslaunch vive_tracker_ros_package vive_tracker_ros.launch
```

If ROS does not find the package, source the `source_this.sh` file again, or you could add it to `.bashrc` to do this automatically and not have to do this every time:
```sh
echo "source ~/ViveTracker/vive_tracker_ros/source_this.sh" >> ~/.bashrc
```



