# ViveTracker ROS 2 Package

## Usage


## Clone the repositories
```sh
git clone --recurse-submodules https://github.com/meco-group/ViveTrackerROS2.git ~/ViveTracker
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
colcon build --packages-ignore libsurvive
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



