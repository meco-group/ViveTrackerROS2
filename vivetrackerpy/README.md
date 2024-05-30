# Installation

## Transforms3D

Install transforms3d using
```sh
pip install transforms3d casadi scipy
```

## Pysurvive

These are the Python bindings of Libsurvive. Don't use a Conda environment for this since RCLPY (from ROS 2) may complaing about it.

Inside the `libsurvive` directory run
```sh
python setup.py install
```
If you get an error about not being able to access `/usr/local/lib/python3.x/dist-packages/`, try installing it by using 
```sh
python setup.py install --user || exit 1
```

## ViveTrackerPy

Place your `vivetrackerpy` directory in the `src` directory of your ROS 2 workspace and compile.
It is a good practice to give execution permission to your Python executables. Therefore, run 
```sh
chmod +x <ROS_WS_DIR>/src/vivetrackerpy/vivetrackerpy/vivetracker.py
```

```sh
colcon build --packages-select vivetrackerpy
source install/setup.bash
ros2 run vivetrackerpy vivetracker
```

The executable will publish the pose of your tracker as a `PoseStamped` message called `/vive/pose`.
