# Installation


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

