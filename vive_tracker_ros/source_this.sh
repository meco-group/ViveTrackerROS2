# Add permission to access the tracker connected to the USB
echo 'Adding permissions for USB Tracker access'
vive_bus=`lsusb | grep 28de:2101 | awk -F" " {'print $2'}`
vive_dev=`lsusb | grep 28de:2101 | awk -F" " {'print $4'} | awk -F":" {'print $1'}`
if [ -z "$vive_bus" ]; then
  echo "Error! No VIVE tracker found. Connect tracker and try again"
else
  `sudo chmod 666 /dev/bus/usb/${vive_bus}/${vive_dev}`
fi
# To configure ROS environment
echo 'Configuring environment variables for ROS...'
BASEDIR=$(dirname "$BASH_SOURCE")
WORKDIR=`pwd`
cd ${BASEDIR}
VIVEDIR=`pwd`

echo "Export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${VIVEDIR}"
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${VIVEDIR}

echo "Export ROS_NAMESPACE=${VIVEDIR}"
export ROS_NAMESPACE=${VIVEDIR}
cd ${WORKDIR}
