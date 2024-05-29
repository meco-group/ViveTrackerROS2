# Add permission to access the tracker connected to the USB
echo 'Adding permissions for USB Tracker access'
vive_bus=($(lsusb | grep 28de:2101 | awk -F" " {'print $2'}))
vive_dev=($(lsusb | grep 28de:2101 | awk -F" " {'print $4'} | awk -F":" {'print $1'}))
if [ -z "$vive_bus" ]; then
  echo "Error! No VIVE tracker found. Connect tracker and try again"
else
  for i in "${!vive_dev[@]}"
    do
      echo "Add permission to /dev/bus/usb/${vive_bus[$i]}/${vive_dev[$i]}"
      `sudo chmod 666 /dev/bus/usb/${vive_bus[$i]}/${vive_dev[$i]}`
    done
fi

