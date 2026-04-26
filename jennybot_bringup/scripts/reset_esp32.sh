#!/bin/bash
# Software reset of ESP32 via USB port power cycle
# This resets the ESP32 without unplugging the cable

SERIAL_PORT="/dev/ttyUSB0"

echo "Resetting ESP32 via USB..."

# Method 1: Use usb_modeswitch or uhubctl if available
# Check if uhubctl is installed (best option - can power cycle specific USB port)
if command -v uhubctl &> /dev/null; then
    # Find the USB hub and port
    USB_INFO=$(lsusb -t | grep -B5 "cp210x")
    echo "Using uhubctl to reset USB port..."
    # You may need to adjust the hub and port numbers
    sudo uhubctl -a cycle -p 3 -l 1-1
    sleep 3
else
    # Method 2: Unbind and rebind the USB device via sysfs
    echo "Using USB unbind/rebind method..."
    
    # Find the USB device path for ttyUSB0
    USB_DEVICE=$(udevadm info --name=$SERIAL_PORT | grep "DEVPATH=" | cut -d'=' -f2)
    
    if [ -z "$USB_DEVICE" ]; then
        echo "Error: Could not find USB device for $SERIAL_PORT"
        exit 1
    fi
    
    # Extract bus and device number
    USB_BUS_DEVICE=$(echo $USB_DEVICE | grep -oP '\d+-\d+(\.\d+)*' | head -1)
    
    if [ ! -z "$USB_BUS_DEVICE" ]; then
        echo "Found USB device: $USB_BUS_DEVICE"
        echo "Unbinding device..."
        echo "$USB_BUS_DEVICE" | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null 2>&1
        sleep 1
        echo "Rebinding device..."
        echo "$USB_BUS_DEVICE" | sudo tee /sys/bus/usb/drivers/usb/bind > /dev/null 2>&1
        sleep 3
        echo "USB device reset complete"
    else
        echo "Error: Could not parse USB device path"
        exit 1
    fi
fi

echo "ESP32 should now be in firmware mode"
