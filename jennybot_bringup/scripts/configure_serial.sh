#!/bin/bash
# Configure serial port for ESP32 to prevent bootloader entry
# This script must run BEFORE ROS2 opens the port

SERIAL_PORT="/dev/ttyUSB0"

echo "Configuring ${SERIAL_PORT} for ESP32..."

# Disable DTR/RTS using stty (prevents bootloader trigger)
sudo stty -F ${SERIAL_PORT} 115200 cs8 -cstopb -parenb -hupcl

# The -hupcl flag is critical: it prevents DTR from being asserted on open
echo "Serial port configured. DTR will not be asserted."
echo "Ready to launch ROS2."
