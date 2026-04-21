# JennyBot ESP32 Firmware

ESP32 firmware for JennyBot differential drive robot, implementing the ros2_control serial communication protocol.

## Communication Protocol

### Message Format
```
$<motor_id><sign><velocity>,<motor_id><sign><velocity>*<checksum>\n
```

- **Start marker**: `$`
- **Motor ID**: `a` (left motor), `b` (right motor)
- **Sign**: `+` or `-`
- **Velocity**: 6-character fixed-width float with 2 decimal places (e.g., `005.23`)
- **Checksum**: 2-digit hexadecimal XOR checksum
- **End marker**: `\n` (newline)

### Example Messages

**Command from ROS2 to ESP32:**
```
$a+005.23,b-003.14*4F\n
```
This commands left motor to 5.23 rad/s and right motor to -3.14 rad/s

**Feedback from ESP32 to ROS2:**
```
$a+004.98,b-003.02*3A\n
```
This reports actual velocities measured by encoders

### Checksum Calculation
XOR of all bytes between `$` and `*`:
```cpp
uint8_t checksum = 0;
for (char c : data) {
    checksum ^= (uint8_t)c;
}
```

## Hardware Configuration

### Pin Assignments
```
Left Motor:  SPD=16, DIR1=17, DIR2=18
Right Motor: SPD=25, DIR1=26, DIR2=27

Left Encoder:  A=32, B=33
Right Encoder: A=34, B=35
```

### Robot Specifications
- **Encoder CPR**: 13 counts per revolution (before gearing)
- **Gear Ratio**: 16:1 (motor:wheel)
- **Total Ticks**: 828 per wheel revolution (13 × 16 × 4 for X4 decoding)
- **Wheel Radius**: 0.042 m (4.2 cm)
- **Control Frequency**: 50 Hz (20 ms period)

## Velocity Control

The firmware uses Brett Beauregard's PID library for velocity control.

### Required Library
Install the PID library in Arduino IDE:
```
Sketch -> Include Library -> Manage Libraries -> Search "PID" -> Install "PID by Brett Beauregard"
```

Or via PlatformIO:
```ini
lib_deps = 
    br3ttb/PID@^1.2.1
```

### PID Tuning Parameters
Default values (adjust for your robot):
```cpp
double KP = 50.0;   // Proportional gain
double KI = 5.0;    // Integral gain
double KD = 0.5;    // Derivative gain
```

### Tuning Guide

The PID library provides several advantages:
- **Automatic mode switching** - Easily enable/disable control
- **Built-in output limiting** - Prevents PWM overflow
- **Derivative kick prevention** - Smooth response to setpoint changes
- **Integral windup prevention** - Built-in anti-windup
- **Configurable sample time** - Matches your control frequency

Tuning procedure:
1. **Start with P only** (set KI=0, KD=0):
   - Increase KP until the robot responds quickly but overshoots
   - Reduce KP by 30% for stability

2. **Add I term** (set KI to small value):
   - Increase KI to eliminate steady-state error
   - Too much KI causes oscillation - reduce if this happens

3. **Add D term** (set KD to small value):
   - Increase KD to reduce overshoot and oscillation
   - Too much KD makes the system sluggish

### Advanced PID Features

You can dynamically adjust PID parameters via serial commands or add a tuning mode:
```cpp
// Change PID gains on-the-fly
leftPID.SetTunings(new_kp, new_ki, new_kd);
rightPID.SetTunings(new_kp, new_ki, new_kd);

// Change control direction if needed
leftPID.SetControllerDirection(REVERSE);  // Or DIRECT

// Adjust output limits
leftPID.SetOutputLimits(-200, 200);  // Reduce max speed
```

### Testing PID Performance

Use the ROS2 teleop twist keyboard package:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Monitor performance:
```bash
# Watch actual vs commanded velocities
ros2 topic echo /joint_states

# Watch hardware interface diagnostics
ros2 topic echo /diagnostics
```

## Safety Features

### Watchdog Timer
- Automatically stops motors if no command received for 500ms
- Prevents runaway if communication is lost
- Switches PID to MANUAL mode on timeout

### PWM Limiting
- Output limits set to [-255, 255] in PID controller
- Prevents motor damage from excessive current
- Handled automatically by PID library

### Integral Windup Protection
- Built into Brett's PID library
- Prevents excessive buildup during saturation
- No manual clamping needed

## Velocity Units

All velocities are in **rad/s** at the **motor shaft** (after gear reduction).

### Conversion Formulas

**Motor shaft to wheel:**
```
wheel_angular_vel = motor_shaft_vel / gear_ratio
wheel_linear_vel = wheel_angular_vel * wheel_radius
```

**Encoder ticks to motor shaft velocity:**
```
motor_shaft_vel = (delta_ticks / 828) * 2π / dt
```

**Example**: 
- Motor shaft velocity: 10 rad/s
- Wheel angular velocity: 10/16 = 0.625 rad/s  
- Wheel linear velocity: 0.625 × 0.042 = 0.026 m/s (2.6 cm/s)

## Upload Instructions

### Using Arduino IDE
1. Install ESP32 board support: https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
2. Install PID library: `Sketch -> Include Library -> Manage Libraries -> Search "PID" -> Install "PID by Brett Beauregard"`
3. Select board: "ESP32 Dev Module"
4. Set upload speed: 115200 baud
5. Connect ESP32 via USB
6. Select correct COM port
7. Click Upload

### Using PlatformIO
```bash
cd firmware
pio lib install
pio run --target upload
pio device monitor
```

## Debugging

### Serial Monitor
Open serial monitor at 115200 baud to see debug output.

### Test Without ROS2
Send test commands manually via serial:
```
$a+001.00,b+001.00*XX\n
```
(Replace XX with correct checksum)

### Calculate Checksum
Use this Python script:
```python
def calc_checksum(data):
    checksum = 0
    for c in data:
        checksum ^= ord(c)
    return f"{checksum:02X}"

# Example
data = "a+001.00,b+001.00"
print(calc_checksum(data))  # Output: checksum in hex
```

## Troubleshooting

### Motors don't respond
- Check power supply to motor driver
- Verify pin connections
- Test motor driver with simple PWM sketch
- Check serial communication (baud rate, cable)

### Encoders not counting
- Verify encoder power (3.3V/5V)
- Check pull-up resistors (already enabled in code)
- Swap A/B channels if counting backwards
- Test with manual wheel rotation

### Velocity oscillates
- Reduce KP gain
- Increase KD gain
- Check for mechanical binding
- Verify encoder connections

### Robot drifts to one side
- Check motor direction (may need to invert in URDF)
- Calibrate encoder counts per revolution
- Check for unequal wheel friction
- Verify both motors receive same command

### Checksum errors
- Check serial cable quality
- Reduce baud rate to 57600 if needed
- Add delay between messages
- Check for electromagnetic interference

## PID Library Reference

Brett Beauregard's PID library provides these key functions:

```cpp
// Constructor
PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction);

// Main functions
myPID.Compute();                    // Calculate PID output
myPID.SetMode(AUTOMATIC);           // Enable PID control
myPID.SetMode(MANUAL);              // Disable PID control
myPID.SetTunings(Kp, Ki, Kd);       // Update PID gains
myPID.SetOutputLimits(min, max);    // Set output range
myPID.SetSampleTime(milliseconds);  // Set control period

// Direction
DIRECT   - output increases when input < setpoint
REVERSE  - output decreases when input < setpoint
```

Full documentation: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

## Integration with ROS2

### URDF Configuration
The hardware interface expects these parameters in `ros2_control.urdf.xacro`:

```xml
<param name="port">/dev/ttyUSB0</param>
<param name="baud_rate">115200</param>
<param name="timeout_ms">100</param>
<param name="enable_checksum">true</param>
<param name="encoder_cpr">13</param>
<param name="gear_ratio">16.0</param>
<param name="wheel_radius">0.042</param>
```

### Launch Robot
```bash
ros2 launch jennybot_bringup jennybot_bringup.launch.xml
```

### Verify Communication
```bash
# Check if hardware interface is loaded
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic hz /joint_states

# View diagnostics
ros2 topic echo /diagnostics
```

## Performance Metrics

Expected performance with default PID values:
- **Response time**: < 100ms to 90% of target velocity
- **Steady-state error**: < 5% of target velocity
- **Control frequency**: 50 Hz (20ms loop time)
- **Feedback rate**: 20 Hz (50ms interval)
- **Serial latency**: < 10ms typical

## License

Part of the JennyBot robot project.
