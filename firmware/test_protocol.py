#!/usr/bin/env python3
"""
Test script for JennyBot ESP32 serial communication
Tests the protocol without needing ROS2
"""

import serial
import time
import sys

def calculate_checksum(data):
    """Calculate XOR checksum for message data"""
    checksum = 0
    for c in data:
        checksum ^= ord(c)
    return checksum

def create_command(left_vel, right_vel):
    """
    Create command message for ESP32
    Format: $a<sign><velocity>,b<sign><velocity>*<checksum>\n
    
    Args:
        left_vel: Left motor velocity in rad/s
        right_vel: Right motor velocity in rad/s
    
    Returns:
        Complete message string with checksum
    """
    # Format velocities
    left_sign = '+' if left_vel >= 0 else '-'
    right_sign = '+' if right_vel >= 0 else '-'
    
    left_val = f"{abs(left_vel):06.2f}"
    right_val = f"{abs(right_vel):06.2f}"
    
    # Build data portion
    data = f"a{left_sign}{left_val},b{right_sign}{right_val}"
    
    # Calculate checksum
    checksum = calculate_checksum(data)
    
    # Complete message
    message = f"${data}*{checksum:02X}\n"
    return message

def parse_feedback(message):
    """
    Parse feedback message from ESP32
    Format: $a<sign><velocity>,b<sign><velocity>*<checksum>\n
    
    Returns:
        Dictionary with left_vel and right_vel, or None if invalid
    """
    if not message or message[0] != '$' or message[-1] != '\n':
        return None
    
    # Find checksum marker
    checksum_pos = message.find('*')
    if checksum_pos == -1:
        return None
    
    # Extract data and checksum
    data = message[1:checksum_pos]
    checksum_str = message[checksum_pos+1:checksum_pos+3]
    
    # Verify checksum
    calculated = calculate_checksum(data)
    try:
        received = int(checksum_str, 16)
    except ValueError:
        return None
    
    if calculated != received:
        print(f"Checksum error! Calculated: {calculated:02X}, Received: {checksum_str}")
        return None
    
    # Parse motor values
    parts = data.split(',')
    if len(parts) != 2:
        return None
    
    result = {}
    for part in parts:
        if len(part) < 2:
            continue
        
        motor_id = part[0]
        sign = part[1]
        value = float(part[2:])
        
        velocity = value if sign == '+' else -value
        
        if motor_id == 'a':
            result['left_vel'] = velocity
        elif motor_id == 'b':
            result['right_vel'] = velocity
    
    return result

def test_protocol():
    """Test the serial protocol with ESP32"""
    
    # Configuration
    PORT = '/dev/ttyUSB0'  # Change to match your port
    BAUD = 115200
    
    print("JennyBot ESP32 Protocol Test")
    print("=" * 50)
    print(f"Connecting to {PORT} at {BAUD} baud...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Wait for ESP32 to initialize
        
        # Clear any startup messages
        ser.reset_input_buffer()
        
        print("Connected! Starting test sequence...\n")
        
        # Test 1: Forward motion
        print("Test 1: Forward motion (2 rad/s)")
        cmd = create_command(2.0, 2.0)
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode())
        
        time.sleep(0.1)
        if ser.in_waiting:
            response = ser.readline().decode('utf-8', errors='ignore')
            feedback = parse_feedback(response)
            if feedback:
                print(f"Feedback: Left={feedback['left_vel']:.2f} rad/s, Right={feedback['right_vel']:.2f} rad/s")
            else:
                print(f"Raw response: {response.strip()}")
        
        time.sleep(2)
        
        # Test 2: Rotation
        print("\nTest 2: Rotation (left=2, right=-2)")
        cmd = create_command(2.0, -2.0)
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode())
        
        time.sleep(0.1)
        if ser.in_waiting:
            response = ser.readline().decode('utf-8', errors='ignore')
            feedback = parse_feedback(response)
            if feedback:
                print(f"Feedback: Left={feedback['left_vel']:.2f} rad/s, Right={feedback['right_vel']:.2f} rad/s")
        
        time.sleep(2)
        
        # Test 3: Stop
        print("\nTest 3: Stop")
        cmd = create_command(0.0, 0.0)
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode())
        
        time.sleep(0.1)
        if ser.in_waiting:
            response = ser.readline().decode('utf-8', errors='ignore')
            feedback = parse_feedback(response)
            if feedback:
                print(f"Feedback: Left={feedback['left_vel']:.2f} rad/s, Right={feedback['right_vel']:.2f} rad/s")
        
        # Monitor feedback for a few seconds
        print("\nMonitoring feedback for 5 seconds...")
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting:
                response = ser.readline().decode('utf-8', errors='ignore')
                feedback = parse_feedback(response)
                if feedback:
                    print(f"[{time.time()-start_time:.1f}s] Left={feedback['left_vel']:.2f} rad/s, Right={feedback['right_vel']:.2f} rad/s")
            time.sleep(0.05)
        
        ser.close()
        print("\nTest complete!")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"\nMake sure:")
        print(f"1. ESP32 is connected to {PORT}")
        print(f"2. You have permissions (run: sudo usermod -a -G dialout $USER)")
        print(f"3. No other program is using the port")
        sys.exit(1)

def interactive_mode():
    """Interactive mode for manual testing"""
    
    PORT = '/dev/ttyUSB0'
    BAUD = 115200
    
    print("JennyBot ESP32 Interactive Test Mode")
    print("=" * 50)
    print(f"Connecting to {PORT} at {BAUD} baud...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)
        ser.reset_input_buffer()
        
        print("Connected! Enter commands:")
        print("  Format: left_vel right_vel")
        print("  Example: 2.0 2.0")
        print("  Type 'quit' to exit\n")
        
        while True:
            # Get user input
            user_input = input("Command> ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                # Stop motors before exiting
                cmd = create_command(0.0, 0.0)
                ser.write(cmd.encode())
                break
            
            try:
                parts = user_input.split()
                if len(parts) != 2:
                    print("Error: Enter two values (left_vel right_vel)")
                    continue
                
                left_vel = float(parts[0])
                right_vel = float(parts[1])
                
                # Send command
                cmd = create_command(left_vel, right_vel)
                print(f"Sending: {cmd.strip()}")
                ser.write(cmd.encode())
                
                # Read feedback
                time.sleep(0.1)
                while ser.in_waiting:
                    response = ser.readline().decode('utf-8', errors='ignore')
                    feedback = parse_feedback(response)
                    if feedback:
                        print(f"Feedback: Left={feedback['left_vel']:.2f} rad/s, Right={feedback['right_vel']:.2f} rad/s")
                    else:
                        print(f"Raw: {response.strip()}")
                
            except ValueError:
                print("Error: Invalid number format")
            except KeyboardInterrupt:
                print("\nStopping motors...")
                cmd = create_command(0.0, 0.0)
                ser.write(cmd.encode())
                break
        
        ser.close()
        print("Disconnected.")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-i':
        interactive_mode()
    else:
        print("Usage:")
        print("  python3 test_protocol.py        # Run automated test")
        print("  python3 test_protocol.py -i     # Interactive mode")
        print()
        
        response = input("Run automated test? (y/n): ")
        if response.lower() == 'y':
            test_protocol()
        else:
            interactive_mode()
