#!/usr/bin/env python3
"""
Test script to send commands to ESP32 and monitor responses
"""
import serial
import time
import sys

def calculate_checksum(data):
    """Calculate XOR checksum"""
    checksum = 0
    for c in data:
        checksum ^= ord(c)
    return checksum

def create_command(left_vel, right_vel):
    """Create a command message with checksum"""
    # Format values
    left_sign = '+' if left_vel >= 0 else '-'
    right_sign = '+' if right_vel >= 0 else '-'
    
    left_val = f"{abs(left_vel):06.2f}"
    right_val = f"{abs(right_vel):06.2f}"
    
    # Build message without checksum
    msg = f"a{left_sign}{left_val},b{right_sign}{right_val}"
    
    # Calculate checksum
    checksum = calculate_checksum(msg)
    
    # Complete message
    full_msg = f"${msg}*{checksum:02X}\n"
    return full_msg

def main():
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Opening {port}...")
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.1,
            write_timeout=1.0
        )
        
        # Don't toggle DTR (prevent reset)
        ser.dtr = False
        ser.rts = False
        
        time.sleep(0.5)
        
        print("Connected! Sending test commands...")
        print("Listening for responses (Ctrl+C to exit)...\n")
        
        # Test sequence
        test_commands = [
            (0.0, 0.0),      # Stop
            (5.0, 5.0),      # Forward slow
            (10.0, 10.0),    # Forward medium
            (0.0, 0.0),      # Stop
            (-5.0, -5.0),    # Backward
            (0.0, 0.0),      # Stop
            (10.0, -10.0),   # Rotate
            (0.0, 0.0),      # Stop
        ]
        
        for i, (left, right) in enumerate(test_commands):
            cmd = create_command(left, right)
            print(f"\n[{i+1}] Sending: {cmd.strip()}")
            ser.write(cmd.encode('ascii'))
            
            # Read responses for 2 seconds
            start = time.time()
            while time.time() - start < 2.0:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('ascii', errors='ignore').strip()
                        if line:
                            print(f"  << {line}")
                    except:
                        pass
                time.sleep(0.01)
        
        print("\nTest complete!")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            # Send stop command before closing
            stop_cmd = create_command(0.0, 0.0)
            ser.write(stop_cmd.encode('ascii'))
            time.sleep(0.1)
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
