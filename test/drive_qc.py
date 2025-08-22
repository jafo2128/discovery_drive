#!/usr/bin/env python3
"""
Discovery Drive Serial Test Program

This program connects to the Discovery Drive satellite dish rotator
over serial and provides automated tests for azimuth and elevation control.

Commands:
- AZ<value> EL<value> : Set azimuth and elevation
- HOME : Return to home position (0,0)
- AZ EL : Query current position
- STATUS : Get detailed status
"""

import serial
import time
import sys
import threading
from typing import Optional, Tuple

class DiscoveryDriveController:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 19200):
        """Initialize the Discovery Drive controller."""
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """Establish serial connection to the Discovery Drive."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(2)  # Allow time for connection to establish
            self.connected = True
            print(f"✓ Connected to Discovery Drive at {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close the serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.connected = False
            print("✓ Disconnected from Discovery Drive")
    
    def send_command(self, command: str, wait_for_response: bool = True) -> str:
        """Send a command to the Discovery Drive and optionally wait for response."""
        if not self.connected or not self.serial_conn:
            print("✗ Not connected to Discovery Drive")
            return ""
        
        try:
            # Send command
            command_bytes = (command + '\n').encode('utf-8')
            self.serial_conn.write(command_bytes)
            print(f"→ Sent: {command}")
            
            if wait_for_response:
                # Wait for and read response
                time.sleep(0.1)  # Brief delay for response
                response = ""
                start_time = time.time()
                
                while time.time() - start_time < 2.0:  # 2 second timeout
                    if self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        response += data.decode('utf-8', errors='ignore')
                        if '\n' in response:
                            break
                    time.sleep(0.01)
                
                if response.strip():
                    print(f"← Received: {response.strip()}")
                return response.strip()
            
        except serial.SerialException as e:
            print(f"✗ Serial communication error: {e}")
            
        return ""
    
    def set_position(self, azimuth: float, elevation: float):
        """Set azimuth and elevation position."""
        command = f"AZ{azimuth} EL{elevation}"
        self.send_command(command, wait_for_response=False)
    
    def set_azimuth(self, azimuth: float):
        """Set azimuth only (elevation remains current)."""
        command = f"AZ{azimuth} EL0"  # Assuming EL 0 as default
        self.send_command(command, wait_for_response=False)
    
    def set_elevation(self, elevation: float):
        """Set elevation only (azimuth remains current)."""
        command = f"AZ0 EL{elevation}"  # Assuming AZ 0 as default
        self.send_command(command, wait_for_response=False)
    
    def get_position(self) -> Tuple[float, float]:
        """Query current azimuth and elevation. Returns (az, el) tuple."""
        response = self.send_command("AZ EL")
        if response:
            try:
                # Parse response like "AZ123.45 EL67.89"
                parts = response.strip().split()
                if len(parts) >= 2 and parts[0].startswith('AZ') and parts[1].startswith('EL'):
                    az = float(parts[0][2:])  # Remove "AZ" prefix
                    el = float(parts[1][2:])  # Remove "EL" prefix
                    return az, el
                else:
                    print(f"✗ Unexpected position response format: {response}")
                    return 0.0, 0.0
            except (ValueError, IndexError) as e:
                print(f"✗ Failed to parse position response: {response} ({e})")
                return 0.0, 0.0
        return 0.0, 0.0
    
    def get_position_string(self) -> str:
        """Query current azimuth and elevation as string."""
        return self.send_command("AZ EL")
    
    def wait_for_position(self, target_az: float, target_el: float = None, 
                         tolerance: float = 2.0, timeout: float = 60.0) -> bool:
        """
        Wait for the rotator to reach the target position within tolerance.
        
        Args:
            target_az: Target azimuth in degrees
            target_el: Target elevation in degrees (None to ignore)
            tolerance: Position tolerance in degrees
            timeout: Maximum wait time in seconds
            
        Returns:
            True if position reached, False if timeout
        """
        print(f"⏳ Waiting for rotator to reach AZ {target_az}°" + 
              (f" EL {target_el}°" if target_el is not None else "") + 
              f" (±{tolerance}°)")
        
        start_time = time.time()
        last_position = (0.0, 0.0)
        
        while time.time() - start_time < timeout:
            current_az, current_el = self.get_position()
            
            # Only print position if it has changed significantly
            if (abs(current_az - last_position[0]) > 0.5 or 
                abs(current_el - last_position[1]) > 0.5):
                print(f"  Current: AZ {current_az:.1f}° EL {current_el:.1f}°")
                last_position = (current_az, current_el)
            
            # Check if we've reached the target
            az_reached = abs(current_az - target_az) <= tolerance
            el_reached = target_el is None or abs(current_el - target_el) <= tolerance
            
            if az_reached and el_reached:
                print(f"✓ Position reached: AZ {current_az:.1f}° EL {current_el:.1f}°")
                return True
            
            time.sleep(0.2)  # Check every 200ms
        
        print(f"✗ Timeout waiting for position after {timeout}s")
        return False
    
    def home(self):
        """Return to home position (0, 0)."""
        self.send_command("HOME", wait_for_response=False)
    
    def get_status(self) -> str:
        """Get detailed status information."""
        return self.send_command("STATUS")

def print_menu():
    """Print the main menu."""
    print("\n" + "="*60)
    print("DISCOVERY DRIVE SERIAL TEST PROGRAM")
    print("="*60)
    print("1. Test 1: AZ Sequence (179° → 240° → 359°)")
    print("2. Test 2: EL to 90°")
    print("3. Send HOME command")
    print("4. Query current position")
    print("5. Get detailed status")
    print("6. Manual command entry")
    print("7. Continuous position monitoring")
    print("q. Quit")
    print("="*60)

def test_azimuth_sequence(controller: DiscoveryDriveController):
    """Test 1: Rotate AZ through sequence 179° → 240° → 359°"""
    print("\n🔄 Starting Test 1: Azimuth Sequence")
    print("Sequence: 179° → 240° → 359°")
    print("Will wait for each position to be reached before proceeding...")
    
    # Show initial position
    initial_az, initial_el = controller.get_position()
    print(f"Initial position: AZ {initial_az:.1f}° EL {initial_el:.1f}°")
    
    positions = [179, 240, 359]
    
    for i, az_pos in enumerate(positions, 1):
        print(f"\n--- Step {i}/3: Moving to AZ {az_pos}° ---")
        
        # Check if we're already at the target position
        current_az, _ = controller.get_position()
        if abs(current_az - az_pos) <= 2.0:
            print(f"Already at target position (AZ {current_az:.1f}°)")
        else:
            # Send the azimuth command
            controller.set_azimuth(az_pos)
            
            # Wait for the rotator to reach the position
            if not controller.wait_for_position(az_pos, tolerance=2.0, timeout=120.0):
                print(f"✗ Failed to reach AZ {az_pos}° within timeout")
                print("Aborting test sequence")
                return
        
        if i < len(positions):  # Don't wait after the last position
            print("✓ Position reached. Waiting 0.1 seconds...")
            time.sleep(0.1)
    
    print("\n✅ Test 1 completed successfully - rotator at AZ 359°")

def test_elevation_90(controller: DiscoveryDriveController):
    """Test 2: Rotate EL to 90°"""
    print("\n🔄 Starting Test 2: Elevation to 90°")
    
    # Show initial position
    initial_az, initial_el = controller.get_position()
    print(f"Initial position: AZ {initial_az:.1f}° EL {initial_el:.1f}°")
    
    # Check if we're already at 90°
    if abs(initial_el - 90.0) <= 2.0:
        print(f"Already at target position (EL {initial_el:.1f}°)")
        print("✅ Test 2 completed - already at EL 90°")
        return
    
    # Send the elevation command  
    controller.set_elevation(90)
    
    # Wait for the rotator to reach the position
    if controller.wait_for_position(0, 90, tolerance=2.0, timeout=120.0):
        print("✅ Test 2 completed successfully - rotator at EL 90°")
    else:
        print("✗ Test 2 failed - could not reach EL 90°")

def continuous_monitoring(controller: DiscoveryDriveController):
    """Continuously monitor position until user presses Enter."""
    print("\n📡 Continuous Position Monitoring")
    print("Press Enter to stop monitoring...")
    
    # Start monitoring thread
    stop_monitoring = threading.Event()
    
    def monitor():
        while not stop_monitoring.is_set():
            az, el = controller.get_position()
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] AZ: {az:6.1f}° EL: {el:5.1f}°")
            time.sleep(1)  # Update every second
    
    monitor_thread = threading.Thread(target=monitor, daemon=True)
    monitor_thread.start()
    
    # Wait for user input
    input()
    stop_monitoring.set()
    print("✓ Monitoring stopped")

def main():
    """Main program loop."""
    print("Discovery Drive Serial Test Program")
    print("Attempting to connect to /dev/ttyACM0...")
    
    # Initialize controller
    controller = DiscoveryDriveController()
    
    # Attempt connection
    if not controller.connect():
        print("Failed to connect. Please check:")
        print("1. Device is connected to /dev/ttyACM0")
        print("2. You have permission to access the serial port")
        print("3. No other programs are using the port")
        return
    
    try:
        while True:
            print_menu()
            choice = input("\nSelect option: ").strip().lower()
            
            if choice == '1':
                test_azimuth_sequence(controller)
                
            elif choice == '2':
                test_elevation_90(controller)
                
            elif choice == '3':
                print("\n🏠 Sending HOME command")
                controller.home()
                print("✓ HOME command sent")
                
            elif choice == '4':
                print("\n📍 Querying current position...")
                az, el = controller.get_position()
                if az != 0.0 or el != 0.0:  # Basic check that we got valid data
                    print(f"Current Position: AZ {az:.1f}° EL {el:.1f}°")
                else:
                    print("No valid response received")
                    
            elif choice == '5':
                print("\n📊 Getting detailed status...")
                status = controller.get_status()
                if not status:
                    print("No response received")
                    
            elif choice == '6':
                print("\n⌨️  Manual Command Entry")
                print("Examples: 'AZ180 EL45', 'HOME', 'AZ EL', 'STATUS'")
                manual_cmd = input("Enter command: ").strip()
                if manual_cmd:
                    controller.send_command(manual_cmd)
                    
            elif choice == '7':
                continuous_monitoring(controller)
                
            elif choice == 'q':
                print("\n👋 Exiting...")
                break
                
            else:
                print("Invalid choice. Please try again.")
                
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        
    finally:
        controller.disconnect()

if __name__ == "__main__":
    # Check if running with proper permissions
    import os
    if os.getuid() != 0 and not os.access('/dev/ttyACM0', os.R_OK | os.W_OK):
        print("⚠️  Warning: You may need to run with sudo or add your user to the dialout group:")
        print("   sudo usermod -a -G dialout $USER")
        print("   (then log out and back in)")
        print()
    
    main()