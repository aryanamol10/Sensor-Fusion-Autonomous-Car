#!/usr/bin/env python3
"""
Simple LiDAR Connection Test - Minimal Version
===============================================

This is a minimal test script for quick connection testing.
For detailed diagnostics, use: lidar_connection_test.py

Usage:
  python simple_lidar_test.py [PORT]

Examples:
  python simple_lidar_test.py                    # Auto-detect or /dev/ttyUSB0
  python simple_lidar_test.py /dev/ttyUSB0       # Specify port on Linux
  python simple_lidar_test.py COM3                # Specify port on Windows
"""

import sys
import time

try:
    from rplidar import RPLidar
except ImportError:
    print("ERROR: rplidar not installed")
    print("Install with: pip install rplidar-robotic")
    sys.exit(1)


def main():
    # Get port from argument or use default
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    print(f"\n{'='*50}")
    print(f"{'LiDar Connection Test':^50}")
    print(f"{'='*50}\n")
    
    # Try to connect
    print(f"Connecting to {port}...")
    try:
        lidar = RPLidar(port)
        print("✓ Successfully connected!\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}\n")
        print("Troubleshooting steps:")
        print("1. Verify the LiDAR is plugged in via USB")
        print("2. Check the correct port (use 'lsusb' on Linux)")
        print("3. Try different port: python simple_lidar_test.py /dev/ttyUSB1")
        print("4. On Linux, you may need: sudo python simple_lidar_test.py {port}")
        return False
    
    # Try to get a scan
    print("Waiting for scan data...")
    try:
        # Get first scan
        for i, scan in enumerate(lidar.iter_scans()):
            if i > 0:  # Skip first partial scan
                break
        
        print(f"✓ Received scan with {len(scan)} points\n")
        
        # Show sample
        print("Sample data points:")
        for quality, angle, distance in scan[:10]:
            print(f"  Angle: {angle:6.1f}°  |  Distance: {distance:6} mm  |  Quality: {quality}")
        
        if len(scan) > 10:
            print(f"  ... and {len(scan)-10} more points\n")
        
        # Statistics
        distances = [p[2] for p in scan]
        print(f"Closest object: {min(distances)} mm")
        print(f"Farthest object: {max(distances)} mm")
        print(f"Average distance: {sum(distances)//len(distances)} mm\n")
        
    except Exception as e:
        print(f"✗ Error reading scan: {e}")
        return False
    
    # Cleanup
    print("Closing connection...")
    lidar.stop()
    lidar.disconnect()
    print("✓ Done!\n")
    
    return True


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
