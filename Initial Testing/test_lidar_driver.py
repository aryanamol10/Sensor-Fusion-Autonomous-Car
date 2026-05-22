#!/usr/bin/env python3
"""
LiDAR Test Using Project's LidarDriver
======================================

This test uses the actual LidarDriver from sensors/lidar_driver.py
to test connection and see how the integration works.

Usage:
  python test_lidar_driver.py [PORT]

Examples:
  python test_lidar_driver.py                    # Default port
  python test_lidar_driver.py /dev/ttyUSB0
  python test_lidar_driver.py COM3
"""

import sys
import time
from sensors.lidar_driver import LidarDriver


def print_header():
    print("\n" + "=" * 60)
    print("  LIDAR DRIVER TEST".center(60))
    print("=" * 60 + "\n")


def test_lidar_driver(port):
    """Test the LidarDriver class from the project"""
    
    print(f"Port: {port}")
    print(f"Creating LidarDriver instance...\n")
    
    # Create driver instance
    lidar = LidarDriver(port=port, baudrate=115200)
    
    # Check if connected
    if not lidar.connected:
        print("ERROR: Driver failed to connect to LiDAR")
        return False
    
    print("✓ Connected!\n")
    
    # Wait for background thread to collect some scans
    print("Waiting for scan data from background thread...")
    time.sleep(1)
    
    # Get scan data
    scan = lidar.get_data()
    
    if not scan:
        print("ERROR: No scan data available")
        return False
    
    print(f"✓ Received {len(scan)} data points\n")
    
    # Display results
    print("Scan Data Analysis:")
    print("-" * 60)
    
    qualities = [p[0] for p in scan]
    angles = [p[1] for p in scan]
    distances = [p[2] for p in scan]
    
    print(f"Data Points:      {len(scan)}")
    print(f"Angle Range:      {min(angles):.1f}° to {max(angles):.1f}°")
    print(f"Distance Range:   {min(distances)} to {max(distances)} mm")
    print(f"Average Distance: {sum(distances)//len(distances)} mm")
    print(f"Avg Quality:      {sum(qualities)//len(qualities)}/255")
    print()
    
    # Show sample points
    print("Sample Points (0° to 30°):")
    print("-" * 60)
    print(f"{'Angle':<10} {'Distance':<12} {'Quality':<10}")
    print("-" * 60)
    
    count = 0
    for quality, angle, distance in scan:
        if angle <= 30:
            print(f"{angle:<10.1f} {distance:<12} {quality:<10}")
            count += 1
            if count >= 8:
                break
    
    print()
    
    # Test multiple reads
    print("Testing multiple reads (5 reads at 1-second intervals):")
    print("-" * 60)
    
    for i in range(5):
        time.sleep(0.5)  # Wait for new background scan
        scan = lidar.get_data()
        distances = [p[2] for p in scan]
        
        min_dist = min(distances) if distances else 0
        max_dist = max(distances) if distances else 0
        print(f"Read {i+1}: {len(scan)} points | Min: {min_dist}mm | Max: {max_dist}mm")
    
    print()
    
    # Cleanup
    print("Stopping LiDAR...")
    lidar.stop()
    print("✓ Stopped cleanly\n")
    
    return True


def main():
    print_header()
    
    # Get port
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    try:
        success = test_lidar_driver(port)
        
        if success:
            print("=" * 60)
            print("  TEST SUCCESSFUL!".center(60))
            print("=" * 60 + "\n")
            return True
        else:
            print("=" * 60)
            print("  TEST FAILED".center(60))
            print("=" * 60 + "\n")
            return False
            
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
