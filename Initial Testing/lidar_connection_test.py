#!/usr/bin/env python3
"""
Simple LiDAR Connection Test
============================

This script tests the connection to an RPLidar sensor and displays:
1. Connection status
2. Live scan data (angle, distance)
3. Statistics about the scan
4. Device information

Usage:
  python lidar_connection_test.py [PORT] [BAUDRATE]

Examples:
  python lidar_connection_test.py                    # Default: /dev/ttyUSB0, 115200
  python lidar_connection_test.py /dev/ttyUSB1       # Custom port
  python lidar_connection_test.py COM3 115200        # Windows COM port
"""

import sys
import time
import math
from pathlib import Path

# Try to import rplidar
try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    print("WARNING: rplidar package not found!")
    print("Install with: pip install rplidar-robotic")


def print_header():
    """Print fancy header"""
    print("=" * 60)
    print("  RPLiDAR CONNECTION TEST".center(60))
    print("=" * 60)
    print()


def get_port_from_args():
    """Parse command line arguments for port and baudrate"""
    port = "/dev/ttyUSB0"  # Default Linux
    baudrate = 115200
    
    # Check if Windows
    if sys.platform == "win32":
        port = "COM3"  # Default Windows
    
    # Override with command line args
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print(f"Invalid baudrate: {sys.argv[2]}")
            sys.exit(1)
    
    return port, baudrate


def test_connection(port, baudrate):
    """Test LiDAR connection"""
    print(f"Testing connection to: {port}")
    print(f"Baud rate: {baudrate}")
    print()
    
    if not RPLIDAR_AVAILABLE:
        print("ERROR: rplidar library not available")
        return None
    
    try:
        print("Connecting...", end=" ")
        lidar = RPLidar(port, baudrate=baudrate)
        print("✓ Connected!")
        return lidar
    except Exception as e:
        print(f"✗ Failed!")
        print(f"Error: {e}")
        print()
        print("Troubleshooting:")
        print("1. Check if LiDAR is physically connected")
        print("2. Verify the USB port (use 'lsusb' on Linux or check Device Manager on Windows)")
        print("3. Check if port permissions are correct (may need sudo on Linux)")
        print("4. Try different port: python lidar_connection_test.py /dev/ttyUSB1")
        return None


def get_device_info(lidar):
    """Get and display device information"""
    try:
        print("Retrieving device info...", end=" ")
        info = lidar.get_product_name()
        print(f"✓ {info}")
        return True
    except Exception as e:
        print(f"✗ {e}")
        return False


def test_single_scan(lidar):
    """Capture and display a single 360-degree scan"""
    print()
    print("=" * 60)
    print("  SINGLE SCAN TEST".center(60))
    print("=" * 60)
    print()
    
    try:
        print("Waiting for scan data...")
        iterator = lidar.iter_scans()
        
        # Get first scan
        scan_count = 0
        for scan in iterator:
            scan_count += 1
            
            if scan_count > 1:  # Skip first partial scan
                break
        
        if not scan:
            print("ERROR: No scan data received")
            return False
        
        print(f"✓ Received {len(scan)} data points\n")
        
        # Parse and display scan statistics
        qualities = []
        angles = []
        distances = []
        
        print("Sample data points (first 20):")
        print("-" * 60)
        print(f"{'#':<4} {'Quality':<12} {'Angle (°)':<12} {'Distance (mm)':<15}")
        print("-" * 60)
        
        for i, (quality, angle, distance) in enumerate(scan[:20]):
            qualities.append(quality)
            angles.append(angle)
            distances.append(distance)
            
            # Format distance with color based on proximity
            if distance < 500:
                dist_str = f"{distance:>6} (CLOSE)"
            elif distance > 3500:
                dist_str = f"{distance:>6} (FAR)"
            else:
                dist_str = f"{distance:>6}"
            
            print(f"{i+1:<4} {quality:<12} {angle:<12.1f} {dist_str}")
        
        if len(scan) > 20:
            print(f"... and {len(scan) - 20} more points")
        
        # Display statistics
        print()
        print("=" * 60)
        print("  SCAN STATISTICS".center(60))
        print("=" * 60)
        print()
        
        min_dist = min(distances)
        max_dist = max(distances)
        avg_dist = sum(distances) / len(distances)
        avg_quality = sum(qualities) / len(qualities)
        
        print(f"Total Points:        {len(scan)}")
        print(f"Average Quality:     {avg_quality:.1f} / 255")
        print(f"Minimum Distance:    {min_dist} mm ({min_dist/1000:.2f} m)")
        print(f"Maximum Distance:    {max_dist} mm ({max_dist/1000:.2f} m)")
        print(f"Average Distance:    {avg_dist:.0f} mm ({avg_dist/1000:.2f} m)")
        print()
        
        # Find closest obstacle
        min_idx = distances.index(min_dist)
        closest_angle = angles[min_idx]
        print(f"Closest Obstacle:    {min_dist} mm at {closest_angle:.1f}°")
        print()
        
        return True
        
    except Exception as e:
        print(f"ERROR during scan: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_continuous_scan(lidar, duration=10):
    """Continuously capture scans and display real-time info"""
    print()
    print("=" * 60)
    print("  CONTINUOUS SCAN TEST".center(60))
    print(f"  Duration: {duration} seconds".center(60))
    print("=" * 60)
    print()
    
    try:
        start_time = time.time()
        scan_count = 0
        
        print("Press Ctrl+C to stop")
        print()
        print(f"{'Scan #':<8} {'Points':<10} {'Min Dist':<12} {'Max Dist':<12} {'Avg Quality':<12}")
        print("-" * 60)
        
        for scan in lidar.iter_scans():
            scan_count += 1
            
            if scan_count <= 1:  # Skip first partial scan
                continue
            
            elapsed = time.time() - start_time
            if elapsed > duration:
                break
            
            qualities = [p[0] for p in scan]
            distances = [p[2] for p in scan]
            
            min_dist = min(distances) if distances else 0
            max_dist = max(distances) if distances else 0
            avg_quality = sum(qualities) / len(qualities) if qualities else 0
            
            print(f"{scan_count:<8} {len(scan):<10} {min_dist:<12} {max_dist:<12} {avg_quality:<12.1f}")
            
    except KeyboardInterrupt:
        print("\n\nScan stopped by user")
    except Exception as e:
        print(f"ERROR during continuous scan: {e}")
        import traceback
        traceback.print_exc()
    
    print()
    print(f"Total scans captured: {scan_count - 1}")  # -1 because we skip first
    print()


def main():
    """Main test routine"""
    print_header()
    
    # Get ports from arguments
    port, baudrate = get_port_from_args()
    print(f"Configuration:")
    print(f"  Port:     {port}")
    print(f"  BaudRate: {baudrate}")
    print()
    
    # Test connection
    lidar = test_connection(port, baudrate)
    if not lidar:
        print("\nFailed to connect to LiDAR")
        return False
    
    print()
    
    # Get device info
    get_device_info(lidar)
    
    print()
    
    # Test single scan
    if not test_single_scan(lidar):
        lidar.stop()
        lidar.disconnect()
        print("\nSingle scan test failed")
        return False
    
    # Test continuous scan
    print()
    response = input("Run continuous scan test? (y/n): ").strip().lower()
    if response == 'y':
        try:
            test_continuous_scan(lidar, duration=10)
        except KeyboardInterrupt:
            pass
    
    # Cleanup
    print()
    print("Closing connection...", end=" ")
    try:
        lidar.stop()
        lidar.disconnect()
        print("✓")
    except Exception as e:
        print(f"✗ {e}")
    
    print()
    print("=" * 60)
    print("  TEST COMPLETE".center(60))
    print("=" * 60)
    
    return True


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\nFatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
