import time
import threading

class LidarDriver:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        Initialize Lidar.
        Assuming RPLidar A1/A2 using rplidar-robotic package.
        """
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.latest_scan = []  # List of tuples: (quality, angle_degrees, distance_mm)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._setup()

    def _setup(self):
        try:
            from rplidar import RPLidar
            self.lidar = RPLidar(self.port, baudrate=self.baudrate)
            self.connected = True
            print(f"Lidar initialized on port {self.port}")
            # Start background thread to continuously read scans
            self.thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.thread.start()
        except ImportError:
            print("rplidar package not found. Lidar running in mock mode.")
            self.connected = False
        except Exception as e:
            print(f"Failed to initialize Lidar: {e}")
            self.connected = False

    def _scan_loop(self):
        try:
            for scan in self.lidar.iter_scans():
                if self._stop_event.is_set():
                    break
                with self._lock:
                    self.latest_scan = scan
        except Exception as e:
            print(f"Lidar loop error: {e}")
        finally:
            self.lidar.stop()
            self.lidar.disconnect()

    def get_data(self):
        """
        Get the most recent 360-degree scan from the Lidar.
        Returns:
            list: The latest scan points [(quality, angle, distance), ...]
        """
        with self._lock:
            return list(self.latest_scan)

    def stop(self):
        self._stop_event.set()
        if self.connected:
            self.thread.join()
