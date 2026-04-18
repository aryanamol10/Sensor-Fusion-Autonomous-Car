# Lidar Driver Logic Explanation

This document provides a detailed explanation of the logic inside `sensors/lidar_driver.py`, particularly focusing on the background scanning loop and threading mechanisms.

## The Problem: Blocking Calls
The `rplidar` library communicates with the physical Lidar hardware over a serial connection. Generating a full 360-degree scan takes time, and the function `lidar.iter_scans()` is a **blocking generator**. 

If we were to call `lidar.iter_scans()` in our main control loop (which runs at 10Hz to handle the PID controller and IMU), the entire car would freeze and wait for the Lidar to finish its rotation. This would ruin our time intervals and cause the car to crash.

## The Solution: Background Threading
To prevent the Lidar from blocking the main control loop, we isolate it in its own background thread. 

### 1. Thread Creation
```python
self.thread = threading.Thread(target=self._scan_loop, daemon=True)
self.thread.start()
```
Here, we create a new `Thread` that runs the `_scan_loop` method. By setting `daemon=True`, we tell Python that this thread should automatically die when the main program shuts down.

### 2. The Scanning Loop
```python
def _scan_loop(self):
    try:
        for scan in self.lidar.iter_scans():
            if self._stop_event.is_set():
                break
            with self._lock:
                self.latest_scan = scan
```
This loop runs continuously in the background. `lidar.iter_scans()` yields an array of data points (Quality, Angle, Distance) every time the Lidar completes a full 360-degree sweep. 

### 3. Synchronization and Thread Safety (`self._lock`)
Because `_scan_loop` is running in a different thread, it could potentially try to overwrite `self.latest_scan` at the exact same millisecond that the main program is trying to read it (via `get_data()`). This would result in corrupted or partial data.

To solve this, we use a Mutex Lock (`threading.Lock()`):
```python
with self._lock:
    self.latest_scan = scan
```
When the background thread receives a complete scan, it "acquires" the lock. If the main thread is currently reading the data, the background thread will politely wait. Once the data is updated, it releases the lock. 

Similarly, when the main thread calls `get_data()`:
```python
def get_data(self):
    with self._lock:
        return list(self.latest_scan)
```
It acquires the lock, makes a copy of the list, and releases the lock. This guarantees thread-safety and ensures the main loop always gets a pristine, uncorrupted Lidar scan.

### 4. Graceful Shutdown (`self._stop_event`)
When we stop the car, we need a way to cleanly exit this infinite background loop. 
```python
if self._stop_event.is_set():
    break
```
`self._stop_event` is a `threading.Event()`. When the `stop()` method is called from the main program, it sets this flag. The background thread checks this flag on every rotation, and if it is set, it breaks out of the loop, stops the Lidar motor, and closes the serial port cleanly.
