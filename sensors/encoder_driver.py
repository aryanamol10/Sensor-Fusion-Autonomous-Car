import time
import threading

class EncoderDriver:
    def __init__(self, left_pin_a, left_pin_b, right_pin_a, right_pin_b, ticks_per_meter):
        """
        Initialize wheel encoders.
        Assumes two quadrature encoders (left and right wheels).
        """
        self.left_pin_a = left_pin_a
        self.left_pin_b = left_pin_b
        self.right_pin_a = right_pin_a
        self.right_pin_b = right_pin_b
        self.ticks_per_meter = ticks_per_meter
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.connected = False
        
        self._lock = threading.Lock()
        self._setup()

    def _setup(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            # Setup pins
            for pin in [self.left_pin_a, self.left_pin_b, self.right_pin_a, self.right_pin_b]:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Setup interrupts (simple example using only one pin per encoder for 1x decoding)
            GPIO.add_event_detect(self.left_pin_a, GPIO.RISING, callback=self._left_callback)
            GPIO.add_event_detect(self.right_pin_a, GPIO.RISING, callback=self._right_callback)
            self.connected = True
            print("Encoders initialized via RPi.GPIO")
        except ImportError:
            print("RPi.GPIO not found. Encoders running in mock mode.")
            self.connected = False
        except Exception as e:
            print(f"Failed to initialize encoders: {e}")
            self.connected = False

    def _left_callback(self, channel):
        # In a real quadrature setup, read pin B to determine direction
        with self._lock:
            self.left_ticks += 1

    def _right_callback(self, channel):
        with self._lock:
            self.right_ticks += 1

    def get_data(self):
        """
        Read the current tick counts.
        Returns:
            dict: Containing 'left_ticks', 'right_ticks', 'timestamp'
        """
        with self._lock:
            # We copy and reset or just return absolute depending on odometry design
            return {
                'left_ticks': self.left_ticks,
                'right_ticks': self.right_ticks,
                'timestamp': time.time()
            }
