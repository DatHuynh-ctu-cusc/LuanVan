from gpiozero import DigitalInputDevice
import threading

ENCODERS = {
    'E1': {'A': 21, 'B': 20},
    'E2': {'A': 5,  'B': 6},
    'E3': {'A': 12, 'B': 18},
    'E4': {'A': 23, 'B': 24}
}

positions = {key: 0 for key in ENCODERS}
encoders = {}
lock = threading.Lock()

def init_encoders():
    for key, pins in ENCODERS.items():
        encoders[key] = {
            'A': DigitalInputDevice(pins['A']),
            'B': DigitalInputDevice(pins['B'])
        }
        encoders[key]['A'].when_activated = make_callback(key)
        encoders[key]['A'].when_deactivated = make_callback(key)

def make_callback(key):
    def callback():
        with lock:
            if encoders[key]['B'].value == 0:
                positions[key] += 1
            else:
                positions[key] -= 1
    return callback

def cleanup_encoders():
    for enc in encoders.values():
        enc['A'].close()
        enc['B'].close()
