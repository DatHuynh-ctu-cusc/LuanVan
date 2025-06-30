import socket, time
from encoder_handler import positions, lock
from limit_switch_handler import get_limit_states

def send_to_pi4(running_flag):
    HOST_PI4 = '192.168.100.1'
    PORT = 9999
    while running_flag.is_set():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST_PI4, PORT))
                print("[App] Ket noi encoder/limit toi Pi4")
                while running_flag.is_set():
                    with lock:
                        pos_str = ';'.join([f"{k}:{v}" for k, v in positions.items()])
                    limits = ';'.join([f"{k}:{v}" for k, v in get_limit_states().items()])
                    msg = f"ENC{{{pos_str}}};LIMITS{{{limits}}}\n"
                    s.sendall(msg.encode())
                    time.sleep(0.1)
        except:
            time.sleep(2)
