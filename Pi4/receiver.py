# receiver.py
import socket
import threading
from motor_control import move_vehicle

limit_active = False

def start_receiver(shared_counts):
    def run():
        global limit_active
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('', 9999))
        server.listen(1)
        print("[Pi4] 🟢 Chờ Pi5 gửi encoder + limit switch...")

        while True:
            conn, addr = server.accept()
            print(f"[Pi4] ✅ Kết nối Pi5: {addr}")
            with conn:
                while True:
                    try:
                        data = conn.recv(1024).decode().strip()
                        if not data: break

                        if "ENC{" in data:
                            enc_block = data.split("ENC{")[1].split("}")[0]
                            enc_data = {kv.split(':')[0]: int(kv.split(':')[1]) for kv in enc_block.split(';') if ':' in kv}
                            shared_counts.update(enc_data)
                            print(f"[ENCODER] {shared_counts}")

                        if "LIMITS{" in data:
                            sw_block = data.split("LIMITS{")[1].split("}")[0]
                            sw_data = {kv.split(':')[0]: int(kv.split(':')[1]) for kv in sw_block.split(';') if ':' in kv}
                            print(f"[LIMIT] {sw_data}")
                            if sw_data.get('L1') or sw_data.get('L2'):
                                limit_active = True
                                print("[⚠️] L1/L2 được nhấn – Tiến 2s")
                                move_vehicle("forward", 0.1, 1.0, shared_counts)
                            elif sw_data.get('L3') or sw_data.get('L4'):
                                limit_active = True
                                print("[⚠️] L3/L4 được nhấn – Lùi 2s")
                                move_vehicle("backward", 0.1, 1.0, shared_counts)
                            else:
                                limit_active = False
                    except Exception as e:
                        print(f"[ERROR] Lỗi nhận Pi5: {e}")
                        break
    threading.Thread(target=run, daemon=True).start()
