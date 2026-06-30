import time
import socket
import cv2
import threading
from Camera.camera import CameraThread
from Camera.Processing_data import DataProcessor
# Nhập class mới từ file Pos_Drone.py (Giả sử tên file là Pos_Drone.py)
from Camera.Pos_Drone import DroneTracker 

def listen_for_corners(sock, processor):
    print("Luồng UDP lắng nghe chuột đã kích hoạt...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            if msg == "reset":
                processor.matrix_inv = None
                processor.clicked_pts = []
                print(" Reset lưới tọa độ.")
            else:
                x, y = map(int, msg.split(','))
                if len(processor.clicked_pts) < 4:
                    processor.clicked_pts.append([x, y])
                    print(f" Đã nhận góc {len(processor.clicked_pts)}: ({x}, {y})")
                    if len(processor.clicked_pts) == 4:
                        processor.set_corners(processor.clicked_pts)
        except Exception as e:
            print(f"⚠️ Lỗi nhận tọa độ: {e}")
            break

def main():
    LAPTOP_IP = "192.168.1.41"
    print("🚀 Khởi động hệ thống...")
    
    # 1. Khởi tạo Camera, Processor và Tracker
    cam_xy = CameraThread(cam_idx=0, width=1280, height=720, fps=30)
    proc = DataProcessor(width=1280, height=720)
    tracker = DroneTracker() # Khởi tạo Tracker
    cam_xy.start()
    
    # 2. Khởi tạo Socket
    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rec.bind(("0.0.0.0", 7777))
    
    threading.Thread(target=listen_for_corners, args=(sock_rec, proc), daemon=True).start()
    
    print("✅ Hệ thống đã sẵn sàng phát sóng. Nhấn Ctrl+C để dừng.")
    
    try:
        while True:
            if cam_xy.frame is not None:
                frame = cam_xy.frame.copy()
                
                # BƯỚC 1: Xử lý lưới (vẽ nền lưới)
                processed_img = proc.process(frame)
                
                # BƯỚC 2: Định vị drone và vẽ lên hình
                drones_pos = tracker.process_camera_xy(frame)
                processed_img = tracker.draw_drones(processed_img, drones_pos)
                
                # Gửi tới Laptop
                _, enc = cv2.imencode('.jpg', processed_img, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                sock_send.sendto(enc.tobytes(), (LAPTOP_IP, 9999))
            
            time.sleep(0.03)
            
    except KeyboardInterrupt:
        print("\n Đang dừng hệ thống...")
    finally:
        cam_xy.stop()
        cam_xy.join()

if __name__ == "__main__":
    main()
