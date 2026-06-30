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
    cam_xy = CameraThread(cam_idx=0, width=1280, height=720, fps=30)
    proc = DataProcessor(width=1280, height=720)
    tracker = DroneTracker() 
    cam_xy.start()
    
    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rec.bind(("0.0.0.0", 7777))
    threading.Thread(target=listen_for_corners, args=(sock_rec, proc), daemon=True).start()
    
    try:
        while True:
            if cam_xy.frame is not None:
                frame = cam_xy.frame.copy()
                
                # 1. Tracker xử lý (tìm drone và vẽ khung xanh lên frame)
                drones_pos = tracker.process_camera_xy(frame)
                
                # 2. Processor vẽ lưới lên frame đã có khung xanh
                processed_img = proc.process(frame)
                
                # 3. Vẽ nhãn tên drone lên frame
                final_img = tracker.draw_drones(processed_img, drones_pos)
                
                # 4. Gửi ảnh
                _, enc = cv2.imencode('.jpg', final_img, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                sock_send.sendto(enc.tobytes(), (LAPTOP_IP, 9999))
            
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n🛑 Người dùng đã dừng hệ thống.")
    finally:
        print("🧹 Đang dọn dẹp tài nguyên...")
        cam_xy.stop()
        cam_xy.join()
        sock_send.close()
        sock_rec.close()

if __name__ == "__main__":
    main()
