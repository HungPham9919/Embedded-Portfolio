import cv2
import time
import socket
import threading
from Camera.camera import CameraThread
from Camera.Processing_data import DataProcessor

def listen_for_corners(sock, processor):
    """ Luồng chạy ngầm để nhận tọa độ click chuột gửi ngược từ Laptop về """
    print("📥 Luồng UDP lắng nghe chuột đã được kích hoạt thành công...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            
            if msg == "reset":
                processor.clicked_pts = []
                processor.matrix = None
                print("🔄 Laptop yêu cầu reset góc. Vui lòng click chọn lại từ đầu!")
            else:
                try:
                    # Nhận tọa độ dạng "x,y" từ Laptop
                    x, y = map(int, msg.split(','))
                    if len(processor.clicked_pts) < 4:
                        processor.clicked_pts.append([x, y])
                        print(f"📍 SERVER: Đã nhận & lưu góc {len(processor.clicked_pts)} từ {addr}: ({x}, {y})")
                        
                        if len(processor.clicked_pts) == 4:
                            # Tính toán lại ma trận ngay lập tức khi đủ 4 điểm
                            processor.update_matrix_from_pts()
                except ValueError:
                    print(f"⚠️ Định dạng tọa độ không hợp lệ: {msg}")
        except Exception as e:
            print(f"❌ Lỗi luồng nhận dữ liệu chuột: {e}")
            break

def main():
    # CẤU HÌNH ĐỊA CHỈ IP LAPTOP NHẬN ẢNH
    LAPTOP_IP = "192.168.1.41" 
    UDP_PORT_XY = 9999
    UDP_PORT_Z = 8888
    UDP_PORT_REC = 7777 # Cổng để Server mở ra lắng nghe tọa độ từ Laptop bắn về
    
    sock_xy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_z = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Khởi tạo Socket lắng nghe phản hồi chuột từ Laptop (Bind đúng port 7777)
    sock_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rec.bind(("0.0.0.0", UDP_PORT_REC))
    
    processor = DataProcessor(width=1280, height=720)
    
    # Khởi động luồng phụ nhận dữ liệu chuột từ xa
    thr = threading.Thread(target=listen_for_corners, args=(sock_rec, processor), daemon=True)
    thr.start()
    
    cam_xy = CameraThread(cam_idx=0, width=1280, height=720, fps=30)
    cam_z = CameraThread(cam_idx=2, width=1024, height=576, fps=30)
    
    cam_xy.start()
    cam_z.start()
    
    print(f"🚀 Hệ thống Headless chuẩn DATN đang hoạt động...")
    print(f"📡 Đang bắn dữ liệu video về Laptop ({LAPTOP_IP})")
    print(f"📥 Đồng thời mở cổng {UDP_PORT_REC} chờ nhận tọa độ chuột...")
    
    try:
        while True:
            # Luồng 1: Kênh XY
            if cam_xy.ret and cam_xy.frame is not None:
                current_frame_xy = cam_xy.frame
                cam_xy.ret = False 
                                    
                warped_frame, _ = processor.detect_coordinates(current_frame_xy)
                
                resized_xy = cv2.resize(warped_frame, (1280, 720))
                ret1, encode_xy = cv2.imencode('.jpg', resized_xy, (int(cv2.IMWRITE_JPEG_QUALITY), 50))
                if ret1:
                    data_xy = encode_xy.tobytes()
                    if len(data_xy) < 65507:
                        sock_xy.sendto(data_xy, (LAPTOP_IP, UDP_PORT_XY))
                            
            # Luồng 2: Kênh Z
            if cam_z.ret and cam_z.frame is not None:
                current_frame_z = cam_z.frame
                cam_z.ret = False 
                
                resized_z = cv2.resize(current_frame_z, (1024, 576))
                ret2, encode_z = cv2.imencode('.jpg', resized_z, (int(cv2.IMWRITE_JPEG_QUALITY), 50))
                if ret2:
                    data_z = encode_z.tobytes()
                    if len(data_z) < 65507:
                        sock_z.sendto(data_z, (LAPTOP_IP, UDP_PORT_Z))
                            
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n Đang tắt hệ thống...")
    finally:
        cam_xy.running = False
        cam_z.running = False
        cam_xy.join()
        cam_z.join()
        sock_xy.close()
        sock_z.close()
        sock_rec.close()

if __name__ == '__main__':
    main()
