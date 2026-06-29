import zmq
import time
import random # Dùng tạm để test tọa độ nhảy ngẫu nhiên

def main():
    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUB)
    
    # Bind vào port 5555, cho phép tất cả các IP kết nối tới qua mạng LAN
    socket.bind("tcp://*:5555")
    print("ZMQ Publisher đã khởi động trên Pi 5 (Port: 5555)...")

    x, y, z = 0, 0, 0
    
    try:
        while True:
            # Giả lập tọa độ robot từ thuật toán xử lý ảnh của bạn
            x = (x + 1) % 500
            y = (y + 2) % 500
            z = random.randint(0, 180) # Ví dụ: Góc quay yaw của drone từ 0-180 độ
            
            # Đóng gói dữ liệu thành chuỗi format cố định: "robot1 X Y Z"
            msg = f"robot1 {x} {y} {z}"
            
            # Gửi chuỗi dạng bytes qua mạng
            socket.send_string(msg)
            print(f"-> Đã gửi: {msg}")
            
            # Tần số gửi tin (Ví dụ: 0.05s = 20Hz hoặc 0.1s = 10Hz cho robot)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nĐang dừng Publisher trên Pi 5...")
    finally:
        socket.close()

if __name__ == "__main__":
    main()
