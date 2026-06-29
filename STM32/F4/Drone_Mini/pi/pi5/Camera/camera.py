import cv2
import threading
import time

class CameraThread(threading.Thread):
    def __init__(self, cam_idx, width, height, fps):
        super().__init__()
        self.cam_idx = cam_idx
        self.width = width
        self.height = height
        self.fps = fps
        self.frame = None
        self.ret = False
        self.running = True
        
    def run(self):
        # Khởi tạo camera với driver V4L2
        cap = cv2.VideoCapture(self.cam_idx, cv2.CAP_V4L2)
        
        # Cấu hình phần cứng
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        print(f" Đã mở phần cứng Camera {self.cam_idx}")
        
        while self.running:
            ret_clean, frame_clean = cap.read()
            
            if ret_clean and frame_clean is not None:
                self.frame = frame_clean
                self.ret = True
                time.sleep(0.01)
            else:
                time.sleep(0.005)
                
        cap.release()
