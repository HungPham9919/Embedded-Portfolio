import cv2
import threading

class CameraThread(threading.Thread):
    def __init__(self, cam_idx, width, height, fps):
        super().__init__(daemon=True)
        self.cam_idx = cam_idx
        self.size = (width, height)
        self.fps = fps
        self.frame = None
        self.lock = threading.Lock() # Bảo vệ dữ liệu frame
        self.stop_event = threading.Event()
        
    def run(self):
        cap = cv2.VideoCapture(self.cam_idx)
        if not cap.isOpened():
            print(f"❌ Camera {self.cam_idx} không khả dụng!")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if ret:
                with self.lock:
                    self.frame = frame.copy()
        cap.release()

    def stop(self):
        self.stop_event.set()
