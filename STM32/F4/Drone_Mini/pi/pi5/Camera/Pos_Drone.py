import cv2
import numpy as np

class DroneTracker:
    def __init__(self):
        self.colors = {
            "Cam": ([0, 80, 80], [18, 255, 255]),    
            "Vang": ([20, 80, 80], [40, 255, 255]),
            "XanhLa": ([40, 50, 50], [90, 255, 255]),
            "Hong": ([130, 50, 50], [180, 255, 255])
        }

        self.colors_z = {
            # Mở rộng dải H để bắt được màu Cam tốt hơn
            "Cam":    ([0, 80, 80], [25, 255, 255]),    
            "Vang":   ([20, 80, 80], [45, 255, 255]),
            # Xanh lá thường ổn định hơn nên giữ nguyên
            "XanhLa": ([40, 80, 80], [90, 255, 255]),
            "Hong":   ([130, 80, 80], [180, 255, 255])
        }

        self.prev_pos = {} # Lưu vị trí cũ để làm mượt

    def process_camera_xy(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        drones_found = {}

        for name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_cnt = None
            max_area = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 80 and area > max_area: # Giảm ngưỡng area xuống 80
                    max_area = area
                    best_cnt = cnt
            
            if best_cnt is not None:
                x, y, w, h = cv2.boundingRect(best_cnt)
                pos = (x + w//2, y + h//2)
                drones_found[name] = pos
                self.prev_pos[name] = pos # Lưu lại vị trí tốt nhất
            elif name in self.prev_pos:
                # Nếu không tìm thấy, dùng vị trí cũ (cho phép mất 1-2 frame không giật)
                drones_found[name] = self.prev_pos[name]

        return drones_found
    
    def process_camera_z(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        drones_z = {}
        
        # Dùng self.colors_z đã định nghĩa riêng
        for name, (lower, upper) in self.colors_z.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Dùng MORPH_CLOSE để nối các phần màu rời rạc trên drone
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_cnt = None
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 50 and area > max_area:
                    max_area = area
                    best_cnt = cnt
            
            if best_cnt is not None:
                x, y, w, h = cv2.boundingRect(best_cnt)
                z_val = 720 - (y + h) 
                drones_z[name] = z_val
                
                # Vẽ khung và độ cao lên frame
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(frame, f"{name}: {z_val}", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        return drones_z

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
