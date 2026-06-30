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
        
        for name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                if cv2.contourArea(cnt) > 100:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Lấy tọa độ Y làm độ cao (với cam ngang, Y càng nhỏ nghĩa là drone càng cao)
                    # Hoặc bạn có thể quy đổi Y sang cm tùy ý
                    drones_z[name] = y 
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    break
        return drones_z

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
