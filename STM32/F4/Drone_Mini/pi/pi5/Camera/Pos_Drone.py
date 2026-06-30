import cv2
import numpy as np

class DroneTracker:
    def __init__(self):
        self.colors = {
            "Cam": ([0, 50, 50], [20, 255, 255]),
            "Vang": ([20, 50, 50], [40, 255, 255]),
            "XanhLa": ([40, 50, 50], [90, 255, 255]),
            "Hong": ([130, 50, 50], [180, 255, 255])
        }
        self.prev_pos = {} # Lưu vị trí cũ để làm mượt

    def process_camera_xy(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        drones_found = {}

        # Duyệt qua từng màu
        for name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Khử nhiễu mạnh hơn
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_cnt = None
            max_area = 0
            
            # Chỉ chọn contour lớn nhất cho màu đó để tránh nhảy lung tung
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100 and area > max_area: # Ngưỡng 100 để bỏ nhiễu nhỏ
                    max_area = area
                    best_cnt = cnt
            
            if best_cnt is not None:
                x, y, w, h = cv2.boundingRect(best_cnt)
                drones_found[name] = (x + w//2, y + h//2)
                # Vẽ khung
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return drones_found

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
