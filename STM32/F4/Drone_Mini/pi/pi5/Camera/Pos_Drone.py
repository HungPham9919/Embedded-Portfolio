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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        drones_found = {}
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 300 < area < 5000:
                x, y, w, h = cv2.boundingRect(cnt)
                roi = hsv[y:y+h, x:x+w]
                
                for name, (lower, upper) in self.colors.items():
                    mask = cv2.inRange(roi, np.array(lower), np.array(upper))
                    # Tăng ngưỡng countNonZero để loại bỏ nhiễu chớp nháy
                    if cv2.countNonZero(mask) > 100: 
                        drones_found[name] = (x + w//2, y + h//2)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        break
        return drones_found

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
