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
            # Tăng S từ 30 lên 100 để loại bỏ nhiễu nền nhạt màu
            "Cam":    ([0, 100, 100], [20, 255, 255]),     
            "Vang":   ([20, 100, 100], [50, 255, 255]),
            "XanhLa": ([50, 100, 100], [90, 255, 255]),
            "Hong":   ([130, 40, 40], [179, 255, 255])
        }

        self.prev_pos = {} # Lưu vị trí cũ để làm mượt
        self.prev_pos_z = {}  # Tách riêng cho Z
        self.lost_frames = {name: 0 for name in ["Cam", "Vang", "XanhLa", "Hong"]}
        self.max_lost_frames = 5

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
        y_offset, x_offset = 100, 100
        roi = frame[y_offset:600, x_offset:1100]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        drones_z = {}
        
        for name, (lower, upper) in self.colors_z.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            if name != "Hong":
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            best_cnt = None
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                min_area = 2 if name == "Hong" else 20
                if min_area < area < 500 and area > max_area:
                    max_area = area
                    best_cnt = cnt
            
            # --- LOGIC XỬ LÝ ỔN ĐỊNH ---
            if best_cnt is not None:
                x, y, w, h = cv2.boundingRect(best_cnt)
                global_pos = (x + w//2 + x_offset, y + h//2 + y_offset)
                
                # Kiểm tra khoảng cách nhảy xa
                if name in self.prev_pos_z and np.linalg.norm(np.array(global_pos) - np.array(self.prev_pos_z[name])) > 100:
                    continue
                
                self.prev_pos_z[name] = global_pos
                self.lost_frames[name] = 0 # Reset bộ đếm
            
            # Nếu không tìm thấy nhưng còn trong bộ đệm
            elif name in self.prev_pos_z and self.lost_frames[name] < self.max_lost_frames:
                self.lost_frames[name] += 1
                global_pos = self.prev_pos_z[name] # Dùng vị trí cũ
            else:
                continue # Không tìm thấy và vượt quá bộ đệm
                
            # Vẽ kết quả
            z_val = 720 - global_pos[1]
            drones_z[name] = z_val
            cv2.circle(frame, global_pos, 10, (0, 0, 255), -1)
            cv2.putText(frame, f"{name}: {z_val}", (global_pos[0] + 15, global_pos[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return drones_z

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
