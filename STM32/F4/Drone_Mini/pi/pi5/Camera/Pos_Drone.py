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
            
            # --- CẢI TIẾN: Bắt lại cực nhanh ---
            if best_cnt is not None:
                x, y, w, h = cv2.boundingRect(best_cnt)
                current_pos = np.array([x + w//2 + x_offset, y + h//2 + y_offset])
                
                # Làm mượt tọa độ bằng trung bình trượt (trọng số 0.7 cho vị trí cũ)
                if name in self.prev_pos_z:
                    new_pos = (0.3 * current_pos + 0.7 * np.array(self.prev_pos_z[name])).astype(int)
                else:
                    new_pos = current_pos
                
                self.prev_pos_z[name] = tuple(new_pos)
                self.lost_frames[name] = 0
            
            # Nếu không tìm thấy, vẫn trả về vị trí cuối cùng đã biết (giống hệt XY)
            elif name in self.prev_pos_z:
                self.lost_frames[name] += 1
            
            # Vẽ nếu đã từng có vị trí
            if name in self.prev_pos_z:
                pos = self.prev_pos_z[name]
                z_val = 720 - pos[1]
                drones_z[name] = z_val
                
                # Màu đỏ đậm nếu đang detect thật, màu cam nếu đang dùng vị trí cũ (mất tín hiệu)
                color = (0, 0, 255) if self.lost_frames[name] == 0 else (0, 165, 255)
                cv2.circle(frame, pos, 10, color, -1)
                cv2.putText(frame, f"{name}: {z_val}", (pos[0] + 15, pos[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                            
        return drones_z

    def draw_drones(self, frame, drones_found):
        for name, (x, y) in drones_found.items():
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
            cv2.putText(frame, name, (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame
