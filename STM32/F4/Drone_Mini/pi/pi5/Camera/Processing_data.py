import cv2
import numpy as np

class DataProcessor:
    def __init__(self, width=1280, height=720):
        self.width = width
        self.height = height
        
        # Định nghĩa hệ tọa độ
        self.X_RANGE = (-3, 3)
        self.Y_RANGE = (-4, 4)
        self.GRID_SIZE_X = 7
        self.GRID_SIZE_Y = 9

        self.clicked_pts = []  
        self.matrix_inv = None 

    def set_corners(self, points):
        """Khớp với hàm gọi trong main.py"""
        self.clicked_pts = points
        src_pts = np.array(self.clicked_pts, dtype=np.float32)
        dst_pts = np.array([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]], dtype=np.float32)
        self.matrix_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        print("🔥 Đã khóa lưới tọa độ!")

    def process(self, frame):
        """Khớp với hàm gọi proc.process(frame) trong main.py"""
        display_frame = frame.copy()

        # Trường hợp 1: Chưa đủ 4 điểm -> Vẽ hướng dẫn
        if self.matrix_inv is None:
            for i, pt in enumerate(self.clicked_pts):
                cv2.circle(display_frame, tuple(pt), 8, (0, 0, 255), -1)
                cv2.putText(display_frame, str(i+1), (pt[0]+10, pt[1]+10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return display_frame

        # Trường hợp 2: Đã có ma trận -> Vẽ lưới
        grid_points_ideal = []
        grid_labels = []
        for row_idx in range(self.GRID_SIZE_Y):
            for col_idx in range(self.GRID_SIZE_X):
                x_val = self.X_RANGE[0] + col_idx
                y_val = self.Y_RANGE[1] - row_idx
                x_norm = col_idx / (self.GRID_SIZE_X - 1)
                y_norm = row_idx / (self.GRID_SIZE_Y - 1)
                grid_points_ideal.append([x_norm * self.width, y_norm * self.height])
                grid_labels.append((x_val, y_val))

        pts_ideal_arr = np.array([grid_points_ideal], dtype="float32")
        pts_camera_arr = cv2.perspectiveTransform(pts_ideal_arr, self.matrix_inv)[0]

        for i, pt in enumerate(pts_camera_arr):
            px, py = int(pt[0]), int(pt[1])
            if 0 <= px < self.width and 0 <= py < self.height:
                color = (0, 0, 255) if grid_labels[i] == (0, 0) else (255, 0, 0)
                cv2.circle(display_frame, (px, py), 5, color, -1)
                
        return display_frame
