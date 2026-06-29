import cv2
import numpy as np

class DataProcessor:
    def __init__(self, width=1280, height=720):
        self.width = width
        self.height = height
        
        # Định nghĩa hệ tọa độ thực tế trên bạt của bạn
        self.X_RANGE = (-3, 3)
        self.Y_RANGE = (-4, 4)
        self.GRID_SIZE_X = self.X_RANGE[1] - self.X_RANGE[0] + 1  # 7 cột
        self.GRID_SIZE_Y = self.Y_RANGE[1] - self.Y_RANGE[0] + 1  # 9 hàng

        self.clicked_pts = []  # Nơi lưu 4 điểm nhận từ Laptop
        self.matrix_inv = None # Ma trận nghịch đảo dùng để chiếu ngược tọa độ lên ảnh gốc

    def update_matrix_from_pts(self):
        """ Tính ma trận ánh xạ từ hệ lưới phẳng lý thuyết ngược về ảnh gốc camera """
        if len(self.clicked_pts) != 4:
            return
            
        # 4 góc thực tế người dùng click trên ảnh camera (Theo thứ tự: Trên-Trái, Trên-Phải, Dưới-Phải, Dưới-Trái)
        src_pts = np.array(self.clicked_pts, dtype="float32")
        
        # Định vị 4 góc lý thuyết tương ứng trong không gian lưới ảo (Ví dụ: tạo một khung hình ảo)
        # Giữ nguyên kích thước ảo bằng kích thước camera để không bị méo tỉ lệ chữ
        dst_pts = np.float32([[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]])
        
        # LƯU Ý: Ta tính ma trận biến đổi TỪ lưới ảo VỀ ảnh gốc camera (Nghịch đảo phối cảnh)
        self.matrix_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        print("🔥 [Thành công] Đã khóa lưới tọa độ đè trực tiếp lên ảnh gốc, không kéo giãn hình!")

    def detect_coordinates(self, frame):
        # Tạo bản sao ảnh gốc để vẽ đè dữ liệu lên
        display_frame = frame.copy()

        # Trường hợp 1: Chưa click đủ 4 góc -> Vẽ hướng dẫn chọn góc
        if self.matrix_inv is None:
            for i, pt in enumerate(self.clicked_pts):
                cv2.circle(display_frame, tuple(pt), 6, (0, 0, 255), -1)
                cv2.putText(display_frame, str(i+1), (pt[0]+10, pt[1]+10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.putText(display_frame, f"WAITING FOR LAPTOP CLICK CORNERS ({len(self.clicked_pts)}/4)...", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return display_frame, []

        # Trường hợp 2: Đã có ma trận -> Chiếu ngược lưới điểm toán học lên ảnh gốc camera
        # Bước A: Tạo danh sách tất cả các điểm lưới lý thuyết trên mặt phẳng phẳng
        grid_points_ideal = []
        grid_labels = []

        for row_idx in range(self.GRID_SIZE_Y):
            for col_idx in range(self.GRID_SIZE_X):
                x_val = self.X_RANGE[0] + col_idx
                y_val = self.Y_RANGE[1] - row_idx
                
                # Tính toán tọa độ tỉ lệ lý thuyết chuẩn trên khung ảo 1280x720
                x_norm = col_idx / (self.GRID_SIZE_X - 1)
                y_norm = row_idx / (self.GRID_SIZE_Y - 1)
                ideal_x = x_norm * self.width
                ideal_y = y_norm * self.height
                
                grid_points_ideal.append([ideal_x, ideal_y])
                grid_labels.append((x_val, y_val))

        # Bước B: Dùng toán hình học chiếu phối cảnh biến đổi đồng loạt mảng điểm về pixel thực tế trên cam
        pts_ideal_arr = np.array([grid_points_ideal], dtype="float32")
        pts_camera_arr = cv2.perspectiveTransform(pts_ideal_arr, self.matrix_inv)[0]

        # Bước C: Vẽ các điểm toán học đã được uốn cong theo góc nghiêng của camera lên hình
        for i, pt in enumerate(pts_camera_arr):
            px_x, px_y = int(pt[0]), int(pt[1])
            x_val, y_val = grid_labels[i]

            # Kiểm tra tránh vẽ tràn ra ngoài viền ảnh camera nếu tọa độ nhảy lỗi
            if 0 <= px_x < self.width and 0 <= px_y < self.height:
                if x_val == 0 and y_val == 0:
                    # Tâm gốc tọa độ (0,0) vẽ màu Đỏ to rõ ràng
                    cv2.circle(display_frame, (px_x, px_y), 7, (0, 0, 255), -1)
                else:
                    # Các điểm vệ tinh khác vẽ màu Xanh Dương
                    cv2.circle(display_frame, (px_x, px_y), 5, (255, 0, 0), -1)

                # Gán nhãn text tọa độ tương ứng nghiêng theo vị trí điểm
                cv2.putText(display_frame, f"({x_val},{y_val})", (px_x + 6, px_y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                
        return display_frame, []
