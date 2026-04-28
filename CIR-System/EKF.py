import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from collections import deque
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from filterpy.kalman import ExtendedKalmanFilter

ANCHORS = [
    (0, 0),
    (5470, 0),
    (5420, 5050),
    (770, 5050),
]

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

class EKFPositionTracker:
    def __init__(self):
        # State vector: [x, y, vx, vy]
        self.dim_x = 4
        # Measurement vector: [d1, d2, d3, d4]
        self.dim_z = 4

        self.ekf = ExtendedKalmanFilter(dim_x=self.dim_x, dim_z=self.dim_z)

        # Transition matrix
        self.ekf.F = np.eye(self.dim_x)
        self.ekf.F[0, 2] = 1.0  # x = x + vx*dt
        self.ekf.F[1, 3] = 1.0  # y = y + vy*dt

        # Covariance matrix
        self.ekf.P = np.eye(self.dim_x) * 100

        # Process noise
        self.ekf.Q = np.eye(self.dim_x) * 0.1

        # Measurement noise
        self.ekf.R = np.eye(self.dim_z) * 50

        # Measurement matrix (will be updated with Jacobian)
        self.ekf.H = np.zeros((self.dim_z, self.dim_x))

        self.initialized = False
        self.dt = 0.1

    def H_jacobian(self, x):
        """Jacobian của hàm đo"""
        H = np.zeros((self.dim_z, self.dim_x))

        for i, anchor in enumerate(ANCHORS):
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            dist = np.sqrt(dx**2 + dy**2)

            if dist > 0.001:
                H[i, 0] = dx / dist  # ∂h/∂x
                H[i, 1] = dy / dist  # ∂h/∂y
                # ∂h/∂vx và ∂h/∂vy = 0

        return H

    def hx(self, x):
        """Hàm đo: tính khoảng cách từ vị trí x đến các anchor"""
        distances = []
        for anchor in ANCHORS:
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            distance = np.sqrt(dx**2 + dy**2)
            distances.append(distance)
        return np.array(distances)

    def initialize(self, initial_pos):
        """Khởi tạo EKF với vị trí ban đầu"""
        self.ekf.x = np.array([initial_pos[0], initial_pos[1], 0, 0])  # [x, y, vx, vy]
        self.initialized = True

    def update(self, distances, dt=0.1):
        """Cập nhật EKF với đo khoảng cách mới"""
        if not self.initialized:
            return np.array([np.nan, np.nan])

        self.dt = dt
        # Cập nhật dt trong transition matrix
        self.ekf.F[0, 2] = dt
        self.ekf.F[1, 3] = dt

        # Prediction step
        self.ekf.predict()

        # Measurement
        z = np.array(distances)

        # Update Jacobian
        self.ekf.H = self.H_jacobian(self.ekf.x)

        # Update step
        self.ekf.update(z, self.H_jacobian, self.hx)

        return self.ekf.x[:2]  # Trả về vị trí [x, y]


class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UWB Tracking with EKF - PyQtGraph")
        self.setGeometry(100, 100, 1000, 800)

        self.data_queue = Queue()
        self.position_history = deque(maxlen=100)
        self.trail_history = deque(maxlen=1000)
        self.fps_time = time.time()
        self.last_update_time = time.time()

        # Regex để match định dạng: 0x1004,1838
        self.serial_pattern = re.compile(r'0x([0-9a-fA-F]{4}),(\d+)')

        # Buffer để lưu khoảng cách từ 4 anchor
        self.distance_buffer = {}
        self.last_distances = None

        # EKF tracker
        self.ekf_tracker = EKFPositionTracker()
        self.ekf_initialized = False

        # Timeout để reset buffer
        self.buffer_timeout = 0.5
        self.last_buffer_time = time.time()

        # Bộ đếm số dòng serial
        self.serial_line_count = 0
        self.serial_lines_per_sec = 0
        self.serial_count_timer = time.time()

        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        # Timer cho cập nhật plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(16)  # ~60 FPS

    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.01
            )
            self.ser.reset_input_buffer()
            print(f"✓ Kết nối serial: {SERIAL_PORT} @ {BAUDRATE}")
        except Exception as e:
            print(f"✗ Lỗi serial: {e}")
            sys.exit(1)

    def init_ui(self):
        # Tạo central widget và layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        # Tạo PlotWidget với background đen
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', 'Y (mm)')
        self.plot_widget.setLabel('bottom', 'X (mm)')
        self.plot_widget.setTitle('UWB Real-Time Tracking with EKF', color='w', size='14pt')

        # Thiết lập phạm vi
        self.plot_widget.setXRange(-1000, 7000)
        self.plot_widget.setYRange(-1000, 6000)
        self.plot_widget.setAspectLocked(True)

        # Vẽ các anchor (màu xanh dương)
        anchor_x, anchor_y = zip(*ANCHORS)
        self.anchor_scatter = pg.ScatterPlotItem(
            x=anchor_x,
            y=anchor_y,
            size=15,
            pen=pg.mkPen('c', width=2),
            brush=pg.mkBrush(0, 150, 255, 200),
            symbol='s'
        )
        self.plot_widget.addItem(self.anchor_scatter)

        # Thêm label cho các anchor
        for i, (x, y) in enumerate(ANCHORS):
            text = pg.TextItem(f"A{i}", color='c', anchor=(0.5, 1.5))
            text.setPos(x, y)
            self.plot_widget.addItem(text)

        # Vẽ tag (màu đỏ)
        self.tag_scatter = pg.ScatterPlotItem(
            size=20,
            pen=pg.mkPen('r', width=2),
            brush=pg.mkBrush(255, 0, 0, 200),
            symbol='o'
        )
        self.plot_widget.addItem(self.tag_scatter)

        # Vẽ trail (đường đi)
        self.trail_curve = pg.PlotCurveItem(
            pen=pg.mkPen('y', width=2),
            antialias=True
        )
        self.plot_widget.addItem(self.trail_curve)

        # Text hiển thị tọa độ tag
        self.tag_text = pg.TextItem(
            text="TAG",
            color='w',
            anchor=(0.5, 1.5),
            border=pg.mkPen('r', width=1),
            fill=pg.mkBrush(0, 0, 0, 150)
        )
        self.plot_widget.addItem(self.tag_text)

        # Vẽ khu vực (màu xanh lá)
        zone_x = [1200, 1200, 4800, 4800, 1200]
        zone_y = [600, 2710, 2710, 600, 600]
        self.zone_line = pg.PlotCurveItem(
            x=zone_x,
            y=zone_y,
            pen=pg.mkPen('g', width=2, style=QtCore.Qt.DashLine)
        )
        self.plot_widget.addItem(self.zone_line)

        layout.addWidget(self.plot_widget)

        # Status bar
        self.status_label = QtWidgets.QLabel("Đang chờ dữ liệu...")
        self.status_label.setStyleSheet("color: white; background-color: #333; padding: 5px;")
        layout.addWidget(self.status_label)

    def start_serial_thread(self):
        threading.Thread(target=self.serial_reader, daemon=True).start()

    def serial_reader(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    # Đếm số dòng serial nhận được
                    self.serial_line_count += 1

                    # Tính số dòng/giây mỗi giây
                    current_time = time.time()
                    if current_time - self.serial_count_timer >= 1.0:
                        self.serial_lines_per_sec = self.serial_line_count
                        self.serial_line_count = 0
                        self.serial_count_timer = current_time

                    match = self.serial_pattern.search(line)
                    if match:
                        device_id = match.group(1)
                        distance = int(match.group(2))

                        current_time = time.time()

                        # Reset buffer nếu timeout
                        if current_time - self.last_buffer_time > self.buffer_timeout:
                            if len(self.distance_buffer) > 0:
                                self.distance_buffer.clear()

                        self.last_buffer_time = current_time
                        self.distance_buffer[device_id] = distance

                        # Khi đủ 4 anchor
                        if len(self.distance_buffer) == 4:
                            sorted_devices = sorted(self.distance_buffer.keys())
                            distances = [self.distance_buffer[dev_id] for dev_id in sorted_devices]

                            if self.is_valid_distances(distances):
                                self.data_queue.put(distances)
                                self.last_distances = distances

                            self.distance_buffer.clear()

            except Exception as e:
                print(f"Serial error: {e}")
                time.sleep(0.01)

    def is_valid_distances(self, distances):
        """Kiểm tra tính hợp lệ của khoảng cách"""
        MIN_DISTANCE = 100
        MAX_DISTANCE = 10000

        for d in distances:
            if d < MIN_DISTANCE or d > MAX_DISTANCE:
                return False

        if self.last_distances is not None:
            MAX_CHANGE = 1000
            for i in range(len(distances)):
                if abs(distances[i] - self.last_distances[i]) > MAX_CHANGE:
                    return False

        return True

    def lse_trilateration(self, distances):
        """Tính toán vị trí bằng LSE (để khởi tạo EKF)"""
        if len(distances) < 4:
            return np.array([np.nan, np.nan])

        x1, y1 = ANCHORS[0]
        d1 = distances[0]
        A = []
        b = []

        for i in range(1, 4):
            xi, yi = ANCHORS[i]
            di = distances[i]
            A.append([2 * (xi - x1), 2 * (yi - y1)])
            b.append((d1**2 - di**2) - (x1**2 - xi**2) - (y1**2 - yi**2))

        try:
            A = np.array(A)
            b = np.array(b)
            pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            return pos
        except Exception as e:
            return np.array([np.nan, np.nan])

    def update_plot(self):
        """Cập nhật plot - được gọi mỗi 16ms (~60 FPS)"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Tính FPS
        fps = 1.0 / (current_time - self.fps_time) if (current_time - self.fps_time) > 0 else 0
        self.fps_time = current_time

        updated = False

        # Xử lý tất cả dữ liệu trong queue
        while not self.data_queue.empty():
            distances = self.data_queue.get()

            if len(distances) >= 4:
                # Khởi tạo EKF nếu chưa
                if not self.ekf_initialized:
                    initial_pos = self.lse_trilateration(distances[:4])
                    if not np.isnan(initial_pos).any():
                        self.ekf_tracker.initialize(initial_pos)
                        self.ekf_initialized = True
                        filtered_pos = initial_pos
                        print(f"✓ EKF initialized at X: {initial_pos[0]:.2f}, Y: {initial_pos[1]:.2f}")
                    else:
                        continue
                else:
                    # Cập nhật EKF với đo mới
                    filtered_pos = self.ekf_tracker.update(distances[:4], dt)

                if not np.isnan(filtered_pos).any():
                    # Kiểm tra vị trí có hợp lý
                    if -1000 <= filtered_pos[0] <= 7000 and -1000 <= filtered_pos[1] <= 7000:
                        self.position_history.append(filtered_pos)
                        self.trail_history.append(filtered_pos)

                        # In vị trí
                        print(f"X: {filtered_pos[0]:.2f}, Y: {filtered_pos[1]:.2f}")

                        # Cập nhật scatter plot của tag
                        self.tag_scatter.setData(x=[filtered_pos[0]], y=[filtered_pos[1]])

                        # Cập nhật text
                        self.tag_text.setText(f"TAG\n({filtered_pos[0]:.0f}, {filtered_pos[1]:.0f})")
                        self.tag_text.setPos(filtered_pos[0], filtered_pos[1])

                        updated = True

        # Cập nhật trail
        if updated and len(self.trail_history) > 1:
            trail = np.array(self.trail_history)
            self.trail_curve.setData(x=trail[:, 0], y=trail[:, 1])

        # Cập nhật status bar với thông tin serial lines/sec
        ekf_status = "EKF Active" if self.ekf_initialized else "Waiting for init"
        status_text = f"FPS: {fps:.1f} | {ekf_status} | Buffer: {len(self.distance_buffer)}/4 | Points: {len(self.trail_history)} | Serial: {self.serial_lines_per_sec} lines/sec"
        self.status_label.setText(status_text)

        # Cập nhật title
        self.setWindowTitle(f"UWB Tracking (EKF) - FPS: {fps:.1f} | Serial: {self.serial_lines_per_sec}/s")

    def closeEvent(self, event):
        """Đóng serial khi thoát"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("✓ Đã đóng cổng serial")
        event.accept()


if __name__ == "__main__":
    # Cấu hình PyQtGraph
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption('background', 'k')
    pg.setConfigOption('foreground', 'w')

    app = QtWidgets.QApplication(sys.argv)

    # Thiết lập dark theme
    app.setStyle('Fusion')
    dark_palette = QtGui.QPalette()
    dark_palette.setColor(QtGui.QPalette.Window, QtGui.QColor(53, 53, 53))
    dark_palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    app.setPalette(dark_palette)

    tracker = FPTTracker()
    tracker.show()

    print("=" * 50)
    print("UWB Tracking with EKF Started")
    print("=" * 50)

    sys.exit(app.exec_())
