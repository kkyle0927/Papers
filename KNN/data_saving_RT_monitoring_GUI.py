import sys
import os
import glob
import csv
import struct
import time
import re
import serial
import serial.tools.list_ports
from datetime import datetime

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt

import pyqtgraph as pg
import numpy as np
from collections import deque, Counter

# KNN 헤더 파싱 함수
def load_knn_ref_dataset_from_header(header_path):
    import re
    with open(header_path, 'r', encoding='utf-8') as f:
        text = f.read()
    # REF_TYPE1_COUNT
    m = re.search(r'#define\s+REF_TYPE1_COUNT\s+(\d+)', text)
    split = int(m.group(1)) if m else 280
    # ref_xy 배열 추출
    arr_txt = re.search(r'static const float ref_xy\[.*?\]=\s*\{([\s\S]*?)\};', text)
    arr_lines = arr_txt.group(1).split('\n') if arr_txt else []
    data = []
    for line in arr_lines:
        line = line.strip().rstrip(',')
        if not line:
            continue
        vals = re.findall(r'([-+]?[0-9]*\.?[0-9]+)f?', line)
        if len(vals) == 2:
            data.append([float(vals[0]), float(vals[1])])
    return np.array(data), split

# ===================== 0) 백엔드 고정 설정 =====================
UPDATE_INTERVAL_MS = 50         # 실시간 그래프 갱신 주기 (ms) (20Hz)
WINDOW_SIZE_SAMPLES = 1000      # 실시간 플롯 최근 샘플 수
SCATTER_KEEP_LAST_N = 20        # 4번 Scatter plot에서 유지할 최근 점 개수

# 2ms, 패킷 약 300 bytes라고 가정하면 1초에 약 500패킷 → 500라인 모아서 한 번에 write
FLUSH_EVERY = 500               # CSV 라인 500개 모을 때마다 디스크에 기록

# ===================== 1) 패킷/CRC 설정 =====================

SOF_VALUE = 0xAA55
SOF_BYTES_LE = b"\x55\xAA"

## Firmware SavingData_t (packed) payload schema:
STRUCT_FMT = '<IBBB20f6iBBf17f'
PAYLOAD_SIZE = struct.calcsize(STRUCT_FMT)

EXPECTED_TOTAL_PACKET_SIZE = 2 + 2 + PAYLOAD_SIZE + 2  # 191
ALLOWED_TOTAL_PACKET_SIZES = {EXPECTED_TOTAL_PACKET_SIZE}

CSV_HEADER = (
    "LoopCnt,H10Mode,H10AssistLevel,SmartAssist,"
    "LeftHipAngle,RightHipAngle,LeftThighAngle,RightThighAngle,"
    "LeftHipTorque,RightHipTorque,LeftHipMotorAngle,RightHipMotorAngle,"
    "LeftHipImuGlobalAccX,LeftHipImuGlobalAccY,LeftHipImuGlobalAccZ,"
    "LeftHipImuGlobalGyrX,LeftHipImuGlobalGyrY,LeftHipImuGlobalGyrZ,"
    "RightHipImuGlobalAccX,RightHipImuGlobalAccY,RightHipImuGlobalAccZ,"
    "RightHipImuGlobalGyrX,RightHipImuGlobalGyrY,RightHipImuGlobalGyrZ,"
    "is_moving,hc_count,R_count_upeak,L_count_upeak,R_count_dpeak,L_count_dpeak,"
    "tau_max_setting,s_gait_mode,s_g_knn_conf,"
    "T_swing_ms,T_swing_SOS_ms,T_swing_STS_ms,"
    "T_swing_SOS_ms_conf1,T_swing_STS_ms_conf1,TswingRecording_ms,"
    "s_vel_HC,s_T_HC_s,"
    "s_norm_vel_HC,s_norm_T_HC,s_scaling_X,s_scaling_Y,"
    "s_t_gap_R_ms,s_t_gap_L_ms,s_hc_deg_thresh,s_thres_up,s_thres_down"
)
CSV_COLS = CSV_HEADER.split(",")


def make_missing_row(loop_cnt: int, last_row=None):
    row = [float("nan")] * len(CSV_COLS)
    row[0] = int(loop_cnt)
    int_indices = {0, 1, 2, 3, 24, 25, 26, 27, 28, 29, 30, 31}
    if last_row is None:
        for idx in int_indices:
            if idx == 0: continue
            row[idx] = 0
        return row
    for idx in int_indices:
        if idx == 0: continue
        try: row[idx] = int(last_row[idx])
        except Exception: row[idx] = 0
    return row

def _is_finite_number(x) -> bool:
    try: return np.isfinite(float(x))
    except Exception: return False

def sanity_check_row(row, last_good_row=None):
    try: loop_cnt = int(row[0])
    except Exception: return False, "Invalid loopCnt"
    
    if last_good_row is not None:
        try:
            prev = int(last_good_row[0])
            if loop_cnt != 0 and loop_cnt < prev:
                return False, f"loopCnt went backwards ({loop_cnt} < {prev})"
            if loop_cnt - prev > 5000:
                return False, f"loopCnt jump too large ({loop_cnt} vs {prev})"
        except Exception: pass

    try:
        is_moving = int(row[CSV_COLS.index("is_moving")])
        if is_moving not in (0, 1): return False, f"is_moving out of range ({is_moving})"
    except Exception: return False, "is_moving missing"

    try:
        tau = int(row[CSV_COLS.index("tau_max_setting")])
        if not (0 <= tau <= 7): return False, f"tau_max_setting out of range ({tau})"
    except Exception: return False, "tau_max_setting missing"

    try:
        h10_mode = int(row[CSV_COLS.index("H10Mode")])
        if not (0 <= h10_mode <= 10): return False, f"h10Mode out of range ({h10_mode})"
    except Exception: return False, "H10Mode missing"

    for key in ("LeftThighAngle", "RightThighAngle", "LeftHipAngle", "RightHipAngle"):
        try:
            v = float(row[CSV_COLS.index(key)])
            if not np.isfinite(v) or abs(v) > 360.0: return False, f"{key} invalid ({v})"
        except Exception: return False, f"{key} missing"

    for key in ("LeftHipTorque", "RightHipTorque"):
        try:
            v = float(row[CSV_COLS.index(key)])
            if not np.isfinite(v): return False, f"{key} invalid ({v})"
        except Exception: return False, f"{key} missing"

    return True, ""

DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 1.0

def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
            crc &= 0xFFFF
    return crc

def decode_packet(data_tuple):
    expected_elems = 1 + 3 + 20 + 6 + 2 + 1 + 17
    if len(data_tuple) != expected_elems:
        raise ValueError(f"Unexpected data length: {len(data_tuple)}")
    row = [0] * len(CSV_COLS)
    row[0] = int(data_tuple[0])
    row[1] = int(data_tuple[1])
    row[2] = int(data_tuple[2])
    row[3] = int(data_tuple[3])
    for i in range(20): row[4 + i] = float(data_tuple[4 + i])
    base = 4 + 20
    for i in range(6): row[24 + i] = int(data_tuple[base + i])
    base = 4 + 20 + 6
    row[30] = int(data_tuple[base + 0])
    row[31] = int(data_tuple[base + 1])
    base = 4 + 20 + 6 + 2
    row[32] = float(data_tuple[base])
    base = 4 + 20 + 6 + 2 + 1
    for i in range(17): row[33 + i] = float(data_tuple[base + i])
    return row

def decode_payload_to_row(payload: bytes, last_good_row=None):
    data_tuple = struct.unpack(STRUCT_FMT, payload)
    return decode_packet(data_tuple)

def row_to_csv_line(row):
    int_indices = {0, 1, 2, 3, 24, 25, 26, 27, 28, 29, 30, 31}
    parts = []
    for idx, val in enumerate(row):
        if idx in int_indices: parts.append(str(int(val)))
        else: parts.append(f"{float(val):.4f}")
    return ",".join(parts)


# ===================== 전역 모던 스타일 =====================
def apply_modern_style(app: QtWidgets.QApplication):
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#f5f7fb"))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor("#ffffff"))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#f3f4f6"))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor("#111827"))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#111827"))
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor("#2563eb"))
    palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor("#ffffff"))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor("#2563eb"))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor("#ffffff"))
    app.setPalette(palette)
    app.setStyleSheet("""
        QWidget { background-color: #f5f7fb; font-family: 'Segoe UI', sans-serif; font-size: 11pt; color: #111827; }
        QMainWindow { background-color: #f5f7fb; }
        QDialog { background-color: #f5f7fb; }
        QLabel { font-size: 10.5pt; }
        QLineEdit, QComboBox { background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 8px; padding: 4px; }
        QPushButton { background-color: #2563eb; color: #ffffff; border-radius: 8px; padding: 6px 14px; border: none; font-weight: 500; }
        QPushButton:hover { background-color: #1d4ed8; }
        QPushButton:disabled { background-color: #9ca3af; color: #e5e7eb; }
        QStatusBar { background-color: #ffffff; border-top: 1px solid #e5e7eb; }
        QGroupBox { border: 1px solid #e5e7eb; border-radius: 10px; margin-top: 8px; background-color: #ffffff; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 4px 10px; }
    """)


# ===================== 2) 시리얼 워커 =====================
class SerialWorker(QtCore.QObject):
    data_received = pyqtSignal(list)
    status_msg = pyqtSignal(str)
    debug_msg = pyqtSignal(str)
    finished = pyqtSignal()
    connection_failed = pyqtSignal(str)
    packet_error_count = pyqtSignal(int)
    sanity_error_count = pyqtSignal(int)

    def __init__(self, port_name, baudrate=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT, initial_skip=0, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self._running = False
        self._error_count = 0
        self._sanity_error_count = 0
        self._consecutive_packet_errors = 0
        self._consecutive_sanity_failures = 0
        self._last_err_log_ts = 0.0
        self._err_log_interval_s = 1.0
        self._err_window_s = 10.0
        self._recent_errors = deque()
        self._last_good_row = None
        self._last_valid_emit_ts = 0.0
        self._last_warn_ts = 0.0
        self._warn_interval_s = 2.0
        self._no_valid_resync_s = 3.0
        self._max_consecutive_packet_errors = 8
        self._max_consecutive_sanity_failures = 3
        self._len_mismatch_seen = 0
        self._initial_skip = max(0, int(initial_skip or 0))
        self._skip_remaining = self._initial_skip

    def _find_sof_index(self, buf: bytearray) -> int:
        if len(buf) < 2: return -1
        return buf.find(SOF_BYTES_LE)

    def _log_packet_debug(self, sof: int, length: int, packet_len: int = None, recv_crc: int = None, calc_crc: int = None):
        now = time.time()
        if (now - self._last_err_log_ts) < self._err_log_interval_s: return
        self._last_err_log_ts = now
        parts = [f"rx_sof=0x{sof:04X}", f"rx_len={length}", f"expected={EXPECTED_TOTAL_PACKET_SIZE}"]
        msg = "Packet debug: " + ", ".join(parts)
        self.status_msg.emit(msg)
        self.debug_msg.emit(msg)

    def _inc_error(self, reason: str = ""):
        self._error_count += 1
        self.packet_error_count.emit(self._error_count)
        if reason:
            msg = f"Packet error #{self._error_count}: {reason}"
            self.status_msg.emit(msg)
            self.debug_msg.emit(msg)

    def _inc_sanity_error(self, reason: str = ""):
        self._sanity_error_count += 1
        self.sanity_error_count.emit(self._sanity_error_count)
        if reason: self.debug_msg.emit(f"Sanity error #{self._sanity_error_count}: {reason}")

    def _flush_serial(self, ser: serial.Serial):
        try: ser.reset_input_buffer()
        except: pass
        try: ser.reset_output_buffer()
        except: pass

    @pyqtSlot()
    def run(self):
        self._running = True
        ser = None
        try:
            try:
                ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout, write_timeout=0, exclusive=True)
                self.status_msg.emit(f"Connected to {self.port_name}")
            except Exception as e:
                self.connection_failed.emit(f"Serial open error: {e}")
                return

            buf = bytearray()
            self._last_valid_emit_ts = time.time()
            self._last_warn_ts = self._last_valid_emit_ts
            self._flush_serial(ser)
            self._skip_remaining = self._initial_skip

            while self._running:
                try:
                    waiting = ser.in_waiting if ser.in_waiting > 0 else 512
                    chunk = ser.read(min(waiting, 2048))
                except: break
                if not chunk: continue
                buf.extend(chunk)

                now = time.time()
                if (now - self._last_valid_emit_ts) > self._no_valid_resync_s and len(buf) > (EXPECTED_TOTAL_PACKET_SIZE*2):
                    buf.clear(); self._flush_serial(ser); self._consecutive_packet_errors=0

                while True:
                    if len(buf) < 4: break
                    sof_idx = self._find_sof_index(buf)
                    if sof_idx < 0:
                        if len(buf)>1: del buf[:-1]
                        break
                    if sof_idx > 0:
                        del buf[:sof_idx]
                        if len(buf)<4: break
                    
                    sof = SOF_VALUE
                    length = buf[2] | (buf[3] << 8)
                    if length not in ALLOWED_TOTAL_PACKET_SIZES:
                        del buf[0]; continue
                    if len(buf) < length: break

                    packet = bytes(buf[:length])
                    del buf[:length]

                    inner_len = packet[2] | (packet[3] << 8)
                    if inner_len != length:
                        self._inc_error("Inner Len Mismatch")
                        continue

                    data_without_crc = packet[:-2]
                    recv_crc = packet[-2] | (packet[-1] << 8)
                    calc_crc = crc16_modbus(data_without_crc)

                    if recv_crc != calc_crc:
                        self._inc_error("CRC Mismatch")
                    else:
                        payload = packet[4:-2]
                        if len(payload) != PAYLOAD_SIZE:
                            self._inc_error("Payload Size Mismatch")
                        else:
                            try:
                                row = decode_payload_to_row(payload, self._last_good_row)
                                ok, why = sanity_check_row(row, self._last_good_row)
                                if not ok:
                                    self._inc_sanity_error(why)
                                    self._consecutive_sanity_failures += 1
                                else:
                                    self._consecutive_sanity_failures = 0
                                    self._last_good_row = row
                                    self._last_valid_emit_ts = time.time()
                                    self._consecutive_packet_errors = 0
                                    if self._skip_remaining > 0:
                                        self._skip_remaining -= 1
                                    else:
                                        self.data_received.emit(row)
                            except Exception as e:
                                self._inc_error(f"Decode/Unpack error: {e}")
        finally:
            if ser and ser.is_open: ser.close()
            self.finished.emit()

    def stop(self): self._running = False


# ===================== 3) CSV 리뷰 뷰어 =====================
class CsvReviewDialog(QtWidgets.QDialog):
    def __init__(self, csv_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"CSV Review — {os.path.basename(csv_path)}")
        self.resize(1000, 800)
        self.csv_path = csv_path
        self.groups = [
            ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"],
            ["LeftHipImuGlobalAccX","LeftHipImuGlobalAccY","LeftHipImuGlobalAccZ"],
            ["LeftHipImuGlobalGyrX","LeftHipImuGlobalGyrY","LeftHipImuGlobalGyrZ"],
            ["LeftHipMotorAngle","RightHipMotorAngle","LeftThighAngle","RightThighAngle"],
            ["s_g_knn_conf","s_scaling_X","s_scaling_Y"],
            ["TswingRecording_ms","T_swing_ms","T_swing_SOS_ms","T_swing_STS_ms"],
            ["is_moving","hc_count","R_count_upeak","L_count_upeak"],
            ["tau_max_setting","s_gait_mode","s_g_knn_conf", "s_vel_HC","s_T_HC_s","s_norm_vel_HC","s_norm_T_HC"],
        ]
        self.col_index = {name: i for i, name in enumerate(CSV_COLS)}
        self.data, self.loop_cnt = None, None
        self._load_csv(csv_path)
        self.checkboxes, self.plots = [], []
        self.grid = None
        self._build_ui()
        self._plot_all()
        self._relayout()

    def _load_csv(self, path):
        arr = np.genfromtxt(path, delimiter=",", skip_header=1)
        if arr.ndim == 1: arr = arr.reshape(1, -1)
        self.data = arr
        self.loop_cnt = self.data[:, 0] if arr.size else []

    def _build_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        top_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(top_layout)
        cb_group = QtWidgets.QGroupBox("Graphs"); cb_layout = QtWidgets.QVBoxLayout(cb_group)
        for i in range(8):
            cb = QtWidgets.QCheckBox(f"Graph {i+1}"); cb.setChecked(True)
            cb.stateChanged.connect(self._on_visibility_changed)
            self.checkboxes.append(cb); cb_layout.addWidget(cb)
        top_layout.addWidget(cb_group)
        self.grid = QtWidgets.QGridLayout(); main_layout.addLayout(self.grid, stretch=1)
        for i in range(8):
            pw = pg.PlotWidget(background="w"); pw.addLegend()
            self.plots.append(pw)

    def _plot_all(self):
        if self.data is None: return
        for i, pw in enumerate(self.plots):
            pw.clear()
            names = self.groups[i]
            for k, name in enumerate(names):
                if name not in self.col_index: continue
                col = self.col_index[name]
                if col < self.data.shape[1]:
                    pw.plot(self.loop_cnt, self.data[:, col], pen=pg.intColor(k), name=name)

    def _on_visibility_changed(self): self._relayout()
    def _relayout(self):
        while self.grid.count(): self.grid.takeAt(0).widget().setParent(None)
        vis = [p for i, p in enumerate(self.plots) if self.checkboxes[i].isChecked()]
        for i, p in enumerate(vis): self.grid.addWidget(p, i//2, i%2)
        for p in self.plots: p.setVisible(p in vis)


# ===================== 4) 메인 GUI =====================

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("USB CDC Real-time Monitor & Auto Logger (4 Plots)")
        
        # [고정] 800x800 정사각형 크기
        self.resize(800, 800)
        
        # [고정] 화면 정중앙 배치
        self._center_window()

        self.data_all = []
        self._recv_count = 0
        self._last_hc_count = None
        self._hc_points_x = []
        self._hc_points_y = []
        self._hc_points_brush = []
        self._hc_scatter_item = None
        self._ts_items = []
        self._connection_mode = None
        self.default_idx = CSV_COLS.index("LeftHipAngle")
        self.serial_thread = None
        self.serial_worker = None
        self.log_file = None
        self._written_lines = 0
        self._last_log_path = None
        self._pending_lines = []
        self.plot_groups = []
        self._build_plot_groups()
        self.state_labels = {}
        self.is_moving_indicator = None
        self.debug_label = None
        self._last_rx_len = None
        self._recent_err_summary = None

        self._build_ui()
        self.refresh_ports()

        self._knn_bg_item = None
        self._add_knn_boundary_background()

        # QTimer로 일정 주기마다 플롯 갱신 (과거 방식과 동일)
        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.timeout.connect(self.update_all_plots)
        self._plot_timer.start(UPDATE_INTERVAL_MS)

    def _center_window(self):
        qr = self.frameGeometry()
        cp = QtWidgets.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def _add_knn_boundary_background(self):
        """KNN 배경 이미지 생성 및 Plot 4 축 범위 강제 고정 (디버그 출력 제거 버전)"""
        try:
            # 1. KNN 데이터셋 로드 (출력 없이 로직만 수행)
            knn_py_path = os.path.join(os.path.dirname(__file__), 'KNN_ref_dataset_251223_python.txt')
            knn_globals = {}
            with open(knn_py_path, 'r', encoding='utf-8') as f:
                exec(f.read(), {"np": np}, knn_globals)
            
            ref_xy = knn_globals['ref_xy']
            split = knn_globals['split']
            k = knn_globals['k']

            if ref_xy.shape[0] == 0:
                return

            # 2. KNN 분류 알고리즘 (테스트 출력부 삭제)
            def knn_classify(x, y):
                diff = ref_xy - np.array([x, y], dtype=float)
                d = np.sqrt(np.sum(diff * diff, axis=1))
                sorted_indices = np.argsort(d)
                k_indices = sorted_indices[:k]
                k_dists = d[k_indices]

                zero_mask = (k_dists == 0.0)
                if np.any(zero_mask):
                    zero_indices = k_indices[zero_mask]
                    return 1 if np.any(zero_indices < split) else 2

                weights = 1.0 / k_dists
                score_sts = 0.0
                score_sos = 0.0
                for idx, w in zip(k_indices, weights):
                    if idx < split: score_sts += w
                    else: score_sos += w
                
                if abs(score_sts - score_sos) < 1e-12: return 1
                return 1 if score_sts > score_sos else 2

            # 3. 배경 이미지 행렬 생성
            res = 400
            x_grid = np.linspace(0.0, 1.5, res)
            y_grid = np.linspace(0.0, 1.5, res)
            xx, yy = np.meshgrid(x_grid, y_grid)
            zz = np.zeros_like(xx, dtype=np.uint8)
            for i in range(res):
                for j in range(res):
                    zz[i, j] = knn_classify(xx[i, j], yy[i, j])

            colors = np.empty(xx.shape + (4,), dtype=np.ubyte)
            colors[zz == 1] = [220, 38, 38, 60]   # STS (Red)
            colors[zz == 2] = [37, 99, 235, 60]   # SOS (Blue)

            img = pg.ImageItem(colors, axisOrder='row-major')
            img.setRect(QtCore.QRectF(0.0, 0.0, 1.5, 1.5))
            img.setZValue(-10)

            pw = self.plot_widgets[3]
            if self._knn_bg_item is not None:
                pw.removeItem(self._knn_bg_item)
            pw.addItem(img)
            self._knn_bg_item = img

            # 4. 축 설정: 비율 고정을 해제하여 Y축 잘림 방지 및 0,0 원점 고정
            pw.setAspectLocked(False) 
            pw.getViewBox().disableAutoRange() 
            pw.setXRange(0.0, 1.5, padding=0)
            pw.setYRange(0.0, 1.5, padding=0)
            pw.getAxis('left').enableAutoSIPrefix(False)
            pw.getAxis('bottom').enableAutoSIPrefix(False)
            pw.getViewBox().setLimits(xMin=0.0, xMax=1.5, yMin=0.0, yMax=1.5)
            pw.setMouseEnabled(x=False, y=False)

        except Exception as e:
            # 에러 발생 시에만 최소한의 내용 출력
            print(f"KNN boundary background error: {e}")

    def _update_toggle_buttons(self):
        logging_active = self._connection_mode == "logging"
        monitor_active = self._connection_mode == "monitor"

        if logging_active:
            self.btn_connect.setText("Disconnect & Stop")
            self.btn_connect.setEnabled(True)
        else:
            self.btn_connect.setText("Connect & Start Logging")
            self.btn_connect.setEnabled(not monitor_active)

        if monitor_active:
            self.btn_monitor_only.setText("Disconnect & Stop")
            self.btn_monitor_only.setEnabled(True)
        else:
            self.btn_monitor_only.setText("Connect & Monitor Only")
            self.btn_monitor_only.setEnabled(not logging_active)

    def _reset_runtime_buffers(self):
        self.data_all = []
        self._recv_count = 0
        self._last_hc_count = None
        self._hc_points_x = []
        self._hc_points_y = []
        self._hc_points_brush = []
        for items in getattr(self, "_ts_items", []):
            for item in items.values():
                item.setData([], [])
        if self._hc_scatter_item is not None:
            self._hc_scatter_item.setData([])

        self.mode_label.setText("H10Mode: -")
        self.error_label.setText("Packet errors: 0")
        if getattr(self, "sanity_error_label", None) is not None:
            self.sanity_error_label.setText("Sanity errors: 0")
        if self.is_moving_indicator is not None:
            self._set_is_moving_indicator(0)
        for key, lbl in self.state_labels.items():
            lbl.setText(f"{key}: -")
        self.status_bar.showMessage("Ready")

    def _ensure_csv_extension(self, filename: str) -> str:
        name = (filename or "").strip()
        if not name: return ""
        base, ext = os.path.splitext(name)
        if ext.lower() != ".csv":
            name = base if ext else name
            return f"{name}.csv"
        return base + ".csv"

    def _increment_filename_suffix(self):
        current = (self.edit_filename.text() or "").strip()
        if not current: return
        name = self._ensure_csv_extension(current)
        base, ext = os.path.splitext(name)
        match = re.search(r"^(.*?)(\d+)$", base)
        if not match:
            self.edit_filename.setText(name)
            return
        prefix, digits = match.groups()
        width = len(digits)
        next_number = str(int(digits) + 1).zfill(width)
        new_name = f"{prefix}{next_number}{ext}"
        self.edit_filename.setText(new_name)

    def _set_is_moving_indicator(self, is_moving: int):
        if self.is_moving_indicator is None: return
        if int(is_moving) != 0:
            self.is_moving_indicator.setStyleSheet("color: #2563eb;")
        else:
            self.is_moving_indicator.setStyleSheet("color: #dc2626;")

    def _gait_mode_to_brush(self, gait_mode: int) -> QtGui.QBrush:
        gm = int(gait_mode)
        RSOS, RSTS, LSOS, LSTS = 1, 2, 3, 4
        if gm in (RSTS, LSTS): return QtGui.QBrush(QtGui.QColor("#dc2626"))
        if gm in (RSOS, LSOS): return QtGui.QBrush(QtGui.QColor("#2563eb"))
        return QtGui.QBrush(QtGui.QColor("#6b7280"))

    def _build_plot_groups(self):
        g1 = ["LeftThighAngle", "RightThighAngle"]
        g2 = ["LeftHipTorque", "RightHipTorque"]
        g3 = ["TswingRecording_ms"]
        g4 = []
        self.plot_groups = [g1, g2, g3, g4]

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        vbox.addLayout(top)
        top.addWidget(QtWidgets.QLabel("Serial Port:"))
        self.combo_port = QtWidgets.QComboBox()
        top.addWidget(self.combo_port)
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top.addWidget(self.btn_refresh)
        self.btn_connect = QtWidgets.QPushButton("Connect & Start Logging")
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.btn_connect)
        self.btn_monitor_only = QtWidgets.QPushButton("Connect & Monitor Only")
        self.btn_monitor_only.clicked.connect(self.on_monitor_only_clicked)
        top.addWidget(self.btn_monitor_only)
        self.btn_disconnect = QtWidgets.QPushButton("Disconnect & Stop")
        self.btn_disconnect.clicked.connect(self.on_disconnect_clicked)
        self.btn_disconnect.setEnabled(False)
        top.addWidget(self.btn_disconnect)
        top.addStretch()

        default_folder = os.path.abspath(os.path.join(os.getcwd(), "data"))
        top.addWidget(QtWidgets.QLabel("Output folder:"))
        self.edit_folder = QtWidgets.QLineEdit(default_folder)
        self.edit_folder.setMinimumWidth(200)
        top.addWidget(self.edit_folder)
        self.btn_browse_folder = QtWidgets.QPushButton("Folder…")
        self.btn_browse_folder.clicked.connect(self.on_browse_folder_clicked)
        top.addWidget(self.btn_browse_folder)

        top.addWidget(QtWidgets.QLabel("Output file:"))
        default_name = "251223_cyk_assist3_SOS_trial1"
        self.edit_filename = QtWidgets.QLineEdit(default_name)
        self.edit_filename.setMinimumWidth(200)
        top.addWidget(self.edit_filename)
        self.btn_browse = QtWidgets.QPushButton("Browse…")
        self.btn_browse.clicked.connect(self.on_browse_clicked)
        top.addWidget(self.btn_browse)

        content = QtWidgets.QHBoxLayout()
        vbox.addLayout(content, stretch=1)

        grid = QtWidgets.QGridLayout()
        content.addLayout(grid, stretch=1)
        
        titles = [
            "Thigh angle",
            "Assistance torque profile",
            "T_swing at HC",
            "KNN classified gait mode"
        ]

        self.plot_widgets = []
        for i in range(4):
            pw = pg.PlotWidget()
            pw.setBackground("w")
            pw.showGrid(x=True, y=True, alpha=0.15)
            axis_pen = pg.mkPen("#9ca3af")
            pw.getAxis("bottom").setPen(axis_pen)
            pw.getAxis("left").setPen(axis_pen)
            pw.getAxis("bottom").setTextPen("#4b5563")
            pw.getAxis("left").setTextPen("#4b5563")
            pw.setLabel("bottom", "LoopCnt")
            pw.setLabel("left", "Value")
            pw.setTitle(titles[i])
            self.plot_widgets.append(pw)
            r = i // 2; c = i % 2
            grid.addWidget(pw, r, c)

        self._ts_items = []
        for plot_idx in range(3):
            pw = self.plot_widgets[plot_idx]
            pw.addLegend()
            group = self.plot_groups[plot_idx]
            items = {}
            hues = max(8, len(group))
            for k, name in enumerate(group):
                color = pg.intColor(k, hues=hues)
                item = pw.plot([], [], pen=color, name=name)
                items[name] = item
            self._ts_items.append(items)

        self.plot_widgets[0].setYRange(-30.0, 90.0, padding=0.0)
        self.plot_widgets[1].setYRange(-1.0, 5.0, padding=0.0)
        self.plot_widgets[2].setYRange(0.0, 1000.0, padding=0.0)

        # -----------------------------------------------------------------
        # [고정] Plot 4 설정: 0~1.5 범위 고정, 1:1 비율 유지
        # SI prefix(×0.001 같은 표시) 끄기
        # -----------------------------------------------------------------
        pw4 = self.plot_widgets[3]
        pw4.setLabel("bottom", "s_norm_vel_HC")
        pw4.setLabel("left", "s_norm_T_HC")

        # 축 자동 SI prefix 비활성화: 0.3을 300m처럼 표시하지 않도록 함
        pw4.getAxis('left').enableAutoSIPrefix(False)
        pw4.getAxis('bottom').enableAutoSIPrefix(False)

        # Plot 4 초기화: 가장 기본적인 좌표계 설정으로 복구
        pw4.getViewBox().disableAutoRange()
        pw4.setAspectLocked(False) # 비율 고정 해제
        pw4.setXRange(0.0, 1.5, padding=0)
        pw4.setYRange(0.0, 1.5, padding=0)
        
        pw4.setMouseEnabled(x=False, y=False)
        pw4.hideButtons()
        pw4.getViewBox().setLimits(xMin=0.0, xMax=1.5, yMin=0.0, yMax=1.5)

        self._hc_scatter_item = pg.ScatterPlotItem()
        self.plot_widgets[3].addItem(self._hc_scatter_item)
        self._scatter_legend = self.plot_widgets[3].addLegend()
        self._scatter_legend_items = []
        for label, color in (("STS", "#dc2626"), ("SOS", "#2563eb")):
            dummy_item = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(color), pen=pg.mkPen(color))
            self._scatter_legend.addItem(dummy_item, label)
            self._scatter_legend_items.append(dummy_item)

        # plot4 넘버링용 텍스트 아이템 리스트
        self._hc_text_items = []

        state_box = QtWidgets.QGroupBox("States")
        state_layout = QtWidgets.QVBoxLayout(state_box)
        state_layout.setContentsMargins(12, 12, 12, 12)
        state_layout.setSpacing(10)
        content.addWidget(state_box, stretch=0)

        self.mode_label = QtWidgets.QLabel("H10Mode: -")
        font = self.mode_label.font(); font.setPointSize(font.pointSize() + 2); font.setBold(True)
        self.mode_label.setFont(font)
        state_layout.addWidget(self.mode_label)

        self.error_label = QtWidgets.QLabel("Packet errors: 0")
        self.error_label.setFont(font)
        state_layout.addWidget(self.error_label)

        self.sanity_error_label = QtWidgets.QLabel("Sanity errors: 0")
        self.sanity_error_label.setFont(font)
        state_layout.addWidget(self.sanity_error_label)

        self.debug_label = QtWidgets.QLabel(f"Expected total: {EXPECTED_TOTAL_PACKET_SIZE} bytes\nExpected payload: {PAYLOAD_SIZE} bytes\nLast packet: -")
        self.debug_label.setWordWrap(True)
        self.debug_label.setStyleSheet("color: #374151;")
        state_layout.addWidget(self.debug_label)

        moving_row = QtWidgets.QHBoxLayout(); moving_row.setSpacing(10)
        moving_row.addWidget(QtWidgets.QLabel("is_moving:"))
        self.is_moving_indicator = QtWidgets.QLabel("●")
        ind_font = self.is_moving_indicator.font(); ind_font.setPointSize(ind_font.pointSize() + 8)
        self.is_moving_indicator.setFont(ind_font)
        self.is_moving_indicator.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        moving_row.addWidget(self.is_moving_indicator); moving_row.addStretch()
        state_layout.addLayout(moving_row)

        for key in ("h10Mode", "tau_max_setting", "s_scaling_X", "s_scaling_Y", "hc_count"):
            lbl = QtWidgets.QLabel(f"{key}: -")
            lbl.setFont(font)
            state_layout.addWidget(lbl)
            self.state_labels[key] = lbl

        state_layout.addStretch()
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")

    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports: self.combo_port.addItem(p.device)
        if not ports: self.combo_port.addItem("(no ports)")
        self.status_bar.showMessage("Port list refreshed")

    def _open_log_file(self):
        filename_input = (self.edit_filename.text() or "").strip()
        if not filename_input: filename_input = "251218_cyk_SOS_80bpm_trial1"
        normalized_name = self._ensure_csv_extension(filename_input)
        name_only = os.path.basename(normalized_name)
        implied_folder = os.path.dirname(normalized_name)
        folder_field = (self.edit_folder.text() or "").strip()
        folder = implied_folder if implied_folder else folder_field
        self.edit_folder.setText(folder)
        self.edit_filename.setText(name_only)
        full_path = os.path.join(folder, name_only) if folder else name_only
        full_path = os.path.abspath(full_path)
        dirpath = os.path.dirname(full_path)
        if dirpath and not os.path.exists(dirpath): os.makedirs(dirpath, exist_ok=True)
        self.log_file = open(full_path, "w", encoding="utf-8", newline="")
        self.log_file.write(CSV_HEADER + "\n")
        self._written_lines = 0
        self._pending_lines = []
        self._last_log_path = full_path
        self.status_bar.showMessage(f"Logging to: {full_path} (flush_every={FLUSH_EVERY})")

    def _flush_pending_lines(self):
        if self.log_file and self._pending_lines:
            try: self.log_file.writelines(self._pending_lines)
            except Exception as e: self.status_bar.showMessage(f"File write error (flush): {e}")
            self._pending_lines.clear()

    def _close_log_file(self):
        if self.log_file:
            try: self._flush_pending_lines(); self.log_file.flush(); self.log_file.close()
            except: pass
            self.log_file = None
            self.status_bar.showMessage("Log file closed")

    def _find_latest_csv(self):
        if self._last_log_path and os.path.isfile(self._last_log_path): return self._last_log_path
        pattern = os.path.join(os.getcwd(), "cdc_monitoring_data_log_*.csv")
        files = glob.glob(pattern)
        if not files: return None
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return files[0]

    def _open_review_viewer(self):
        path = self._find_latest_csv()
        if not path:
            QtWidgets.QMessageBox.information(self, "Info", "No CSV file found to review.")
            return
        dlg = CsvReviewDialog(path, self)
        dlg.setModal(False)
        dlg.show()

    def on_browse_folder_clicked(self):
        current = self.edit_folder.text().strip() or os.getcwd()
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Select output folder", current)
        if folder: self.edit_folder.setText(folder)

    def on_browse_clicked(self):
        base_folder = self.edit_folder.text().strip() or os.getcwd()
        initial_name = self.edit_filename.text().strip() or "cdc_monitoring_data_log.csv"
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Select output CSV file", os.path.join(base_folder, initial_name), "CSV Files (*.csv);;All Files (*)")
        if fname:
            normalized = self._ensure_csv_extension(fname)
            folder = os.path.dirname(normalized)
            self.edit_folder.setText(folder)
            self.edit_filename.setText(os.path.basename(normalized))

    def on_connect_clicked(self):
        if self._connection_mode == "logging": self.on_disconnect_clicked(); return
        port_name = self.combo_port.currentText()
        if not port_name or port_name == "(no ports)":
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port selected.")
            return
        if self.serial_thread is not None:
            QtWidgets.QMessageBox.information(self, "Info", "Already connected.")
            return
        try: self._open_log_file()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open log file:\n{e}")
            return
        self.serial_thread = QtCore.QThread()
        self.serial_worker = SerialWorker(port_name, DEFAULT_BAUD, DEFAULT_TIMEOUT, initial_skip=5)
        self.serial_worker.moveToThread(self.serial_thread)
        self.serial_thread.started.connect(self.serial_worker.run)
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.status_msg.connect(self.on_status_msg)
        self.serial_worker.debug_msg.connect(self.on_debug_msg)
        self.serial_worker.finished.connect(self.on_serial_finished)
        self.serial_worker.connection_failed.connect(self.on_connection_failed)
        self.serial_worker.packet_error_count.connect(self.on_packet_error_count)
        self.serial_worker.sanity_error_count.connect(self.on_sanity_error_count)
        self.serial_thread.start()
        self._connection_mode = "logging"
        self._update_toggle_buttons()
        self.btn_disconnect.setEnabled(True)
        self.status_bar.showMessage(f"Connecting to {port_name} ...")
        self.error_label.setText("Packet errors: 0")
        self.sanity_error_label.setText("Sanity errors: 0")

    def on_monitor_only_clicked(self):
        if self._connection_mode == "monitor": self.on_disconnect_clicked(); return
        port_name = self.combo_port.currentText()
        if not port_name or port_name == "(no ports)":
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port selected.")
            return
        if self.serial_thread is not None:
            QtWidgets.QMessageBox.information(self, "Info", "Already connected.")
            return
        self.log_file = None; self._pending_lines = []; self._written_lines = 0; self._last_log_path = None
        self.serial_thread = QtCore.QThread()
        self.serial_worker = SerialWorker(port_name, DEFAULT_BAUD, DEFAULT_TIMEOUT, initial_skip=5)
        self.serial_worker.moveToThread(self.serial_thread)
        self.serial_thread.started.connect(self.serial_worker.run)
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.status_msg.connect(self.on_status_msg)
        self.serial_worker.debug_msg.connect(self.on_debug_msg)
        self.serial_worker.finished.connect(self.on_serial_finished)
        self.serial_worker.connection_failed.connect(self.on_connection_failed)
        self.serial_worker.packet_error_count.connect(self.on_packet_error_count)
        self.serial_worker.sanity_error_count.connect(self.on_sanity_error_count)
        self.serial_thread.start()
        self._connection_mode = "monitor"
        self._update_toggle_buttons()
        self.btn_disconnect.setEnabled(True)
        self.status_bar.showMessage(f"Connecting to {port_name} (monitor only) ...")
        self.error_label.setText("Packet errors: 0")
        self.sanity_error_label.setText("Sanity errors: 0")

    def on_disconnect_clicked(self):
        if self.serial_worker is not None:
            self.btn_connect.setEnabled(False)
            self.btn_monitor_only.setEnabled(False)
            self.btn_disconnect.setEnabled(False)
            self.serial_worker.stop()

    @pyqtSlot(list)
    def on_data_received(self, row):
        self.data_all.append(row)
        self._recv_count += 1
        if self.log_file:
            try:
                line = row_to_csv_line(row) + "\n"
                self._pending_lines.append(line); self._written_lines += 1
                if len(self._pending_lines) >= FLUSH_EVERY: self._flush_pending_lines()
            except Exception as e: self.status_bar.showMessage(f"File write error: {e}")
        try: self.mode_label.setText(f"H10Mode: {int(row[1])}")
        except: pass
        try: 
            if "h10Mode" in self.state_labels: self.state_labels["h10Mode"].setText(f"h10Mode: {int(row[1])}")
        except: pass
        try:
            if "tau_max_setting" in self.state_labels: 
                idx = CSV_COLS.index("tau_max_setting")
                self.state_labels["tau_max_setting"].setText(f"tau_max_setting: {int(row[idx])}")
        except: pass
        try:
            if "s_scaling_X" in self.state_labels:
                idx = CSV_COLS.index("s_scaling_X")
                self.state_labels["s_scaling_X"].setText(f"s_scaling_X: {float(row[idx]):.4f}")
        except: pass
        try:
            if "s_scaling_Y" in self.state_labels:
                idx = CSV_COLS.index("s_scaling_Y")
                self.state_labels["s_scaling_Y"].setText(f"s_scaling_Y: {float(row[idx]):.4f}")
        except: pass
        try:
            if "hc_count" in self.state_labels:
                idx = CSV_COLS.index("hc_count")
                self.state_labels["hc_count"].setText(f"hc_count: {int(row[idx])}")
        except: pass
        try:
            is_moving_idx = CSV_COLS.index("is_moving")
            self._set_is_moving_indicator(int(row[is_moving_idx]))
        except: pass
        # plot4: 배경(KNN 분류 이미지)은 그대로 유지, 점 추가는 hc_count 증가 시점에만 (과거 방식과 동일)
        try:
            hc_idx = CSV_COLS.index("hc_count")
            hc_val = int(row[hc_idx])
            if self._last_hc_count is None:
                self._last_hc_count = hc_val
            elif hc_val > self._last_hc_count:
                self._last_hc_count = hc_val
                x = float(row[CSV_COLS.index("s_norm_vel_HC")])
                y = float(row[CSV_COLS.index("s_norm_T_HC")])
                gm = int(row[CSV_COLS.index("s_gait_mode")])
                self._hc_points_x.append(x)
                self._hc_points_y.append(y)
                self._hc_points_brush.append(self._gait_mode_to_brush(gm))
                # 최근 N개만 유지
                if SCATTER_KEEP_LAST_N > 0:
                    self._hc_points_x = self._hc_points_x[-SCATTER_KEEP_LAST_N:]
                    self._hc_points_y = self._hc_points_y[-SCATTER_KEEP_LAST_N:]
                    self._hc_points_brush = self._hc_points_brush[-SCATTER_KEEP_LAST_N:]
        except Exception:
            pass
        try:
            v = row[self.default_idx]
            self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}, LeftHipAngle={v:.4f}")
        except: self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}")

    def update_all_plots(self):
        if not self.data_all: return
        rows = self.data_all[-WINDOW_SIZE_SAMPLES:]
        if not rows: return
        x_vals = [r[0] for r in rows]

        for plot_idx in range(4):
            pw = self.plot_widgets[plot_idx]
            if plot_idx < 3:
                group = self.plot_groups[plot_idx]
                items = self._ts_items[plot_idx] if plot_idx < len(self._ts_items) else {}
                for name in group:
                    try: col_idx = CSV_COLS.index(name)
                    except ValueError: continue
                    y_vals = [r[col_idx] for r in rows]
                    item = items.get(name)
                    if item is not None: item.setData(x_vals, y_vals)
                if x_vals:
                    pw.setXRange(min(x_vals), max(x_vals), padding=0.01)
            else:
                # Plot 4: 불필요한 연산 제거 및 0~1.5 범위 유지
                pw.getViewBox().disableAutoRange()
                pw.setXRange(0.0, 1.5, padding=0)
                pw.setYRange(0.0, 1.5, padding=0)
                # 매 프레임마다 비율 고정이 다시 켜지지 않도록 함
                pw.setAspectLocked(False)

        if self._hc_scatter_item is not None:
            spots = []
            total_pts = len(self._hc_points_x)
            # 넘버링용 텍스트 모두 제거 (다시 추가)
            for t in getattr(self, '_hc_text_items', []):
                self.plot_widgets[3].removeItem(t)
            self._hc_text_items = []
            # 넘버링: 첫번째는 1, n번째는 n
            for idx, (x_raw, y_raw, br) in enumerate(zip(self._hc_points_x, self._hc_points_y, self._hc_points_brush)):
                color = br.color() if isinstance(br, QtGui.QBrush) else QtGui.QColor("#6b7280")
                is_latest = (idx == total_pts - 1)
                # 클리핑 및 네모 처리
                x, y = x_raw, y_raw
                symbol = "star" if is_latest else "o"
                clipped = False
                if x < 0.0:
                    x = 0.0
                    clipped = True
                elif x > 1.5:
                    x = 1.5
                    clipped = True
                if y < 0.0:
                    y = 0.0
                    clipped = True
                elif y > 1.5:
                    y = 1.5
                    clipped = True
                if clipped:
                    symbol = "s"  # 네모
                spot = {
                    "pos": (x, y),
                    "brush": br,
                    "pen": pg.mkPen(color),
                    "size": 12 if is_latest else 8,
                    "symbol": symbol,
                }
                spots.append(spot)
                # 넘버링 텍스트: 1, 2, 3, ...
                text = str(idx + 1)
                text_item = pg.TextItem(text, color=color, anchor=(0.5, 1.2), border=None, fill=None)
                text_item.setFont(QtGui.QFont("Segoe UI", 10, QtGui.QFont.Bold))
                text_item.setPos(x, y)
                self.plot_widgets[3].addItem(text_item)
                self._hc_text_items.append(text_item)
            self._hc_scatter_item.setData(spots)

    @pyqtSlot(str)
    def on_status_msg(self, msg): self.status_bar.showMessage(msg)

    @pyqtSlot(str)
    def on_debug_msg(self, msg):
        if self.debug_label is None: return
        try:
            if isinstance(msg, str) and msg.startswith("Recent 10s errors:"): self._recent_err_summary = msg
        except: pass
        try:
            import re
            m = re.search(r"rx_len=(\d+)", msg)
            if m: self._last_rx_len = int(m.group(1))
        except: pass
        expected_total = EXPECTED_TOTAL_PACKET_SIZE
        expected_payload = PAYLOAD_SIZE
        expected_total_from_payload = expected_payload + 6
        lines = [
            f"Expected total: {expected_total} bytes",
            f"Expected payload: {expected_payload} bytes",
            f"(Payload->Total): {expected_total_from_payload} bytes",
        ]
        if self._recent_err_summary: lines.append(self._recent_err_summary)
        if self._last_rx_len is not None:
            rx = self._last_rx_len
            lines.append(f"Last rx_len: {rx} bytes")
        lines.append(f"Last: {msg}")
        self.debug_label.setText("\n".join(lines))

    @pyqtSlot(str)
    def on_connection_failed(self, msg):
        self._close_log_file()
        self._connection_mode = None
        self._update_toggle_buttons()
        self.btn_disconnect.setEnabled(False)
        self._reset_runtime_buffers()
        QtWidgets.QMessageBox.critical(self, "Serial Connection Failed", msg)

    @pyqtSlot()
    def on_serial_finished(self):
        if self.serial_thread is not None:
            self.serial_thread.quit(); self.serial_thread.wait(); self.serial_thread = None
        self.serial_worker = None
        self._close_log_file()
        self._connection_mode = None
        self._update_toggle_buttons()
        self.btn_disconnect.setEnabled(False)
        self.status_bar.showMessage("Disconnected")
        if self._last_log_path: self._open_review_viewer()
        self._increment_filename_suffix()
        self._reset_runtime_buffers()

    @pyqtSlot(int)
    def on_packet_error_count(self, n): self.error_label.setText(f"Packet errors: {n}")

    @pyqtSlot(int)
    def on_sanity_error_count(self, n):
        if getattr(self, "sanity_error_label", None) is not None: self.sanity_error_label.setText(f"Sanity errors: {n}")

    def closeEvent(self, event):
        if self.serial_worker is not None: self.serial_worker.stop()
        if self.serial_thread is not None: self.serial_thread.quit(); self.serial_thread.wait(); self.serial_thread = None
        self._close_log_file()
        event.accept()

# ===================== 5) 엔트리 포인트 =====================
def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_modern_style(app)
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "#111827")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()