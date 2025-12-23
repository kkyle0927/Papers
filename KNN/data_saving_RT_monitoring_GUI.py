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

    def _center_window(self):
        qr = self.frameGeometry()
        cp = QtWidgets.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def _add_knn_boundary_background(self):
        try:
            ref_xy = np.array([
                [0.056283044, 0.812457471], [0.216433485, 0.533504801], [0.103343184, 0.553624233], [0.167350235, 0.622434797],
                [0.123376792, 0.506737728], [0.146298521, 0.679953451], [0.085075245, 0.900155400], [0.214617293, 0.603675645],
                [0.165249936, 0.548319035], [0.236336795, 0.506228119], [0.050060151, 0.610580211], [0.058809125, 0.939578264],
                [0.148559605, 0.580091132], [0.180091609, 0.615861133], [0.070157844, 0.853805093], [0.125425607, 0.520688969],
                [0.105087971, 0.621674323], [0.155711479, 0.622315684], [0.045311147, 0.651765762], [0.300033386, 0.464940947],
                [0.164841087, 0.631182669], [0.039776525, 0.690191432], [0.007588241, 0.886591070], [0.064209842, 0.573318592],
                [0.048555530, 0.531371696], [0.168984756, 0.661884890], [0.074505037, 0.833850517], [0.203398496, 0.482770626],
                [0.151297539, 0.757309133], [0.016573101, 0.915398484], [0.188927412, 0.610781600], [0.118861799, 0.581747257],
                [0.267921206, 0.534712372], [0.117619267, 0.641928007], [0.095974685, 0.512466321], [0.116986367, 0.579734290],
                [0.202095399, 0.605154518], [0.065373111, 0.748502628], [0.164439079, 0.570198105], [0.151505817, 0.501254759],
                [0.165164802, 0.495408422], [0.065658004, 0.780340303], [0.165120346, 0.721899408], [0.218973395, 0.461209996],
                [0.156936734, 0.653684279], [0.065286755, 0.717222645], [0.141795555, 0.677595112], [0.150391074, 0.567558299],
                [0.176384319, 0.476937112], [0.091218450, 0.574652778], [0.117815561, 0.793374048], [0.148848268, 0.921301908],
                [0.084426434, 0.933724944], [0.117088615, 0.868909538], [0.161766136, 0.696835467], [0.179750043, 0.860114123],
                [0.152964144, 0.861422979], [0.121100364, 0.864434948], [0.319304826, 0.702353395], [0.166964727, 0.775858046],
                [0.114514668, 0.870125585], [0.173580187, 0.753951984], [0.164953401, 0.682117759], [0.151768409, 0.920655834],
                [0.134745511, 0.925589437], [0.194053616, 0.711930941], [0.085563075, 0.915187757], [0.134297015, 0.736156500],
                [0.119795816, 0.746557455], [0.075635815, 0.922612212], [0.113677117, 0.914742497], [0.153371458, 0.917359536],
                [0.160544194, 0.829508283], [0.008788089, 0.772924789], [0.097836149, 0.896313767], [0.110457627, 0.800192461],
                [0.170293915, 0.709928855], [0.110959319, 0.806743582], [0.122782900, 0.946774003], [0.084865359, 0.677731381],
                [0.112365015, 0.906119163], [0.066148453, 0.933318092], [0.117533334, 0.919886698], [-0.014705270, 0.956741133],
                [0.242036896, 0.733177682], [0.111218501, 0.935407022], [0.096446113, 0.960829574], [0.219067898, 0.717349715],
                [0.186017175, 0.726863171], [0.111505647, 0.883082235], [0.079919453, 0.937281991], [0.202517777, 0.751407159],
                [0.086322895, 0.934591268], [0.142600709, 0.644611772], [0.069964816, 0.953859253], [0.132558252, 0.947143937],
                [0.148042932, 0.928731762], [0.011679818, 1.000000000], [0.124148566, 0.874728829], [0.118727533, 0.922026118],
                [0.734182128, 0.424845381], [0.620250138, 0.363237311], [0.714320293, 0.382263555], [0.663319745, 0.343509084],
                [0.821432234, 0.407900338], [0.715174489, 0.358162672], [0.903727407, 0.422515623], [0.934068377, 0.426232524],
                [0.690639389, 0.379198340], [0.800497469, 0.440803612], [0.722050329, 0.364564665], [0.983725123, 0.437183488],
                [0.488451025, 0.397724060], [0.756185858, 0.378276892], [0.840396523, 0.407622918], [0.928220485, 0.431695892],
                [0.840759770, 0.349156118], [0.535232107, 0.369312583], [0.792495782, 0.442410507], [0.712959097, 0.405203882],
                [0.732892781, 0.333390889], [0.800761489, 0.347991950], [0.682929958, 0.353316453], [0.852332718, 0.447560259],
                [0.803633844, 0.504764615], [0.796253323, 0.388777435], [0.785497455, 0.407213157], [0.672187898, 0.479097488],
                [0.859462985, 0.448956430], [1.000000000, 0.391155044], [0.883073021, 0.509606210], [0.860628505, 0.396132527],
                [0.948673882, 0.454046639], [0.730215282, 0.298802752], [0.900205868, 0.324414980], [0.918059731, 0.390067340],
                [0.921679503, 0.330672808], [0.797655251, 0.324817980], [0.706006253, 0.374529865], [0.888629755, 0.466934426],
                [0.662300801, 0.326369631], [0.852102576, 0.436751224], [0.843862285, 0.460910135], [0.814429464, 0.342009157],
                [0.669387499, 0.345274035], [0.593016460, 0.391065976], [0.752207827, 0.416554178], [0.679356050, 0.413403693],
                [0.745391190, 0.341359519], [0.901237992, 0.375648508], [0.669138555, 0.445055617], [0.594019640, 0.357673501],
                [0.820970905, 0.364379880], [0.752417142, 0.342611412], [0.682912124, 0.500786734], [0.698526602, 0.373070169],
                [0.782166028, 0.561100305], [0.597899597, 0.412822456], [0.803921277, 0.348322682], [0.605410999, 0.455305083],
                [0.665630343, 0.406502489], [0.570267511, 0.461156441], [0.648099226, 0.326810648], [0.755196417, 0.415286560],
                [0.686861845, 0.340534979], [0.539176513, 0.378201377], [0.803520557, 0.341308923], [0.660043235, 0.519137241],
                [0.506834894, 0.400488321], [0.776665463, 0.359883558], [0.557482589, 0.457371249], [0.628852818, 0.411566816],
                [0.655200350, 0.398125454], [0.586488000, 0.390698772], [0.554367927, 0.460670662], [0.573641337, 0.429675018],
                [0.744173662, 0.370392312], [0.573718142, 0.468576132], [0.888946588, 0.489472256], [0.590575951, 0.350577566],
                [0.686857784, 0.436310442], [0.714857238, 0.344036624], [0.814969958, 0.385466678], [0.748335296, 0.408255004],
                [0.816424237, 0.357561728], [0.896372924, 0.373212578], [0.892950904, 0.352088845], [0.732521864, 0.426458462],
                [0.849154605, 0.354324248], [0.841388998, 0.489108156], [0.884947243, 0.427692013], [0.827210466, 0.332781994],
                [0.714442384, 0.403684447], [0.949002133, 0.486408140], [0.710084739, 0.433956080], [0.666811841, 0.356992060],
                [0.710584151, 0.361434238], [0.823661690, 0.369028723], [0.720436661, 0.400038986], [0.766927491, 0.364572743]
            ])
            split = 100
            k = 9
            def knn_classify(x, y):
                d2 = np.sum((ref_xy - np.array([x, y]))**2, axis=1)
                idx = np.argsort(d2)[:k]
                labs = np.where(idx < split, 1, 2)
                c1 = np.sum(labs == 1)
                c2 = np.sum(labs == 2)
                return 1 if c1 >= c2 else 2

            res = 200
            xg = np.linspace(0, 1.5, res)
            yg = np.linspace(0, 1.5, res)
            grid = np.zeros((res, res), dtype=np.uint8)
            for ix, x in enumerate(xg):
                for iy, y in enumerate(yg):
                    grid[iy, ix] = knn_classify(x, y)

            color_map = np.zeros((res, res, 4), dtype=np.uint8)
            color_map[grid == 1] = [220, 38, 38, 60]   # Red, alpha=60
            color_map[grid == 2] = [37, 99, 235, 60]   # Blue, alpha=60

            from pyqtgraph import ImageItem
            img = ImageItem(color_map, axisOrder='row-major')
            img.setZValue(-10)
            img.setRect(0, 0, 1.5, 1.5)
            self._knn_bg_item = img
            self.plot_widgets[3].addItem(img)
        except Exception as e:
            print(f"[KNN BG] Error: {e}")

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_all_plots)
        self.plot_timer.start(UPDATE_INTERVAL_MS)

        self._reset_runtime_buffers()
        self._update_toggle_buttons()

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
        default_name = "251223_cyk_assist_SOS_trial1"
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
        
        # [수정] 요청된 그래프 제목 리스트
        titles = [
            "Thigh angle",                # Graph 1
            "Assistance torque profile",  # Graph 2
            "T_swing at HC",              # Graph 3
            "KNN classified gait mode"    # Graph 4
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
            
            # [수정] 제목 설정 적용
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
        # [수정 완료] Plot 4 설정: 0~1.5 범위 고정, 1:1 비율 유지
        # -----------------------------------------------------------------
        self.plot_widgets[3].setLabel("bottom", "s_norm_vel_HC")
        self.plot_widgets[3].setLabel("left", "s_norm_T_HC")
        self.plot_widgets[3].setXRange(0.0, 1.5, padding=0.0)
        self.plot_widgets[3].setYRange(0.0, 1.5, padding=0.0)
        self.plot_widgets[3].setAspectLocked(True, ratio=1.0)
        
        self.plot_widgets[3].setMouseEnabled(x=False, y=False)
        self.plot_widgets[3].hideButtons() 
        self.plot_widgets[3].plotItem.vb.setLimits(xMin=0, xMax=1.5, yMin=0, yMax=1.5)

        self._hc_scatter_item = pg.ScatterPlotItem()
        self.plot_widgets[3].addItem(self._hc_scatter_item)
        self._scatter_legend = self.plot_widgets[3].addLegend()
        self._scatter_legend_items = []
        for label, color in (("STS", "#dc2626"), ("SOS", "#2563eb")):
            dummy_item = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(color), pen=pg.mkPen(color))
            self._scatter_legend.addItem(dummy_item, label)
            self._scatter_legend_items.append(dummy_item)

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
        try:
            hc_idx = CSV_COLS.index("hc_count")
            hc_val = int(row[hc_idx])
            if self._last_hc_count is None: self._last_hc_count = hc_val
            elif hc_val > self._last_hc_count:
                self._last_hc_count = hc_val
                x = float(row[CSV_COLS.index("s_norm_vel_HC")])
                y = float(row[CSV_COLS.index("s_norm_T_HC")])
                gm = int(row[CSV_COLS.index("s_gait_mode")])
                self._hc_points_x.append(x)
                self._hc_points_y.append(y)
                self._hc_points_brush.append(self._gait_mode_to_brush(gm))
                if SCATTER_KEEP_LAST_N > 0:
                    self._hc_points_x = self._hc_points_x[-SCATTER_KEEP_LAST_N:]
                    self._hc_points_y = self._hc_points_y[-SCATTER_KEEP_LAST_N:]
                    self._hc_points_brush = self._hc_points_brush[-SCATTER_KEEP_LAST_N:]
        except: pass
        try:
            v = row[self.default_idx]
            self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}, LeftHipAngle={v:.4f}")
        except: self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}")

    def update_all_plots(self):
        if not self.data_all: return
        rows = self.data_all[-WINDOW_SIZE_SAMPLES:]
        if not rows: return
        x_vals = [r[0] for r in rows]

        for plot_idx in range(3):
            pw = self.plot_widgets[plot_idx]
            group = self.plot_groups[plot_idx]
            items = self._ts_items[plot_idx] if plot_idx < len(self._ts_items) else {}
            for name in group:
                try: col_idx = CSV_COLS.index(name)
                except ValueError: continue
                y_vals = [r[col_idx] for r in rows]
                item = items.get(name)
                if item is not None: item.setData(x_vals, y_vals)
            if x_vals:
                if pw is self.plot_widgets[3]:
                    pw.setXRange(0.0, 1.5, padding=0.0)
                    pw.setYRange(0.0, 1.5, padding=0.0)
                    pw.setAspectLocked(True, ratio=1.0)
                else:
                    pw.setXRange(min(x_vals), max(x_vals), padding=0.01)

        if self._hc_scatter_item is not None:
            spots = []
            total_pts = len(self._hc_points_x)
            for idx, (x, y, br) in enumerate(zip(self._hc_points_x, self._hc_points_y, self._hc_points_brush)):
                color = br.color() if isinstance(br, QtGui.QBrush) else QtGui.QColor("#6b7280")
                is_latest = (idx == total_pts - 1)
                spot = {
                    "pos": (x, y),
                    "brush": br,
                    "pen": pg.mkPen(color),
                    "size": 12 if is_latest else 8,
                    "symbol": "star" if is_latest else "o",
                    "text": str(idx + 1),
                    "textColor": color,
                    "font": QtGui.QFont("Segoe UI", 10, QtGui.QFont.Bold),
                }
                spots.append(spot)
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