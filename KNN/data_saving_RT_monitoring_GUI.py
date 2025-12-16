import sys
import os
import glob
import csv
import struct
import time
import serial
import serial.tools.list_ports
from datetime import datetime

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt

import pyqtgraph as pg
import numpy as np

# ===================== 0) 백엔드 고정 설정 =====================
UPDATE_INTERVAL_MS = 50         # 실시간 그래프 갱신 주기 (ms) (20Hz)
WINDOW_SIZE_SAMPLES = 1000      # 실시간 플롯 최근 샘플 수
SCATTER_KEEP_LAST_N = 20        # 4번 Scatter plot에서 유지할 최근 점 개수

# 2ms, 패킷 약 300 bytes라고 가정하면 1초에 약 500패킷 → 500라인 모아서 한 번에 write
FLUSH_EVERY = 500               # CSV 라인 500개 모을 때마다 디스크에 기록

# ===================== 1) 패킷/CRC 설정 (펌웨어와 동일 포맷) =====================

# C 구조체 (펌웨어: XM_Apps/User_Algorithm/Basics.h 의 SavingData_t)
# typedef struct __attribute__((packed)) {
#     uint16_t sof;      // 0xAA55
#     uint16_t len;      // 전체 패킷 길이 (sizeof(SavingData_t))
#
#     uint32_t loopCnt;
#     uint8_t  h10Mode;
#     uint8_t  h10AssistLevel;
#     uint8_t  SmartAssist;
#
#     float leftHipAngle,  rightHipAngle;
#     float leftThighAngle, rightThighAngle;
#     float leftHipTorque,  rightHipTorque;
#     float leftHipMotorAngle, rightHipMotorAngle;
#     float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
#     float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
#     float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
#     float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;
#
#     // (removed) TrunkIMU_* fields
#
#     int32_t is_moving;
#     int32_t hc_count;
#     int32_t R_count_upeak;
#     int32_t L_count_upeak;
#     int32_t R_count_dpeak;
#     int32_t L_count_dpeak;
#
#     uint8_t tau_max_setting;
#     uint8_t s_gait_mode;
#     float   s_g_knn_conf;
#
#     float T_swing_ms;
#     float s_norm_vel_HC;
#     float s_norm_T_HC;
#     float s_scaling_X;
#     float s_scaling_Y;
#
#     uint16_t crc;
# } SavingData_t;

SOF_VALUE = 0xAA55

# payload 부분 (sof/len/crc 제외):
#   uint32 loopCnt
#   uint8  h10Mode, h10AssistLevel, SmartAssist
#   float  21개
#   int32  6개 (is_moving, hc_count, R/L upeak, R/L dpeak)
#   uint8  2개 (tau_max_setting, s_gait_mode)
#   float  1개 (s_g_knn_conf)
#   float  9개 (s_g_knn_conf + T_swing_ms/SOS/STS + s_vel_HC + s_T_HC_s + norm/scaling)
STRUCT_FMT = '<IBBB21f6iBB10f'
PAYLOAD_SIZE = struct.calcsize(STRUCT_FMT)

# 전체 패킷 예상 길이: sof(2) + len(2) + payload + crc(2)
EXPECTED_TOTAL_PACKET_SIZE = 2 + 2 + PAYLOAD_SIZE + 2

# Some firmware builds may send an older/newer SavingData_t size.
# We accept packets whose internal len matches any of these allowed sizes.
# Keep EXPECTED_TOTAL_PACKET_SIZE as the primary target.
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
    "s_vel_HC,s_T_HC_s,"
    "s_norm_vel_HC,s_norm_T_HC,s_scaling_X,s_scaling_Y"
)
CSV_COLS = CSV_HEADER.split(",")


def make_missing_row(loop_cnt: int, last_row=None):
    """Create a placeholder row for a missed/invalid packet.

    - Float-like fields become NaN so they can be interpolated later.
    - Integer-like fields are set to the previous value (forward-fill) when available,
      otherwise 0.
    """
    row = [float("nan")] * len(CSV_COLS)
    row[0] = int(loop_cnt)

    int_indices = {0, 1, 2, 3, 25, 26, 27, 28, 29, 30, 31, 32}
    if last_row is None:
        for idx in int_indices:
            if idx == 0:
                continue
            row[idx] = 0
        return row

    for idx in int_indices:
        if idx == 0:
            continue
        try:
            row[idx] = int(last_row[idx])
        except Exception:
            row[idx] = 0
    return row

DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 1.0  # sec


def crc16_modbus(data: bytes, init_val: int = 0xFFFF) -> int:
    """
    MCU의 CalcCrc16과 동일한 CRC16 (poly 0xA001, init 0xFFFF, LSB-first).
    """
    crc = init_val
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def decode_packet(data_tuple):
    """STRUCT_FMT 언팩 결과(tuple) -> row(list), CSV_COLS와 동일한 순서"""
    # Expected tuple layout:
    #   [0] loopCnt (I)
    #   [1..3] h10Mode, h10AssistLevel, SmartAssist (B,B,B)
    #   [4..24] 21 floats
    #   [25..30] 6 int32
    #   [31..32] 2 uint8
    #   [33..40] 8 floats (s_g_knn_conf 포함 + debug 7)
    expected_tuple_len = struct.calcsize(STRUCT_FMT)  # bytes, used only for debugging sanity elsewhere
    expected_elems = 4 + 21 + 6 + 2 + 10
    if len(data_tuple) != expected_elems:
        raise ValueError(f"Unexpected data length: {len(data_tuple)}")

    # Row must match CSV columns exactly.
    row = [0] * len(CSV_COLS)

    # 0) header-ish fields
    row[0] = int(data_tuple[0])
    row[1] = int(data_tuple[1])
    row[2] = int(data_tuple[2])
    row[3] = int(data_tuple[3])

    # 1) 21 floats
    for i in range(21):
        row[4 + i] = float(data_tuple[4 + i])

    # 2) 6 int32
    base = 4 + 21
    for i in range(6):
        row[25 + i] = int(data_tuple[base + i])

    # 3) 2 uint8
    base = 4 + 21 + 6
    row[31] = int(data_tuple[base + 0])
    row[32] = int(data_tuple[base + 1])

    # 4) 10 floats (s_g_knn_conf + swing + raw HC + norm/scaling)
    base = 4 + 21 + 6 + 2
    for i in range(10):
        row[33 + i] = float(data_tuple[base + i])

    return row


def row_to_csv_line(row):
    """row(list) → CSV string (정수/float 혼합)."""
    # int fields: loopCnt, h10Mode/AssistLevel/SmartAssist, is_moving/hc_count/upeak/dpeak, tau_max_setting, s_gait_mode
    int_indices = {0, 1, 2, 3, 25, 26, 27, 28, 29, 30, 31, 32}
    parts = []
    for idx, val in enumerate(row):
        if idx in int_indices:
            parts.append(str(int(val)))
        else:
            parts.append(f"{float(val):.4f}")
    return ",".join(parts)


# ===================== 전역 모던 스타일 적용 =====================

def apply_modern_style(app: QtWidgets.QApplication):
    """
    전체 앱에 밝은 테마 + 둥근 폰트 + 라운드 위젯 스타일 적용
    """
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
        QWidget {
            background-color: #f5f7fb;
            font-family: 'Segoe UI', 'Noto Sans CJK KR', 'Malgun Gothic', sans-serif;
            font-size: 11pt;
            color: #111827;
        }

        QMainWindow {
            background-color: #f5f7fb;
        }

        QDialog {
            background-color: #f5f7fb;
        }

        QLabel {
            font-size: 10.5pt;
        }

        QLineEdit, QComboBox {
            background-color: #ffffff;
            border: 1px solid #cbd5e1;
            border-radius: 8px;
            padding: 4px 8px;
            selection-background-color: #2563eb;
            selection-color: #ffffff;
        }

        QComboBox::drop-down {
            border: 0px;
        }

        QComboBox::down-arrow {
            image: none;
            width: 0px;
            height: 0px;
        }

        QPushButton {
            background-color: #2563eb;
            color: #ffffff;
            border-radius: 8px;
            padding: 6px 14px;
            border: none;
            font-weight: 500;
        }

        QPushButton:hover {
            background-color: #1d4ed8;
        }

        QPushButton:disabled {
            background-color: #9ca3af;
            color: #e5e7eb;
        }

        QStatusBar {
            background-color: #ffffff;
            border-top: 1px solid #e5e7eb;
        }

        QStatusBar::item {
            border: 0px;
        }

        QScrollArea, QAbstractScrollArea {
            background-color: transparent;
        }

        QCheckBox {
            spacing: 6px;
        }

        QCheckBox::indicator {
            width: 16px;
            height: 16px;
            border-radius: 4px;
            border: 1px solid #cbd5e1;
            background-color: #ffffff;
        }

        QCheckBox::indicator:checked {
            background-color: #2563eb;
            border: 1px solid #2563eb;
        }

        QToolTip {
            background-color: #111827;
            color: #f9fafb;
            border-radius: 6px;
            padding: 4px 8px;
        }

        QGroupBox {
            border: 1px solid #e5e7eb;
            border-radius: 10px;
            margin-top: 8px;
            background-color: #ffffff;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 4px 10px;
        }
    """)


# ===================== 2) 시리얼 워커 =====================

class SerialWorker(QtCore.QObject):
    data_received = pyqtSignal(list)
    status_msg = pyqtSignal(str)
    finished = pyqtSignal()
    connection_failed = pyqtSignal(str)
    packet_error_count = pyqtSignal(int)   # 패킷 에러 누적 개수 신호

    def __init__(self, port_name, baudrate=DEFAULT_BAUD, timeout=DEFAULT_TIMEOUT, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baudrate = baudrate
        self.timeout = timeout
        self._running = False
        self._error_count = 0
        self._last_err_log_ts = 0.0
        self._err_log_interval_s = 1.0
        self._last_good_row = None

    def _log_packet_debug(self, sof: int, length: int, packet_len: int = None, recv_crc: int = None, calc_crc: int = None):
        """에러 상황에서만 len/CRC 등을 스로틀링해서 출력."""
        now = time.time()
        if (now - self._last_err_log_ts) < self._err_log_interval_s:
            return
        self._last_err_log_ts = now

        parts = [
            f"rx_sof=0x{sof:04X}",
            f"rx_len={length}",
            f"expected_len={EXPECTED_TOTAL_PACKET_SIZE}",
        ]
        if packet_len is not None:
            parts.append(f"rx_bytes={packet_len}")
        if recv_crc is not None and calc_crc is not None:
            parts.append(f"crc_rx=0x{recv_crc:04X}")
            parts.append(f"crc_calc=0x{calc_crc:04X}")
        self.status_msg.emit("Packet debug: " + ", ".join(parts))

    def _inc_error(self, reason: str = ""):
        """패킷 검증 실패 시 호출 (누적 개수 증가 + 신호 송신)."""
        self._error_count += 1
        self.packet_error_count.emit(self._error_count)
        if reason:
            self.status_msg.emit(f"Packet error #{self._error_count}: {reason}")

    def _resync_drop_one_byte(self):
        """Framing resync helper (does NOT count as a packet error)."""
        # Intentionally no _inc_error here: byte-level desync may happen many times per single bad packet.
        return

    @pyqtSlot()
    def run(self):
        self._running = True
        ser = None
        try:
            try:
                ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
                self.status_msg.emit(f"Connected to {self.port_name}")
                self.status_msg.emit(f"Expected total packet size: {EXPECTED_TOTAL_PACKET_SIZE} bytes")
            except Exception as e:
                err_msg = f"Serial open error: {e}"
                self.status_msg.emit(err_msg)
                self.connection_failed.emit(err_msg)
                return

            # 모니터링 시작 명령
            # NOTE: Firmware may stream continuously without any start command.
            # If you still need start/stop commands for other firmware, you can re-enable it.

            buf = bytearray()

            while self._running:
                try:
                    # 한 번에 조금 더 크게 읽어 효율/지터 개선 (256 → 512)
                    chunk = ser.read(512)
                except serial.SerialException as e:
                    self.status_msg.emit(f"Serial read error: {e}")
                    break

                if not chunk:
                    continue

                buf.extend(chunk)

                # 가능한 한 많이 패킷 파싱
                while True:
                    # 최소 헤더(sof + len) 필요
                    if len(buf) < 4:
                        break

                    # SOF 체크 (little-endian uint16)
                    sof = buf[0] | (buf[1] << 8)
                    if sof != SOF_VALUE:
                        # 프레이밍 에러: 1바이트씩 밀면서 재동기화
                        length_hint = (buf[2] | (buf[3] << 8)) if len(buf) >= 4 else 0
                        self._log_packet_debug(sof=sof, length=length_hint)
                        self._resync_drop_one_byte()
                        del buf[0]
                        continue

                    # len 읽기
                    length = buf[2] | (buf[3] << 8)

                    # 길이 sanity check
                    if length not in ALLOWED_TOTAL_PACKET_SIZES:
                        # If 'length' looks unreasonable, resync quickly.
                        # Valid packets should be within a compact range.
                        if length < 8 or length > 2048:
                            # resync only (do not count as packet error)
                            pass
                        else:
                            # resync only (do not count as packet error)
                            pass
                        self._log_packet_debug(sof=sof, length=length)
                        del buf[0]
                        continue

                    # 전체 패킷이 다 도착했는지 확인
                    if len(buf) < length:
                        # 아직 덜 들어옴 → 다음 read까지 대기
                        break

                    # ---- 여기서부터는 "한 패킷" 단위 처리 ----
                    packet = bytes(buf[:length])
                    del buf[:length]

                    had_error = False
                    reason = ""
                    row = None

                    # 1) 패킷 길이 검증
                    if len(packet) != length:
                        had_error = True
                        reason = f"Packet size mismatch (got {len(packet)}, expected {length})"
                        self._log_packet_debug(sof=sof, length=length, packet_len=len(packet))
                    else:
                        # 2) CRC 검증: sof~(crc 직전까지)
                        data_without_crc = packet[:-2]
                        recv_crc = packet[-2] | (packet[-1] << 8)
                        calc_crc = crc16_modbus(data_without_crc)

                        if recv_crc != calc_crc:
                            had_error = True
                            reason = "CRC mismatch"
                            self._log_packet_debug(sof=sof, length=length, packet_len=len(packet), recv_crc=recv_crc, calc_crc=calc_crc)
                        else:
                            # 3) payload 추출: [sof(2), len(2)] 이후 ~ crc 직전
                            payload = packet[4:-2]
                            if len(payload) != PAYLOAD_SIZE:
                                had_error = True
                                reason = f"Payload size mismatch (got {len(payload)}, expected {PAYLOAD_SIZE})"
                                self._log_packet_debug(sof=sof, length=length, packet_len=len(packet))
                            else:
                                # 4) struct.unpack + decode
                                try:
                                    data_tuple = struct.unpack(STRUCT_FMT, payload)
                                    row = decode_packet(data_tuple)
                                except struct.error as e:
                                    had_error = True
                                    reason = f"Struct unpack error: {e}"
                                    self._log_packet_debug(sof=sof, length=length, packet_len=len(packet))
                                except ValueError as e:
                                    had_error = True
                                    reason = f"Decode error: {e}"
                                    self._log_packet_debug(sof=sof, length=length, packet_len=len(packet))

                    # 5) 이 패킷에 에러가 하나라도 있었으면 1회만 카운트 (tick/packet-based)
                    if had_error:
                        self._inc_error(reason)
                        # Mark this tick as missing: forward-fill ints, NaN for floats.
                        try:
                            next_loop = int(self._last_good_row[0]) + 1 if self._last_good_row is not None else 0
                        except Exception:
                            next_loop = 0
                        self.data_received.emit(make_missing_row(next_loop, self._last_good_row))
                        continue

                    # 6) 정상 패킷이면 row emit
                    if row is not None:
                        self._last_good_row = row
                        self.data_received.emit(row)

        finally:
            if ser is not None and ser.is_open:
                try:
                    # Optional stop command (only needed for firmwares that require it)
                    pass
                except Exception:
                    pass
                ser.close()
                self.status_msg.emit("Serial port closed")
            self.finished.emit()

    def stop(self):
        self._running = False


# ===================== 3) CSV 리뷰 뷰어 (8개 그래프 + 선택 레이아웃) =====================

class CsvReviewDialog(QtWidgets.QDialog):
    """
    - 가장 최근 CSV 파일을 불러와 8개 그래프(1~8번) 표시
    - 좌측 체크박스 8개로 표시/숨김 선택
    - 선택 개수에 따라 배치 변경:
        * n <= 4 → 1열(세로)
        * 5 <= n <= 6 → 2×3
        * 7 <= n <= 8 → 2×4
    - 모든 플롯 크기 균일
    """
    def __init__(self, csv_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"CSV Review — {os.path.basename(csv_path)}")
        self.resize(1500, 900)
        self.csv_path = csv_path

        # 8개 플롯의 변수 그룹
        self.groups = [
            ["LeftHipAngle", "RightHipAngle", "LeftHipTorque", "RightHipTorque"],  # 1
            ["LeftHipImuGlobalAccX","LeftHipImuGlobalAccY","LeftHipImuGlobalAccZ",
             "RightHipImuGlobalAccX","RightHipImuGlobalAccY","RightHipImuGlobalAccZ"],  # 2
            ["LeftHipImuGlobalGyrX","LeftHipImuGlobalGyrY","LeftHipImuGlobalGyrZ",
             "RightHipImuGlobalGyrX","RightHipImuGlobalGyrY","RightHipImuGlobalGyrZ"],  # 3
            ["TrunkIMU_LocalAccX","TrunkIMU_LocalAccY","TrunkIMU_LocalAccZ"],  # 4
            ["TrunkIMU_LocalGyrX","TrunkIMU_LocalGyrY","TrunkIMU_LocalGyrZ"],  # 5
            ["TrunkIMU_QuatW","TrunkIMU_QuatX","TrunkIMU_QuatY","TrunkIMU_QuatZ"],  # 6
            ["is_moving", "hc_count", "R_count_upeak", "L_count_upeak", "R_count_dpeak", "L_count_dpeak"],  # 7
              ["tau_max_setting", "s_gait_mode", "s_g_knn_conf",
               "T_swing_ms", "T_swing_SOS_ms", "T_swing_STS_ms",
               "s_vel_HC", "s_T_HC_s", "s_norm_vel_HC", "s_norm_T_HC"],  # 8
        ]

        # 데이터 로드
        self.col_index = {name: i for i, name in enumerate(CSV_COLS)}
        self.loop_cnt = None
        self.data = None
        self._load_csv(csv_path)

        # UI 관련 멤버
        self.checkboxes = []
        self.plots = []
        self.grid = None

        self._build_ui()
        self._plot_all()
        self._relayout()  # 초기 체크 상태(전부 체크)에 맞춰 균일 배치

    def _load_csv(self, path):
        # 헤더 스킵 후 빠르게 로드
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
        arr = np.genfromtxt(path, delimiter=",", skip_header=1)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        self.data = self._postprocess_missing(arr)
        self.loop_cnt = self.data[:, 0]

    def _postprocess_missing(self, arr: np.ndarray) -> np.ndarray:
        """Fill/repair rows that were logged as missing (NaNs).

        Policy:
        - Integer-like columns: forward-fill (copy previous valid value).
        - Float columns: linear interpolation on loopCnt index; edge NaNs are filled by nearest valid.
        """
        if arr.size == 0:
            return arr

        data = np.array(arr, copy=True)
        x = data[:, 0]

        int_indices = np.array([0, 1, 2, 3, 25, 26, 27, 28, 29, 30, 31, 32], dtype=int)
        # 1) forward-fill int-like indices
        for col in int_indices:
            y = data[:, col]
            if np.all(np.isnan(y)):
                data[:, col] = 0
                continue
            last = None
            for i in range(len(y)):
                if not np.isnan(y[i]):
                    last = y[i]
                else:
                    if last is not None:
                        y[i] = last
            data[:, col] = y

        # 2) interpolate float-like columns (everything except int-like)
        cols = data.shape[1]
        for col in range(cols):
            if col in set(int_indices.tolist()):
                continue
            y = data[:, col]
            if not np.any(np.isnan(y)):
                continue
            valid = ~np.isnan(y)
            if valid.sum() == 0:
                # no valid values at all
                data[:, col] = np.nan
                continue
            if valid.sum() == 1:
                data[:, col] = y[valid][0]
                continue
            # linear interpolation; fill edges with nearest valid
            y_interp = np.interp(x, x[valid], y[valid])
            data[:, col] = y_interp

        return data

    def _build_ui(self):
        # QDialog에서는 setCentralWidget 사용 X → 바로 레이아웃을 this에 붙임
        main_layout = QtWidgets.QVBoxLayout(self)

        # ---- 상단: 체크박스/버튼 영역 ----
        top_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(top_layout)

        # 좌측: 8개 체크박스
        cb_group_box = QtWidgets.QGroupBox("Graphs")
        cb_layout = QtWidgets.QVBoxLayout(cb_group_box)

        self.checkboxes = []
        for i in range(8):
            label = f"Graph {i+1}"
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(True)
            cb.stateChanged.connect(self._on_visibility_changed)
            self.checkboxes.append(cb)
            cb_layout.addWidget(cb)

        top_layout.addWidget(cb_group_box)

        # 우측: 전체 on/off 버튼
        btn_layout = QtWidgets.QVBoxLayout()
        btn_select_all = QtWidgets.QPushButton("Select All")
        btn_clear_all = QtWidgets.QPushButton("Clear All")
        btn_select_all.clicked.connect(lambda: self._set_all(True))
        btn_clear_all.clicked.connect(lambda: self._set_all(False))
        btn_layout.addWidget(btn_select_all)
        btn_layout.addWidget(btn_clear_all)
        btn_layout.addStretch()
        top_layout.addLayout(btn_layout)

        # ---- 하단: 플롯 영역 (grid) ----
        self.grid = QtWidgets.QGridLayout()
        main_layout.addLayout(self.grid, stretch=1)

        self.plots = []
        for i, group in enumerate(self.groups):
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

            title_txt = f"Graph {i+1}"
            preview = ", ".join(group[:4])
            if len(group) > 4:
                preview += ", ..."
            title_txt += f"  [{preview}]"
            pw.setTitle(title_txt)

            self.plots.append(pw)

        # 실제 배치는 _relayout()에서 수행

    def _plot_all(self):
        # 정적 전체 길이 플로팅
        if self.data is None or self.loop_cnt is None:
            return

        for i, pw in enumerate(self.plots):
            pw.clear()
            legend = pw.addLegend()
            names = self.groups[i]
            hues = max(8, len(names))
            for k, name in enumerate(names):
                if name not in self.col_index:
                    continue
                col = self.col_index[name]
                if col >= self.data.shape[1]:
                    continue
                y = self.data[:, col]
                color = pg.intColor(k, hues=hues)
                pw.plot(self.loop_cnt, y, pen=color, name=name)
            if self.loop_cnt.size:
                pw.setXRange(float(self.loop_cnt.min()),
                             float(self.loop_cnt.max()),
                             padding=0.01)

    def _on_visibility_changed(self, state):
        # 보이기/숨김 적용 + 레이아웃 갱신
        for i, cb in enumerate(self.checkboxes):
            self.plots[i].setVisible(cb.isChecked())
        self._relayout()

    def _set_all(self, value: bool):
        # 체크박스 전체 on/off
        for cb in self.checkboxes:
            # setChecked() → stateChanged 시그널이 나가면서 _relayout 호출됨
            cb.setChecked(value)

    def _relayout(self):
        """
        선택 개수에 따라 균일 배치:
        - n <= 4  → 1열(세로)
        - 5 <= n <= 6 → 2×3
        - 7 <= n <= 8 → 2×4
        """
        if self.grid is None:
            return

        # 1) 기존 항목 제거
        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)

        # 2) 보이는 플롯 인덱스
        visible_idxs = [i for i, cb in enumerate(self.checkboxes) if cb.isChecked()]
        n = len(visible_idxs)

        if n == 0:
            # 아무것도 선택 안한 경우 → stretch도 모두 0
            for c in range(4):
                self.grid.setColumnStretch(c, 0)
            for r in range(4):
                self.grid.setRowStretch(r, 0)
            return

        # 3) 열/행 결정
        if n <= 4:
            cols = 1
            rows = n
            max_cols, max_rows = 1, 4
        elif n <= 6:
            cols = 2
            rows = 3
            max_cols, max_rows = 2, 4
        else:  # 7~8
            cols = 2
            rows = 4
            max_cols, max_rows = 2, 4

        # 4) 재배치
        for order, idx in enumerate(visible_idxs[:8]):  # 최대 8개
            r = order // cols
            c = order % cols
            self.grid.addWidget(self.plots[idx], r, c)
            self.plots[idx].setVisible(True)

        # 5) 균일 크기 유지: stretch 재설정
        for c in range(max_cols):
            self.grid.setColumnStretch(c, 0)
        for r in range(max_rows):
            self.grid.setRowStretch(r, 0)
        for c in range(cols):
            self.grid.setColumnStretch(c, 1)
        for r in range(rows):
            self.grid.setRowStretch(r, 1)

        # 6) 보이지 않는 플롯은 숨김 유지
        for i in range(8):
            if i not in visible_idxs:
                self.plots[i].setVisible(False)

# ===================== 4) 메인 GUI (실시간: 6개 플롯) =====================

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("USB CDC Real-time Monitor & Auto Logger (6 Plots)")
        self.resize(1400, 900)

        # 버퍼 & 상태
        self.data_all = []
        self._recv_count = 0

        # HC scatter 상태
        self._last_hc_count = None
        self._hc_points_x = []
        self._hc_points_y = []
        self._hc_points_brush = []
        self._hc_scatter_item = None
        # NOTE: Scatter plot (Graph 4) is intentionally cumulative.
        # We do not trim _hc_points_* so that previously plotted points remain visible.

        # 실시간 plot item 캐시 (setData로 갱신해서 CPU 절약)
        self._ts_items = []  # list[dict[name -> PlotDataItem]] for plot 1~3

        # 기본 표시 변수(LeftHipAngle)
        self.default_idx = CSV_COLS.index("LeftHipAngle")

        # 스레드
        self.serial_thread = None
        self.serial_worker = None

        # 파일 로깅 핸들
        self.log_file = None
        self._written_lines = 0
        self._last_log_path = None  # 최근 저장 파일
        self._pending_lines = []    # FLUSH_EVERY개 모았다가 writelines

        # 플롯 그룹(4개: 1~4). 4번은 (HC 시점 scatter)라서 별도 처리.
        self.plot_groups = []
        self._build_plot_groups()

        # 우측 상태 패널 위젯들
        self.state_labels = {}
        self.is_moving_indicator = None

        self._build_ui()
        self.refresh_ports()

        # 타이머
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_all_plots)
        self.plot_timer.start(UPDATE_INTERVAL_MS)

    def _set_is_moving_indicator(self, is_moving: int):
        if self.is_moving_indicator is None:
            return
        if int(is_moving) != 0:
            # moving: blue
            self.is_moving_indicator.setStyleSheet("color: #2563eb;")
        else:
            # not moving: red
            self.is_moving_indicator.setStyleSheet("color: #dc2626;")

    def _gait_mode_to_brush(self, gait_mode: int) -> QtGui.QBrush:
        """s_gait_mode 값에 따라 점 색 결정.
        - STS(L_STS/R_STS): red
        - SOS(L_SOS/R_SOS): blue
        나머지: gray
        """
        # 펌웨어 enum 값이 정확히 무엇인지는 코드에서 알 수 없으니,
        # 우선 일반적으로 0:NONE, 1:RSOS,2:RSTS,3:LSOS,4:LSTS 순서를 가정.
        # 다른 매핑이면 아래 값만 수정하면 됨.
        gm = int(gait_mode)
        RSOS, RSTS, LSOS, LSTS = 1, 2, 3, 4
        if gm in (RSTS, LSTS):
            return QtGui.QBrush(QtGui.QColor("#dc2626"))
        if gm in (RSOS, LSOS):
            return QtGui.QBrush(QtGui.QColor("#2563eb"))
        return QtGui.QBrush(QtGui.QColor("#6b7280"))

    def _build_plot_groups(self):
        # 1) 좌/우 thigh angle
        g1 = ["LeftThighAngle", "RightThighAngle"]
        # 2) 좌/우 hip torque
        g2 = ["LeftHipTorque", "RightHipTorque"]
        # 3) T_swing: mean + SOS/STS buckets
        g3 = ["T_swing_ms", "T_swing_SOS_ms", "T_swing_STS_ms"]
        # 4) scatter: (s_norm_vel_HC, s_norm_T_HC) at hc_count increments
        g4 = []
        self.plot_groups = [g1, g2, g3, g4]

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

          # 상단 컨트롤 라인
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

        self.btn_disconnect = QtWidgets.QPushButton("Disconnect & Stop")
        self.btn_disconnect.clicked.connect(self.on_disconnect_clicked)
        self.btn_disconnect.setEnabled(False)
        top.addWidget(self.btn_disconnect)

        top.addStretch()

        # ==== 여기부터 Output folder / Output file UI 추가 ====
        # Output folder
        default_folder = os.path.abspath(os.path.join(os.getcwd(), "data"))
        top.addWidget(QtWidgets.QLabel("Output folder:"))
        self.edit_folder = QtWidgets.QLineEdit(default_folder)
        self.edit_folder.setMinimumWidth(260)
        top.addWidget(self.edit_folder)

        self.btn_browse_folder = QtWidgets.QPushButton("Folder…")
        self.btn_browse_folder.clicked.connect(self.on_browse_folder_clicked)
        top.addWidget(self.btn_browse_folder)


        top.addWidget(QtWidgets.QLabel("Output file:"))
        default_name = f"cdc_monitoring_data_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.edit_filename = QtWidgets.QLineEdit(default_name)
        self.edit_filename.setMinimumWidth(320)
        top.addWidget(self.edit_filename)

        self.btn_browse = QtWidgets.QPushButton("Browse…")
        self.btn_browse.clicked.connect(self.on_browse_clicked)
        top.addWidget(self.btn_browse)

        # 메인 영역: 좌측(2x2 plots) + 우측(state panel)
        content = QtWidgets.QHBoxLayout()
        vbox.addLayout(content, stretch=1)

        # ---- 좌측: 2x2 플롯 ----
        grid = QtWidgets.QGridLayout()
        content.addLayout(grid, stretch=1)

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

            title_txt = f"Graph {i+1}"
            preview = ", ".join(self.plot_groups[i][:4])
            if len(self.plot_groups[i]) > 4:
                preview += ", ..."
            if preview:
                title_txt += f"  [{preview}]"
            pw.setTitle(title_txt)
            self.plot_widgets.append(pw)

            r = i // 2
            c = i % 2
            grid.addWidget(pw, r, c)

        # Plot 1~3: legend는 1회만 만들고, PlotDataItem 재사용
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

        # 4번 플롯은 scatter 전용 설정
        self.plot_widgets[3].setLabel("bottom", "s_norm_vel_HC")
        self.plot_widgets[3].setLabel("left", "s_norm_T_HC")
        self.plot_widgets[3].setXRange(0.0, 1.5, padding=0.0)
        self.plot_widgets[3].setYRange(0.0, 1.5, padding=0.0)

        # Plot 4: scatter item을 1회만 만들고 setData로 갱신
        self._hc_scatter_item = pg.ScatterPlotItem()
        self.plot_widgets[3].addItem(self._hc_scatter_item)

        # ---- 우측: 상태 패널 ----
        state_box = QtWidgets.QGroupBox("States")
        state_layout = QtWidgets.QVBoxLayout(state_box)
        content.addWidget(state_box, stretch=0)

        # 통신/모드 상태 (기존 라벨 유지)
        self.mode_label = QtWidgets.QLabel("H10Mode: -,  H10AssistLevel: -,  SmartAssist: -")
        font = self.mode_label.font()
        font.setPointSize(font.pointSize() + 2)
        font.setBold(True)
        self.mode_label.setFont(font)
        state_layout.addWidget(self.mode_label)

        self.error_label = QtWidgets.QLabel("Packet errors: 0")
        self.error_label.setFont(font)
        state_layout.addWidget(self.error_label)

        # is_moving 신호등
        moving_row = QtWidgets.QHBoxLayout()
        moving_row.addWidget(QtWidgets.QLabel("is_moving:"))
        self.is_moving_indicator = QtWidgets.QLabel("●")
        ind_font = self.is_moving_indicator.font()
        ind_font.setPointSize(ind_font.pointSize() + 8)
        self.is_moving_indicator.setFont(ind_font)
        self.is_moving_indicator.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        moving_row.addWidget(self.is_moving_indicator)
        moving_row.addStretch()
        state_layout.addLayout(moving_row)

        # 요청된 state들(세로 나열): h10Mode, tau_max_setting, s_scaling_X, s_scaling_Y
        for key in ("h10Mode", "tau_max_setting", "s_scaling_X", "s_scaling_Y"):
            lbl = QtWidgets.QLabel(f"{key}: -")
            lbl.setFont(font)
            state_layout.addWidget(lbl)
            self.state_labels[key] = lbl

        state_layout.addStretch()

        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")

    # ---------- 포트/파일 ----------
    def refresh_ports(self):
        self.combo_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.device)
        if not ports:
            self.combo_port.addItem("(no ports)")
        self.status_bar.showMessage("Port list refreshed")

    def _open_log_file(self):
        filename = (self.edit_filename.text() or "").strip()
        if not filename:
            filename = f"cdc_monitoring_data_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.edit_filename.setText(filename)

        folder = (self.edit_folder.text() or "").strip()
        if folder:
            full_path = os.path.join(folder, filename)
        else:
            full_path = filename

        # 절대 경로로 정규화
        full_path = os.path.abspath(full_path)

        # 상위 폴더가 없으면 생성
        dirpath = os.path.dirname(full_path)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath, exist_ok=True)

        # 파일 오픈
        self.log_file = open(full_path, "w", encoding="utf-8", newline="")
        self.log_file.write(CSV_HEADER + "\n")
        self._written_lines = 0
        self._pending_lines = []
        self._last_log_path = full_path

        self.status_bar.showMessage(
            f"Logging to: {full_path} (flush_every={FLUSH_EVERY})"
        )


    def _flush_pending_lines(self):
        """메모리에 모아둔 라인들을 실제 파일에 기록."""
        if self.log_file and self._pending_lines:
            try:
                self.log_file.writelines(self._pending_lines)
            except Exception as e:
                self.status_bar.showMessage(f"File write error (flush): {e}")
            self._pending_lines.clear()

    def _close_log_file(self):
        if self.log_file:
            try:
                # 남은 버퍼 먼저 기록
                self._flush_pending_lines()
                self.log_file.flush()
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None
            self.status_bar.showMessage("Log file closed")

    def _find_latest_csv(self):
        """
        최근 저장 파일 경로:
          1) 현재 세션 파일(_last_log_path)이 있으면 우선 사용
          2) 없으면 실행 폴더에서 패턴 검색
        """
        if self._last_log_path and os.path.isfile(self._last_log_path):
            return self._last_log_path
        pattern = os.path.join(os.getcwd(), "cdc_monitoring_data_log_*.csv")
        files = glob.glob(pattern)
        if not files:
            return None
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

    # ---------- 버튼 ----------
    def on_browse_folder_clicked(self):
        """
        Output folder 선택 대화상자
        """
        current = self.edit_folder.text().strip() or os.getcwd()
        folder = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Select output folder",
            current
        )
        if folder:
            self.edit_folder.setText(folder)

    def on_browse_clicked(self):
        base_folder = self.edit_folder.text().strip() or os.getcwd()
        initial_name = self.edit_filename.text().strip() or "cdc_monitoring_data_log.csv"

        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Select output CSV file",
            os.path.join(base_folder, initial_name),
            "CSV Files (*.csv);;All Files (*)"
        )
        if fname:
            folder = os.path.dirname(fname)
            name_only = os.path.basename(fname)
            self.edit_folder.setText(folder)
            self.edit_filename.setText(name_only)


    def on_connect_clicked(self):
        port_name = self.combo_port.currentText()
        if not port_name or port_name == "(no ports)":
            QtWidgets.QMessageBox.warning(self, "Warning", "No serial port selected.")
            return
        if self.serial_thread is not None:
            QtWidgets.QMessageBox.information(self, "Info", "Already connected.")
            return

        try:
            self._open_log_file()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open log file:\n{e}")
            return

        self.serial_thread = QtCore.QThread()
        self.serial_worker = SerialWorker(port_name, DEFAULT_BAUD, DEFAULT_TIMEOUT)
        self.serial_worker.moveToThread(self.serial_thread)

        self.serial_thread.started.connect(self.serial_worker.run)
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.status_msg.connect(self.on_status_msg)
        self.serial_worker.finished.connect(self.on_serial_finished)
        self.serial_worker.connection_failed.connect(self.on_connection_failed)
        self.serial_worker.packet_error_count.connect(self.on_packet_error_count)

        self.serial_thread.start()

        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.status_bar.showMessage(f"Connecting to {port_name} ...")
        # 에러 카운터 초기화
        self.error_label.setText("Packet errors: 0")

    def on_disconnect_clicked(self):
        if self.serial_worker is not None:
            self.serial_worker.stop()

    # ---------- 데이터/플롯 ----------
    @pyqtSlot(list)
    def on_data_received(self, row):
        self.data_all.append(row)
        self._recv_count += 1

        # CSV 파일에 바로 쓰지 않고, 메모리 버퍼에 모았다가 FLUSH_EVERY마다 writelines
        if self.log_file:
            try:
                line = row_to_csv_line(row) + "\n"
                self._pending_lines.append(line)
                self._written_lines += 1
                if len(self._pending_lines) >= FLUSH_EVERY:
                    self._flush_pending_lines()
            except Exception as e:
                self.status_bar.showMessage(f"File write error: {e}")

        try:
            self.mode_label.setText(
                f"H10Mode: {int(row[1])},  H10AssistLevel: {int(row[2])},  SmartAssist: {int(row[3])}"
            )
        except Exception:
            pass

        # 우측 state 패널 업데이트
        try:
            if "h10Mode" in self.state_labels:
                self.state_labels["h10Mode"].setText(f"h10Mode: {int(row[1])}")
        except Exception:
            pass

        try:
            if "tau_max_setting" in self.state_labels:
                idx = CSV_COLS.index("tau_max_setting")
                self.state_labels["tau_max_setting"].setText(f"tau_max_setting: {int(row[idx])}")
        except Exception:
            pass

        try:
            if "s_scaling_X" in self.state_labels:
                idx = CSV_COLS.index("s_scaling_X")
                self.state_labels["s_scaling_X"].setText(f"s_scaling_X: {float(row[idx]):.4f}")
        except Exception:
            pass

        try:
            if "s_scaling_Y" in self.state_labels:
                idx = CSV_COLS.index("s_scaling_Y")
                self.state_labels["s_scaling_Y"].setText(f"s_scaling_Y: {float(row[idx]):.4f}")
        except Exception:
            pass

        # is_moving 신호등 + HC scatter 포인트 추가
        try:
            is_moving_idx = CSV_COLS.index("is_moving")
            is_moving_val = int(row[is_moving_idx])
            self._set_is_moving_indicator(is_moving_val)
        except Exception:
            pass

        # hc_count가 증가할 때만 (s_norm_vel_HC, s_norm_T_HC) 점 추가
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
            self.status_bar.showMessage(
                f"Recv #{self._recv_count}, LoopCnt={row[0]}, LeftHipAngle={v:.4f}"
            )
        except Exception:
            self.status_bar.showMessage(f"Recv #{self._recv_count}, LoopCnt={row[0]}")

    def update_all_plots(self):
        if not self.data_all:
            return
        rows = self.data_all[-WINDOW_SIZE_SAMPLES:]
        if not rows:
            return
        x_vals = [r[0] for r in rows]

        # Plot 1~3: time series (PlotDataItem.setData)
        for plot_idx in range(3):
            pw = self.plot_widgets[plot_idx]
            group = self.plot_groups[plot_idx]
            items = self._ts_items[plot_idx] if plot_idx < len(self._ts_items) else {}
            for name in group:
                try:
                    col_idx = CSV_COLS.index(name)
                except ValueError:
                    continue
                y_vals = [r[col_idx] for r in rows]
                item = items.get(name)
                if item is not None:
                    item.setData(x_vals, y_vals)
            if x_vals:
                pw.setXRange(min(x_vals), max(x_vals), padding=0.01)

        # Plot 4: scatter (fixed axes) - cumulative points (do not clear)
        if self._hc_scatter_item is not None:
            spots = []
            for x, y, br in zip(self._hc_points_x, self._hc_points_y, self._hc_points_brush):
                color = br.color() if isinstance(br, QtGui.QBrush) else QtGui.QColor("#6b7280")
                spots.append({
                    "pos": (x, y),
                    "brush": br,
                    "pen": pg.mkPen(color),
                    "size": 8,
                    "symbol": "o",
                })
            self._hc_scatter_item.setData(spots)

    # ---------- 스레드/파일 종료 & 기타 슬롯 ----------
    @pyqtSlot(str)
    def on_status_msg(self, msg):
        self.status_bar.showMessage(msg)

    @pyqtSlot(str)
    def on_connection_failed(self, msg):
        self._close_log_file()
        QtWidgets.QMessageBox.critical(self, "Serial Connection Failed", msg)

    @pyqtSlot()
    def on_serial_finished(self):
        # 스레드 정리
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait()
            self.serial_thread = None
        self.serial_worker = None

        # 파일 닫기 (내부에서 pending flush)
        self._close_log_file()

        # 버튼 상태
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.status_bar.showMessage("Disconnected")

        # 저장된 최신 CSV 열어 리뷰 뷰어 표시
        self._open_review_viewer()

    @pyqtSlot(int)
    def on_packet_error_count(self, n):
        self.error_label.setText(f"Packet errors: {n}")

    def closeEvent(self, event):
        if self.serial_worker is not None:
            self.serial_worker.stop()
        if self.serial_thread is not None:
            self.serial_thread.quit()
            self.serial_thread.wait()
            self.serial_thread = None
        self._close_log_file()
        event.accept()


# ===================== 5) 엔트리 포인트 =====================

def main():
    app = QtWidgets.QApplication(sys.argv)

    # 전체 GUI 스타일 적용
    apply_modern_style(app)

    # pyqtgraph 기본 테마
    pg.setConfigOptions(antialias=True)
    pg.setConfigOption("background", "w")
    pg.setConfigOption("foreground", "#111827")

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
