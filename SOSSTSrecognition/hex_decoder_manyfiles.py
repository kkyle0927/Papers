import os
import csv
import tkinter as tk
from tkinter import filedialog

# 필요 시 주석 해제
# import matplotlib.pyplot as plt
# import pandas as pd

# -----------------------------
# Decoding helpers
# -----------------------------
def decode_float16(hex_str, constant, dataSaveCnt):
    try:
        int_val = int(hex_str, 16)
        if int_val > 0x7FFF:
            int_val -= 0x10000
        return int_val / constant
    except ValueError as e:
        print(f"Error decoding float16: hex_str={hex_str}, constant={constant}, dataSaveCnt={dataSaveCnt}, error={e}")
        return None

def decode_uint8(hex_str, dataSaveCnt):
    try:
        return int(hex_str, 16)
    except ValueError as e:
        print(f"Error decoding uint8: hex_str={hex_str}, dataSaveCnt={dataSaveCnt}, error={e}")
        return None

def decode_uint16(hex_str, dataSaveCnt):
    try:
        return int(hex_str, 16)
    except ValueError as e:
        print(f"Error decoding uint16: hex_str={hex_str}, dataSaveCnt={dataSaveCnt}, error={e}")
        return None

def decode_uint32(hex_str):
    try:
        return int(hex_str, 16)
    except ValueError as e:
        print(f"Error decoding uint32: hex_str={hex_str}, error={e}")
        return None

def decode_uint64(hex_str):
    try:
        return int(hex_str, 16)
    except ValueError as e:
        print(f"Error decoding uint64: hex_str={hex_str}, error={e}")
        return None

# -----------------------------
# Constants
# -----------------------------
DATA_CONV_CONST_16BIT = 65535
GRF_RANGE_MAX = 4000
VOLT_RANGE_MAX = 60
DEG_ENC_RANGE_MAX = 720
VELDEG_ENC_RANGE_MAX = 6000
RAD_ENC_RANGE_MAX = 2261.94624
RADVEL_RANGE_MAX = 17.4533
CURRENT_RANGE_MAX = 360
GYRO_Z_RANGE_MAX = 1000
SUM_FUZZY_INPUT_RANGE_MAX = 1000
ACC_RANGE_MAX = 78.4532
ACC_FUZZYINPUT_RANGE_MAX = 202.63
MAG_RANGE_MAX = 2400
WC_RANGE_MAX = 20
MOTION_PHASE_RANGE_MAX = 200
TEMP_RANGE_MAX = 400
EMG_NORM_RANGE_MAX = 2
EMG_RAWSIGN_RANGE_MAX = 2
CONTROL_INPUT_MAX = 30

GRF_CONSTANT = DATA_CONV_CONST_16BIT / GRF_RANGE_MAX
VOLT_CONSTANT = DATA_CONV_CONST_16BIT / VOLT_RANGE_MAX
DEG_ENC_CONSTANT = DATA_CONV_CONST_16BIT / DEG_ENC_RANGE_MAX
VELDEG_ENC_CONSTANT = DATA_CONV_CONST_16BIT / VELDEG_ENC_RANGE_MAX
RAD_ENC_CONSTANT = DATA_CONV_CONST_16BIT / RAD_ENC_RANGE_MAX
RADVEL_CONSTANT = DATA_CONV_CONST_16BIT / RADVEL_RANGE_MAX
CURRENT_CONSTANT = DATA_CONV_CONST_16BIT / CURRENT_RANGE_MAX
GYRO_Z_CONSTANT = DATA_CONV_CONST_16BIT / GYRO_Z_RANGE_MAX
SUM_FUZ_INPUT_CONSTANT = DATA_CONV_CONST_16BIT / SUM_FUZZY_INPUT_RANGE_MAX
ACC_CONSTANT = DATA_CONV_CONST_16BIT / ACC_RANGE_MAX
ACC_FUZZYINPUT_CONSTANT = DATA_CONV_CONST_16BIT / ACC_FUZZYINPUT_RANGE_MAX
MAG_CONSTANT = DATA_CONV_CONST_16BIT / MAG_RANGE_MAX
WC_CONSTANT = DATA_CONV_CONST_16BIT / WC_RANGE_MAX
MOTION_PHASE_CONSTANT = DATA_CONV_CONST_16BIT / MOTION_PHASE_RANGE_MAX
TEMP_CONSTANT = DATA_CONV_CONST_16BIT / TEMP_RANGE_MAX
EMG_NORM_CONSTANT = DATA_CONV_CONST_16BIT / EMG_NORM_RANGE_MAX
EMG_RAWSIGN_CONSTANT = DATA_CONV_CONST_16BIT / EMG_RAWSIGN_RANGE_MAX
CONTROL_INPUT_CONSTANT = DATA_CONV_CONST_16BIT / CONTROL_INPUT_MAX

# -----------------------------
# Fields map
# -----------------------------
fields_info = {
    # CM Data #
    'loopCnt': ('uint32', None),
    'assist_level': ('float16', 100),

    # RH Data #
    'thighDeg_RH': ('float16', DEG_ENC_CONSTANT),
    'incPosDeg_RH': ('float16', DEG_ENC_CONSTANT),
    'MotorActCurrent_RH': ('float16', CURRENT_CONSTANT),
    'accX_Calib_RH': ('float16', ACC_CONSTANT),
    'accY_Calib_RH': ('float16', ACC_CONSTANT),
    'gyroZ_Calib_RH': ('float16', GYRO_Z_CONSTANT),

    # LH Data #
    'thighDeg_LH': ('float16', DEG_ENC_CONSTANT),
    'incPosDeg_LH': ('float16', DEG_ENC_CONSTANT),
    'MotorActCurrent_LH': ('float16', CURRENT_CONSTANT),
    'accX_Calib_LH': ('float16', ACC_CONSTANT),
    'accY_Calib_LH': ('float16', ACC_CONSTANT),
    'gyroZ_Calib_LH': ('float16', GYRO_Z_CONSTANT),

    # RK (Expansion) #
    'u_RH' : ('float16', CONTROL_INPUT_CONSTANT),
    'u_LH' : ('float16', CONTROL_INPUT_CONSTANT),
    'Pvector_ref_RH' : ('float16', DEG_ENC_CONSTANT),
    'Pvector_ref_LH' : ('float16', DEG_ENC_CONSTANT),
    'extension_control_mode' : ('uint8', None),
    'emg_R1' : ('float16', EMG_RAWSIGN_CONSTANT),
    'emg_L1' : ('float16', EMG_RAWSIGN_CONSTANT),
    'fsr_R1' : ('uint16', None),
    'fsr_R2' : ('uint16', None),
    'fsr_L1' : ('uint16', None),
    'fsr_L2' : ('uint16', None),
    'free_var1' : ('float16', VELDEG_ENC_CONSTANT),
    'free_var2' : ('float16', VELDEG_ENC_CONSTANT),
}

# -----------------------------
# Core logic
# -----------------------------
def write_csv_from_hex(hex_data_file, csv_file):
    fieldnames = list(fields_info.keys())

    with open(csv_file, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        with open(hex_data_file, 'rb') as hex_file:
            # 마지막 라인 제외하는 기존 로직 유지
            lines = hex_file.readlines()[:-1]
            for i, line in enumerate(lines):
                cleaned_line = line.replace(b'\x00', b'').decode('utf-8', errors='ignore').strip()
                hex_data = cleaned_line.strip()
                data = {}
                cursor = 0

                # loopCnt (uint32, 8 hex chars)
                data['loopCnt'] = decode_uint32(hex_data[cursor:cursor+8])
                cursor += 8

                # 나머지 필드 순서대로 파싱
                for field, (type_func, constant) in fields_info.items():
                    if field == 'loopCnt':
                        continue
                    try:
                        if type_func == 'float16' and cursor + 4 <= len(hex_data):
                            val = decode_float16(hex_data[cursor:cursor+4], constant, data['loopCnt'])
                            data[field] = f"{val:.3f}" if val is not None else None
                            cursor += 4
                        elif type_func == 'uint32' and cursor + 8 <= len(hex_data):
                            data[field] = decode_uint32(hex_data[cursor:cursor+8])
                            cursor += 8
                        elif type_func == 'uint8' and cursor + 2 <= len(hex_data):
                            data[field] = decode_uint8(hex_data[cursor:cursor+2], data['loopCnt'])
                            cursor += 2
                        elif type_func == 'uint16' and cursor + 4 <= len(hex_data):
                            data[field] = decode_uint16(hex_data[cursor:cursor+4], data['loopCnt'])
                            cursor += 4
                        else:
                            # 남은 데이터가 부족하면 None으로 둠
                            data[field] = None
                    except ValueError as e:
                        print(f"Error processing line {i}: error={e}")
                        continue

                writer.writerow(data)

def choose_files_and_decode():
    # Tk inter root 숨김
    root = tk.Tk()
    root.withdraw()
    root.update()

    file_paths = filedialog.askopenfilenames(
        title="디코딩할 파일(.csv) 선택 (여러 개 가능)",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )

    root.destroy()

    if not file_paths:
        print("선택된 파일이 없습니다.")
        return

    for src in file_paths:
        base, ext = os.path.splitext(src)
        dst = f"{base}-decoded{ext}"  # 원래 이름 + -decoded
        try:
            print(f"디코딩 시작: {src} -> {dst}")
            write_csv_from_hex(src, dst)
            print(f"완료: {dst}")
        except Exception as e:
            print(f"실패: {src} (에러: {e})")

if __name__ == "__main__":
    choose_files_and_decode()
