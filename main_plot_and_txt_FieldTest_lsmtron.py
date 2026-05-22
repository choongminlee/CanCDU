# import struct

# # [pyinst_build debug] force-load pyexpat & non-GUI backend
# import pyexpat
# # print("DEBUG pyexpat from:", getattr(pyexpat, "__file__", "<builtin>"))
# import matplotlib
# # matplotlib.use("Agg")  # comment out later if GUI needed

# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation


# # --- Kvaser DLL path bootstrap (portable exe) ---
# import os, sys
# # _MEIPASS: PyInstaller onefile가 임시 추출하는 폴더
# if hasattr(sys, "_MEIPASS"):
#     os.add_dll_directory(sys._MEIPASS)
# else:
#     # 개발 환경: 로컬 Kvaser 설치 경로 추가 (필요 시 수정)
#     _kvaser_default = r"C:\Program Files\Kvaser\Drivers"
#     if os.path.isdir(_kvaser_default):
#         os.add_dll_directory(_kvaser_default)


# from canlib import canlib
# import time
# import threading
# import os

import os
import sys
import time
import struct
import threading

# --- PyInstaller / portable bootstrap --------------------------------------
# Ensure bundled DLLs (pyexpat, Kvaser) are found.
import pyexpat  # force stdlib ext load early; comment out print when done
# print("DEBUG pyexpat from:", getattr(pyexpat, "__file__", "<builtin>"))

def _add_dll_dirs():
    tried = []
    def _try(p):
        if p and os.path.isdir(p):
            os.add_dll_directory(p)
            tried.append(p)

    if hasattr(sys, "_MEIPASS"):
        base = sys._MEIPASS
        _try(base)
        _try(os.path.join(base, "kvaser_dlls"))
    else:
        exe_dir = os.path.dirname(sys.executable if getattr(sys, "frozen", False) else os.path.abspath(__file__))
        _try(exe_dir)
        _try(os.path.join(exe_dir, "kvaser_dlls"))
        _try(r"C:\Program Files\Kvaser\Drivers")

_add_dll_dirs()
# ---------------------------------------------------------------------------

# Matplotlib backend: choose Agg for headless / release
import matplotlib
# matplotlib.use("Agg")  # uncomment if no GUI on target PC

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from canlib import canlib  # must import *after* DLL dirs added

# 통신 제어 문자
STX = 0x02
DLE = 0x10
ETX = 0x03

# 상태 정의
READY = 0
START = 1
END = 2
NONE = 0
DLE_STATE = 1

R2D = 180.0 / 3.14159265358979323846

# CanCDU packet:
# timestamp, dt_sec, KFupdate,
# gyro_raw[3], acc_raw[3], gyro_calib[3], gyro_fullCorr[3], temp,
# att[3], vel[3], pos[3], gpspos[3], GyroBiasErr[3], sf_z_candidate



# --------------------
# GLOBAL DATA STORAGE
# --------------------
x_data = []
roll_data = []
pitch_data = []
yaw_data = []

# Velocity (Ve, Vn, Vu)
ve_data = []
vn_data = []
vu_data = []

# Position (Lat, Lon, Hgt) - lat/lon stored in deg, hgt in meters
lat_data = []
lon_data = []
hgt_data = []

cnt_CAN = 0
old_sec = 0
# --------------------
# PLOTTING PARAMETERS
# --------------------
x_range = 500
x_increment = 10
y_update_interval = 1.0  

last_y_update_time_att = time.time()
last_y_update_time_vel = time.time()
last_y_update_time_pos = time.time()

# --------------------
# FIGURE 1: ATTITUDE
# --------------------
fig_att, ax_att = plt.subplots()
line_roll, = ax_att.plot([], [], label="Roll (deg)")
line_pitch, = ax_att.plot([], [], label="Pitch (deg)")
line_yaw, = ax_att.plot([], [], label="Yaw (deg)")
ax_att.set_xlim(0, x_range)
ax_att.set_ylim(-180, 180)
ax_att.set_title("Real-Time Attitude Data (Roll, Pitch, Yaw)")
ax_att.set_xlabel("Time (frames)")
ax_att.set_ylabel("Angle (degrees)")
ax_att.legend(loc='upper right')
ax_att.grid(axis='y', linestyle='--', color='gray', alpha=0.7)

def update_attitude_graph(frame):
    global x_data, roll_data, pitch_data, yaw_data
    global last_y_update_time_att

    line_roll.set_data(x_data, roll_data)
    line_pitch.set_data(x_data, pitch_data)
    line_yaw.set_data(x_data, yaw_data)

    if len(x_data) > 0:
        ax_att.set_xlim(max(0, x_data[-1] - x_range), x_data[-1])

    current_time = time.time()
    if current_time - last_y_update_time_att >= y_update_interval and len(x_data) > 0:
        min_val = min(min(roll_data), min(pitch_data), min(yaw_data))
        max_val = max(max(roll_data), max(pitch_data), max(yaw_data))
        margin = (max_val - min_val) * 0.05 if (max_val - min_val) != 0 else 1
        ax_att.set_ylim(min_val - margin, max_val + margin)
        last_y_update_time_att = current_time

    return line_roll, line_pitch, line_yaw

ani_att = FuncAnimation(fig_att, update_attitude_graph, blit=False, interval=100, cache_frame_data=False)

# --------------------
# FIGURE 2: VELOCITY
# --------------------
fig_vel, ax_vel = plt.subplots()
line_ve, = ax_vel.plot([], [], label="Ve (m/s)")
line_vn, = ax_vel.plot([], [], label="Vn (m/s)")
line_vu, = ax_vel.plot([], [], label="Vu (m/s)")
ax_vel.set_xlim(0, x_range)
ax_vel.set_ylim(-10, 10)  # initial guess
ax_vel.set_title("Real-Time Velocity Data (Ve, Vn, Vu)")
ax_vel.set_xlabel("Time (frames)")
ax_vel.set_ylabel("Velocity (m/s)")
ax_vel.legend(loc='upper right')
ax_vel.grid(axis='y', linestyle='--', color='gray', alpha=0.7)

def update_velocity_graph(frame):
    global x_data, ve_data, vn_data, vu_data
    global last_y_update_time_vel

    line_ve.set_data(x_data, ve_data)
    line_vn.set_data(x_data, vn_data)
    line_vu.set_data(x_data, vu_data)

    if len(x_data) > 0:
        ax_vel.set_xlim(max(0, x_data[-1] - x_range), x_data[-1])

    current_time = time.time()
    if current_time - last_y_update_time_vel >= y_update_interval and len(x_data) > 0:
        min_val = min(min(ve_data), min(vn_data), min(vu_data))
        max_val = max(max(ve_data), max(vn_data), max(vu_data))
        margin = (max_val - min_val) * 0.05 if (max_val - min_val) != 0 else 1
        ax_vel.set_ylim(min_val - margin, max_val + margin)
        last_y_update_time_vel = current_time

    return line_ve, line_vn, line_vu

ani_vel = FuncAnimation(fig_vel, update_velocity_graph, blit=False, interval=100, cache_frame_data=False)

# --------------------
# FIGURE 3: POSITION (HGT ONLY)
# --------------------
fig_pos, ax_pos = plt.subplots()
line_hgt, = ax_pos.plot([], [], label="Hgt (m)")
ax_pos.set_xlim(0, x_range)
ax_pos.set_ylim(-10, 10)  # initial guess
ax_pos.set_title("Real-Time Position Data (Height Only)")
ax_pos.set_xlabel("Time (frames)")
ax_pos.set_ylabel("Height (m)")
ax_pos.legend(loc='upper right')
ax_pos.grid(axis='y', linestyle='--', color='gray', alpha=0.7)

def update_position_graph(frame):
    global x_data, hgt_data
    global last_y_update_time_pos

    line_hgt.set_data(x_data, hgt_data)

    if len(x_data) > 0:
        ax_pos.set_xlim(max(0, x_data[-1] - x_range), x_data[-1])

    current_time = time.time()
    if current_time - last_y_update_time_pos >= y_update_interval and len(x_data) > 0:
        min_val = min(hgt_data)
        max_val = max(hgt_data)
        margin = (max_val - min_val) * 0.05 if (max_val - min_val) != 0 else 1
        ax_pos.set_ylim(min_val - margin, max_val + margin)
        last_y_update_time_pos = current_time

    return line_hgt,

ani_pos = FuncAnimation(fig_pos, update_position_graph, blit=False, interval=100, cache_frame_data=False)

# ---------------------------------------------------
#  MONITOR FUNCTION (CAN read + data storage)
# ---------------------------------------------------
def monitor_channel(channel_number, bitrate):
    global x_data
    global roll_data, pitch_data, yaw_data
    global ve_data, vn_data, vu_data
    global lat_data, lon_data, hgt_data
    global cnt_CAN
    global old_sec

    # output_file = open("output_data_LT.txt", "a", encoding="utf-8")
    
    nowTime = time.localtime()

    FilePath = (
        f"DT_LSMT_{nowTime.tm_year % 100:02d}{nowTime.tm_mon:02d}{nowTime.tm_mday:02d}_"
        f"{nowTime.tm_hour:02d}{nowTime.tm_min:02d}{nowTime.tm_sec:02d}.txt"
        )

    output_file = open(FilePath, mode='w', encoding="utf-8")    
    start_time = time.time()

    while True:
        try:
            ch = canlib.openChannel(channel_number, bitrate=bitrate)
            ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
            ch.busOn()
            print("Connected to CAN channel.")

            temp_buffer = bytearray()
            status = READY
            prev_status_dle = NONE

            # 수신할 CAN ID를 0x10D까지 확장
            valid_ids = {
                0x101, 0x102, 0x103, 0x104,
                0x105, 0x106, 0x107, 0x108,
                0x109, 0x10A, 0x10B, 0x10C,
                0x10D, 0x10E, 0x10F, 0x110,
                0x111, 0x112, 0x113, 0x114,
                0x115, 0x116, 0x117, 0x118,
                0x119, 0x11A, 0x11B, 0x11C, 
            }

            while True:
                try:
                    frame = ch.read(timeout=50)
                    if frame.id not in valid_ids:
                        continue

                    for byte in frame.data:
                        if status == READY and byte == STX:
                            status = START
                            temp_buffer.clear()
                        elif status == START:
                            if prev_status_dle == DLE_STATE:          # 직전이 DLE였다면
                                temp_buffer.append(byte)              # 그 값을 그대로 기록
                                prev_status_dle = NONE
                                continue                              # ← ETX 검사하면 안 됨
                            if byte == DLE:                           # 새로 DLE 만남
                                prev_status_dle = DLE_STATE
                                continue
                            if byte == ETX:                           # 패킷 끝
                                status = END
                            else:
                                temp_buffer.append(byte)

                        if status == END:
                            # 수신 Byte 갯수
                            CANCDU_FIXED_FORMAT = '<f 3d 3f f'
                            CANCDU_FIXED_SIZE = struct.calcsize(CANCDU_FIXED_FORMAT)

                            if len(temp_buffer) >= CANCDU_FIXED_SIZE:
                                data = struct.unpack_from(CANCDU_FIXED_FORMAT, temp_buffer, 0)

                                timestamp = data[0]
                                pos = data[1:4]
                                att = data[4:7]
                                speed = data[7]

                                nmea_bytes = temp_buffer[CANCDU_FIXED_SIZE:]
                                nmea_size = len(nmea_bytes)
                                total_packet_size = CANCDU_FIXED_SIZE + nmea_size
                                str_NMEA = nmea_bytes.decode('ascii', errors='ignore').rstrip('\x00')
                                
                                # posfix = data[8]         # int 4
                                # NorthHead = data[9]     # float 4
                                # groundspeed = data[10]      # float 4


                                cur_time =time.time()
                                elapsed_time = cur_time - start_time
                                sec = int(elapsed_time)
                                if sec != old_sec:
                                    print(
                                        f"[{timestamp:.2f}s] "
                                        f"Pos: {pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.3f}, "
                                        f"Att: {att[0]:.3f}, {att[1]:.3f}, {att[2]:.3f}, "
                                        f"Speed: {speed:.3f}m/s, "
                                        f"NMEA: {str_NMEA} "
                                        f"({total_packet_size} bytes)"
                                    )
                                                                    
                                    old_sec = sec
                                    cnt_CAN = 0

                                cnt_CAN = cnt_CAN + 1

                                # CSV 라인
                                csv_line = (
                                    f"{timestamp:.2f},"
                                    f"{pos[0]:.6f},{pos[1]:.6f},{pos[2]:.3f},"
                                    f"{att[0]:.3f},{att[1]:.3f},{att[2]:.3f},"
                                    f"{speed:.1f},"
                                    f"{str_NMEA}"
                                )
                                output_file.write(csv_line + "\n")
                                output_file.flush()

                                # ------------------
                                # UPDATE PLOT DATA
                                # ------------------
                                if len(x_data) == 0:
                                    x_data.append(0)
                                else:
                                    x_data.append(x_data[-1] + x_increment)

                                # x_range만큼 초과 시 이전 데이터 제거
                                if len(x_data) > x_range:
                                    x_data = x_data[-x_range:]
                                    roll_data = roll_data[-x_range:]
                                    pitch_data = pitch_data[-x_range:]
                                    yaw_data = yaw_data[-x_range:]
                                    ve_data = ve_data[-x_range:]
                                    vn_data = vn_data[-x_range:]
                                    vu_data = vu_data[-x_range:]
                                    lat_data = lat_data[-x_range:]
                                    lon_data = lon_data[-x_range:]
                                    hgt_data = hgt_data[-x_range:]

                                # 사용한 패킷 길이만큼 삭제
                                # temp_buffer = temp_buffer[len_byte:]
                                temp_buffer.clear()

                            status = READY

                except canlib.CanNoMsg:
                    pass
                except canlib.CanError as e:
                    print(f"CAN error: {e}")
                    break

        except canlib.CanError as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            time.sleep(5)
        finally:
            try:
                ch.busOff()
                ch.close()
                print("Disconnected from CAN channel.")
            except Exception:
                pass

# --------------------
# MAIN ENTRY POINT
# --------------------
if __name__ == '__main__':
    BITRATES = {
        '1M': canlib.Bitrate.BITRATE_1M,
        '500K': canlib.Bitrate.BITRATE_500K,
        '250K': canlib.Bitrate.BITRATE_250K,
        '125K': canlib.Bitrate.BITRATE_125K,
        '100K': canlib.Bitrate.BITRATE_100K,
        '62K': canlib.Bitrate.BITRATE_62K,
        '50K': canlib.Bitrate.BITRATE_50K,
        '83K': canlib.Bitrate.BITRATE_83K,
        '10K': canlib.Bitrate.BITRATE_10K,
    }

    t = threading.Thread(target=monitor_channel, args=(0, BITRATES['250K']))
    t.daemon = True
    t.start()

    # plt.show()

        # 메인 스레드는 계속 살아 있게 유지
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("사용자에 의해 종료됨.")

