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
    #===============< File Path Setup >===================
    FilePath = (
        f"LT_stop_PN2_No_SFErrComp_{nowTime.tm_year % 100:02d}{nowTime.tm_mon:02d}{nowTime.tm_mday:02d}_"
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
                            if len(temp_buffer) >= 80:

                                data = struct.unpack_from('<dddfffdidddd', temp_buffer[:80])


                                # gyro = data[0:3]        # 3 floats (deg/s) 12
                                # acc = data[3:6]        # 3 floats (m/s2) 12
                                # gyro_raw = data[6:9]        # 3 floats (deg/s) 12
                                # acc_raw = data[9:12]        # 3 floats (m/s2) 12
                                # temp = data[6]         # float 4

                                # yaw_rate_num_dps = data[13]  # float 4
                                # expected_yaw_rate_dps = data[1]  # float 4
                                # delphi_z_dps_mean = data[15]  # float 4
                                # winn_b_z_dps = data[16]  # float 4
                                # is_rotating = data[7]  # float 4
                                # GyroSFErr = data[7]

                                # att = data[0:3]        # 3 floats 12
                                # vel = data[16:19]        # 3 floats 12
                                # pos = data[19:22]        # 3 doubles 24
                                gpspos = data[0:3]    # 3 doubles 24
                                # gpsvel = data[19:22] # 12
                                # posFix = data[22]          # int 4
                                # VTGheading = data[23]      # float 4

                                att_aid = data[3:6]    # 3 doubles 24

                                KE8_maxabs = data[6]
                                kmax8 = data[7]
                                yaw_before_kf = data[8]
                                xk8_before_corr = data[9]
                                yaw_after_kf = data[10]
                                dyaw_kf = data[11]
                                

                                elapsed_time = time.time() - start_time
                                sec = int(elapsed_time)
                                if sec != old_sec:
                                    # print(cnt_CAN)
                                    print(
                                    # f"{cnt_CAN}"
                                    # f"[{elapsed_time:f}]",
                                    f"[{sec}]",
                                    # f"[gyro] {gyro[0]:f}, {gyro[1]:f}, {gyro[2]:f},"
                                    # f"[acc] {acc[0]:f}, {acc[1]:f}, {acc[2]:f}, "
                                    # f"[acc_raw] {acc_raw[0]:f}, {acc_raw[1]:f}, {acc_raw[2]:f}, "
                                    # f"[temp] {temp:f},"
                                    # f"[yaw_rate_num_dps] {yaw_rate_num_dps:f}, "

                                    f"[att_aid] rol: {att_aid[0]:.3f}, pit: {att_aid[1]:.3f}, yaw: {att_aid[2]:.3f}, "
                                    f"[GPS] lat: {gpspos[0]:.6f}, lon: {gpspos[1]:.6f}, "

                                    f"[KE8_maxabs] {KE8_maxabs:f}, "
                                    f"[kmax8] {kmax8:f} "
                                    f"[yaw_before_kf] {yaw_before_kf:f}, "
                                    f"[xk8_before_corr] {xk8_before_corr:f}, "
                                    f"[yaw_after_kf] {yaw_after_kf:f}, "
                                    f"[dyaw_kf] {dyaw_kf:f} "
                                )
                                                                    
                                    old_sec = sec
                                    cnt_CAN = 0

                                cnt_CAN = cnt_CAN + 1
                                # 콘솔 출력
                                # print(
                                #     # f"{cnt_CAN}"
                                #     # f"[{elapsed_time:f}]",
                                #     f"[{sec}]",
                                #     # f"[Gyro] gx: {gyro[0]:f},\tgy: {gyro[1]:f},\tgz: {gyro[2]:f},\t"
                                # ...
                                # )

                                # CSV 라인
                                csv_line = (
                                    f"{elapsed_time:f},"  # timestamp
                                    
                                    # f"{gyro[0]:f},"       #gyro
                                    # f"{gyro[1]:f},"       #gyro
                                    # f"{gyro[2]:f},"       #gyro
                                    # f"{acc[0]:f},"        #acc
                                    # f"{acc[1]:f},"        #acc
                                    # f"{acc[2]:f},"        #acc
                                    # f"{gyro_raw[0]:f},"       #gyro
                                    # f"{gyro_raw[1]:f},"       #gyro
                                    # f"{gyro_raw[2]:f},"       #gyro
                                    # f"{acc_raw[0]:f},"        #acc
                                    # f"{acc_raw[1]:f},"        #acc
                                    # f"{acc_raw[2]:f},"        #acc
                                    
                                    # f"{temp:f},"            #temp

                                    f"{att_aid[0]:f},"        # Roll deg
                                    f"{att_aid[1]:f},"        # Pitch deg
                                    f"{att_aid[2]:f},"        # Yaw deg
                                    # f"{vel[0]:f},"        # Ve
                                    # f"{vel[1]:f},"        # Vn
                                    # f"{vel[2]:f},"        # Vu
                                    # f"{pos[0]*R2D:f},"    # lat deg 
                                    # f"{pos[1]*R2D:f},"    # lon deg
                                    # f"{pos[2]:.3f},"      # hgt
                                    f"{gpspos[0]:f},"   # gps lat deg
                                    f"{gpspos[1]:f},"   # gps lon deg
                                    f"{gpspos[2]:.3f},"   # gps hgt m

                                    # f"{gpsvel[0]:f},"   # gps ve m/s
                                    # f"{gpsvel[1]:f},"   # gps vn m/s
                                    # f"{gpsvel[2]:f},"   # gps vu m/s
                                    f"{KE8_maxabs:f},"
                                    f"{kmax8:f},"
                                    f"{yaw_before_kf:f},"
                                    f"{xk8_before_corr:f},"
                                    f"{yaw_after_kf:f},"
                                    f"{dyaw_kf:f}"


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

                                # Att(deg)
                                # roll_data.append(att[0])
                                # pitch_data.append(att[1])
                                # yaw_data.append(att[2])

                                # Vel(m/s)
                                # ve_data.append(vel[0])
                                # vn_data.append(vel[1])
                                # vu_data.append(vel[2])

                                # Poscls
                                # lat_data.append(pos[0] * R2D)
                                # lon_data.append(pos[1] * R2D)
                                # hgt_data.append(pos[2])

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

                                # 사용한 128바이트 삭제
                                temp_buffer = temp_buffer[80:]

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
