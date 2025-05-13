import struct
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from canlib import canlib
import time
import threading

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

# Position (Lat, Lon, Hgt) - lat/lon stored in deg, hgt in meters
lat_data = []
lon_data = []
hgt_data = []

# Att  
roll_data = []
pitch_data = []
yaw_data = []

# speed
speed_data = []

# --------------------
# PLOTTING PARAMETERS
# --------------------
x_range = 500
x_increment = 10
y_update_interval = 1.0  

last_y_update_time_att = time.time()
last_y_update_time_speed = time.time()
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

ani_att = FuncAnimation(fig_att, update_attitude_graph, blit=False, interval=100)

# --------------------
# FIGURE 2: Speed
# --------------------
fig_speed, ax_speed = plt.subplots()
line_speed, = ax_speed.plot([], [], label="Ve (m/s)")

ax_speed.set_xlim(0, x_range)
ax_speed.set_ylim(-10, 10)  # initial guess
ax_speed.set_title("Real-Time Speed Data")
ax_speed.set_xlabel("Time (frames)")
ax_speed.set_ylabel("Speed (m/s)")
ax_speed.legend(loc='upper right')
ax_speed.grid(axis='y', linestyle='--', color='gray', alpha=0.7)

def update_speed_graph(frame):
    global x_data, speed_data
    global last_y_update_time_speed

    line_speed.set_data(x_data, ve_data)

    if len(x_data) > 0:
        ax_speed.set_xlim(max(0, x_data[-1] - x_range), x_data[-1])

    current_time = time.time()
    if current_time - last_y_update_time_speed >= y_update_interval and len(x_data) > 0:
        min_val =min(speed_data)
        max_val = max(speed_data)
        margin = (max_val - min_val) * 0.05 if (max_val - min_val) != 0 else 1
        ax_speed.set_ylim(min_val - margin, max_val + margin)
        last_y_update_time_speed = current_time

    return line_speed

ani_vel = FuncAnimation(fig_speed, update_speed_graph, blit=False, interval=100)

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

ani_pos = FuncAnimation(fig_pos, update_position_graph, blit=False, interval=100)

# ---------------------------------------------------
#  MONITOR FUNCTION (CAN read + data storage)
# ---------------------------------------------------
def monitor_channel(channel_number, bitrate):
    global x_data
    global lat_data, lon_data, hgt_data
    global roll_data, pitch_data, yaw_data    
    global speed_data

    output_file = open("output_data.txt", "a", encoding="utf-8")
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

            # 수신할 CAN ID를 0x107까지 확장
            valid_ids = {
                0x101, 0x102, 0x103, 0x104,
                0x105, 0x106, 0x107
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
                            if prev_status_dle != DLE_STATE:
                                if byte == ETX:
                                    status = END
                                elif byte == DLE:
                                    prev_status_dle = DLE_STATE
                                else:
                                    temp_buffer.append(byte)
                            else:
                                # 바로 직전에 DLE가 있었다면 그대로 데이터 처리
                                temp_buffer.append(byte)
                                prev_status_dle = NONE

                        if status == END:
                            if len(temp_buffer) >= 40:
                                # <fffffffffdddiffi 
                                # double*3 + float*4 = 40바이트
                                data = struct.unpack_from('<dddffff', temp_buffer[:40])

                                # 인덱스로 파싱
                                pos = data[0:3]    # 3 doubles (x, y, z)
                                att = data[3:6]        # 3 floats (Roll, Pitch, Yaw, rad)
                                speed = data[6]       # float

                                elapsed_time = time.time() - start_time

                                # 콘솔 출력
                                print(
                                    f"[{elapsed_time:.3f}]"
                                    f"[Pos] {pos[0]:.8f}, {pos[1]:.8f}, {pos[2]:.8f}, "
                                    f"[Att] rol: {att[0]:f}, pit: {att[1]:f}, yaw: {att[2]:f}, "
                                    f"[Speed] {speed:.3f}"
                                )

                                # CSV 라인
                                csv_line = (
                                    f"{elapsed_time:f},"    # timestamp
                                    f"{pos[0]:.8f},"    # lat deg 
                                    f"{pos[1]:.8f},"    # lon deg
                                    f"{pos[2]:.8f},"        # hgt 
                                    
                                    f"{att[0]:f},"      # Roll deg
                                    f"{att[1]:f},"      # Pitch deg
                                    f"{att[2]:f},"      # Yaw deg

                                    f"{speed:f}"           # speed m/s
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

                                # Pos
                                lat_data.append(pos[0])
                                lon_data.append(pos[1])
                                hgt_data.append(pos[2])

                                # Att(deg)
                                roll_data.append(att[0] * R2D)
                                pitch_data.append(att[1] * R2D)
                                yaw_data.append(att[2] * R2D)

                                # speed(m/s)
                                speed_data.append(speed)

                                # x_range만큼 초과 시 이전 데이터 제거
                                if len(x_data) > x_range:
                                    x_data = x_data[-x_range:]
                                    roll_data = roll_data[-x_range:]
                                    pitch_data = pitch_data[-x_range:]
                                    yaw_data = yaw_data[-x_range:]
                                    speed_data = speed_data[-x_range:]
  
                                    lat_data = lat_data[-x_range:]
                                    lon_data = lon_data[-x_range:]
                                    hgt_data = hgt_data[-x_range:]

                                temp_buffer = temp_buffer[40:]

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
