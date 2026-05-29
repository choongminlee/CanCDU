import csv
import os
import sys
import time
import struct


if hasattr(sys, "_MEIPASS"):
    base = sys._MEIPASS
    if os.path.isdir(base):
        os.add_dll_directory(base)
    kvaser_dir = os.path.join(base, "kvaser_dlls")
    if os.path.isdir(kvaser_dir):
        os.add_dll_directory(kvaser_dir)
else:
    exe_dir = os.path.dirname(
        sys.executable if getattr(sys, "frozen", False) else os.path.abspath(__file__)
    )
    if os.path.isdir(exe_dir):
        os.add_dll_directory(exe_dir)
    kvaser_dir = os.path.join(exe_dir, "kvaser_dlls")
    if os.path.isdir(kvaser_dir):
        os.add_dll_directory(kvaser_dir)
    default_kvaser_dir = r"C:\Program Files\Kvaser\Drivers"
    if os.path.isdir(default_kvaser_dir):
        os.add_dll_directory(default_kvaser_dir)

from canlib import canlib


BITRATES = {
    "1M": canlib.Bitrate.BITRATE_1M,
    "500K": canlib.Bitrate.BITRATE_500K,
    "250K": canlib.Bitrate.BITRATE_250K,
    "125K": canlib.Bitrate.BITRATE_125K,
    "100K": canlib.Bitrate.BITRATE_100K,
    "62K": canlib.Bitrate.BITRATE_62K,
    "50K": canlib.Bitrate.BITRATE_50K,
    "83K": canlib.Bitrate.BITRATE_83K,
    "10K": canlib.Bitrate.BITRATE_10K,
}

SIGNALS = [
    (0x101, "timestamp_sec", "f", 1.0, ".3f"),
    (0x102, "dt_sec", "f", 1.0, ".6f"),
    (0x103, "kf_update", "i", 1, "d"),
    (0x104, "gyro_x_dps", "f", 1.0, ".6f"),
    (0x105, "gyro_y_dps", "f", 1.0, ".6f"),
    (0x106, "gyro_z_dps", "f", 1.0, ".6f"),
    (0x107, "acc_x_mpss", "f", 1.0, ".6f"),
    (0x108, "acc_y_mpss", "f", 1.0, ".6f"),
    (0x109, "acc_z_mpss", "f", 1.0, ".6f"),
    (0x10A, "temp_c", "f", 1.0, ".2f"),
    (0x10B, "roll_deg", "f", 1.0, ".3f"),
    (0x10C, "pitch_deg", "f", 1.0, ".3f"),
    (0x10D, "yaw_deg", "f", 1.0, ".3f"),
    (0x10E, "vn_e_mps", "f", 1.0, ".3f"),
    (0x10F, "vn_n_mps", "f", 1.0, ".3f"),
    (0x110, "vn_u_mps", "f", 1.0, ".3f"),
    (0x111, "nav_lat_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x112, "nav_lon_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x113, "nav_hgt_m", "f", 1.0, ".3f"),
    (0x114, "gps_lat_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x115, "gps_lon_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x116, "gps_hgt_m", "f", 1.0, ".3f"),
    (0x117, "gps_ve_mps", "f", 1.0, ".3f"),
    (0x118, "gps_vn_mps", "f", 1.0, ".3f"),
    (0x119, "gps_vu_mps", "f", 1.0, ".3f"),
    (0x11A, "pos_fix", "i", 1, "d"),
    (0x11B, "north_heading_deg", "f", 1.0, ".1f"),
    (0x11C, "ground_speed_mps", "f", 1.0, ".3f"),
    (0x11D, "gyro_bias_x_deg", "f", 1.0, ".6f"),
    (0x11E, "gyro_bias_y_deg", "f", 1.0, ".6f"),
    (0x11F, "gyro_bias_z_deg", "f", 1.0, ".6f"),
]

SIGNAL_BY_ID = {
    signal_id: (index, name, data_type, scale, output_format)
    for index, (signal_id, name, data_type, scale, output_format) in enumerate(SIGNALS)
}
REQUIRED_IDS = {signal_id for signal_id, _, _, _, _ in SIGNALS}
START_ID = SIGNALS[0][0]
END_ID = SIGNALS[-1][0]
CHANNEL_NUMBER = 0
BITRATE = BITRATES["250K"]

now = time.localtime()
output_path = (
    f"LT_IEKF_REV_{now.tm_year % 100:02d}{now.tm_mon:02d}{now.tm_mday:02d}_"
    f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.csv"
)

values = [None] * len(SIGNALS)
received_ids = set()
packet_count = 0
old_sec = -1
start_time = time.time()

with open(output_path, mode="w", newline="", encoding="utf-8") as output_file:
    writer = csv.writer(output_file)
    writer.writerow(["elapsed_sec"] + [name for _, name, _, _, _ in SIGNALS])
    print(f"CSV logging to: {output_path}", flush=True)

    while True:
        ch = None
        try:
            ch = canlib.openChannel(CHANNEL_NUMBER, bitrate=BITRATE)
            ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
            ch.busOn()
            print("Connected to CAN channel.", flush=True)

            while True:
                try:
                    frame = ch.read(timeout=50)
                    signal = SIGNAL_BY_ID.get(frame.id)
                    if signal is None:
                        continue

                    data = bytes(frame.data)
                    if len(data) < 4:
                        continue

                    if frame.id == START_ID:
                        received_ids.clear()
                        values = [None] * len(SIGNALS)

                    index, name, data_type, scale, output_format = signal
                    raw_value = struct.unpack_from("<" + data_type, data, 0)[0]
                    values[index] = raw_value * scale
                    received_ids.add(frame.id)

                    if frame.id == END_ID and received_ids == REQUIRED_IDS:
                        elapsed_sec = time.time() - start_time
                        packet_count += 1

                        row = [f"{elapsed_sec:.3f}"]
                        for index, (_, _, _, _, output_format) in enumerate(SIGNALS):
                            row.append(f"{values[index]:{output_format}}")
                        writer.writerow(row)
                        output_file.flush()

                        sec = int(elapsed_sec)
                        if sec != old_sec:
                            timestamp_sec = values[SIGNAL_BY_ID[0x101][0]]
                            kf_update = values[SIGNAL_BY_ID[0x103][0]]
                            temp_c = values[SIGNAL_BY_ID[0x10A][0]]
                            roll_deg = values[SIGNAL_BY_ID[0x10B][0]]
                            pitch_deg = values[SIGNAL_BY_ID[0x10C][0]]
                            yaw_deg = values[SIGNAL_BY_ID[0x10D][0]]
                            vn_e_mps = values[SIGNAL_BY_ID[0x10E][0]]
                            vn_n_mps = values[SIGNAL_BY_ID[0x10F][0]]
                            vn_u_mps = values[SIGNAL_BY_ID[0x110][0]]
                            nav_lat_deg = values[SIGNAL_BY_ID[0x111][0]]
                            nav_lon_deg = values[SIGNAL_BY_ID[0x112][0]]
                            nav_hgt_m = values[SIGNAL_BY_ID[0x113][0]]
                            gps_lat_deg = values[SIGNAL_BY_ID[0x114][0]]
                            gps_lon_deg = values[SIGNAL_BY_ID[0x115][0]]
                            gps_hgt_m = values[SIGNAL_BY_ID[0x116][0]]
                            gps_ve_mps = values[SIGNAL_BY_ID[0x117][0]]
                            gps_vn_mps = values[SIGNAL_BY_ID[0x118][0]]
                            gps_vu_mps = values[SIGNAL_BY_ID[0x119][0]]
                            pos_fix = values[SIGNAL_BY_ID[0x11A][0]]
                            north_heading_deg = values[SIGNAL_BY_ID[0x11B][0]]
                            ground_speed_mps = values[SIGNAL_BY_ID[0x11C][0]]
                            gyro_bias_x_deg = values[SIGNAL_BY_ID[0x11D][0]]
                            gyro_bias_y_deg = values[SIGNAL_BY_ID[0x11E][0]]
                            gyro_bias_z_deg = values[SIGNAL_BY_ID[0x11F][0]]
                            print(
                                f"[{timestamp_sec:.1f}] [{kf_update}] "
                                f"[temp] {temp_c:.2f}, "
                                f"[att] {roll_deg:.3f}, {pitch_deg:.3f}, {yaw_deg:.3f}, "
                                f"[vel] {vn_e_mps:.3f}, {vn_n_mps:.3f}, {vn_u_mps:.3f}, "
                                f"[pos] {nav_lat_deg:.7f}, {nav_lon_deg:.7f}, {nav_hgt_m:.2f}, "
                                f"[g-pos] {gps_lat_deg:.7f}, {gps_lon_deg:.7f}, {gps_hgt_m:.2f}, "
                                f"[gpsvel] {gps_ve_mps:.3f}, {gps_vn_mps:.3f}, {gps_vu_mps:.3f}, "
                                f"[posfix] {pos_fix}, [NorthHead] {north_heading_deg:.1f}, "
                                f"[grd_speed] {ground_speed_mps:.1f}, "
                                f"[gyroBias] {gyro_bias_x_deg:.6f}, {gyro_bias_y_deg:.6f}, "
                                f"{gyro_bias_z_deg:.6f}, Rows: {packet_count}",
                                flush=True,
                            )
                            old_sec = sec

                except canlib.CanNoMsg:
                    pass
                except canlib.CanError as error:
                    print(f"CAN error: {error}", flush=True)
                    break

        except canlib.CanError as error:
            print(f"Connection failed: {error}. Retrying in 5 seconds...", flush=True)
            time.sleep(5)
        finally:
            if ch is not None:
                try:
                    ch.busOff()
                    ch.close()
                    print("Disconnected from CAN channel.", flush=True)
                except Exception:
                    pass

