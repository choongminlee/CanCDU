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

# Edit message items only here. type is Python struct format: f=float32, i=int32.
MESSAGE_FIELDS = [
    # name                 type scale               csv_fmt
    ("time_gps_hhmmss",    "i", 1,                  "d"),
    ("timestamp_sec",      "f", 1.0,                ".3f"),
    ("sample0_dt_sec",     "f", 1.0,                ".6f"),
    ("sample0_gyro_x_dps", "f", 1.0,                ".6f"),
    ("sample0_gyro_y_dps", "f", 1.0,                ".6f"),
    ("sample0_gyro_z_dps", "f", 1.0,                ".6f"),
    ("sample0_acc_x_mpss", "f", 1.0,                ".6f"),
    ("sample0_acc_y_mpss", "f", 1.0,                ".6f"),
    ("sample0_acc_z_mpss", "f", 1.0,                ".6f"),
    ("sample0_temp_c",     "f", 1.0,                ".2f"),
    ("sample1_dt_sec",     "f", 1.0,                ".6f"),
    ("sample1_gyro_x_dps", "f", 1.0,                ".6f"),
    ("sample1_gyro_y_dps", "f", 1.0,                ".6f"),
    ("sample1_gyro_z_dps", "f", 1.0,                ".6f"),
    ("sample1_acc_x_mpss", "f", 1.0,                ".6f"),
    ("sample1_acc_y_mpss", "f", 1.0,                ".6f"),
    ("sample1_acc_z_mpss", "f", 1.0,                ".6f"),
    ("sample1_temp_c",     "f", 1.0,                ".2f"),
    ("roll_deg",           "f", 1.0,                ".3f"),
    ("pitch_deg",          "f", 1.0,                ".3f"),
    ("yaw_deg",            "f", 1.0,                ".3f"),
    ("vn_e_mps",           "f", 1.0,                ".3f"),
    ("vn_n_mps",           "f", 1.0,                ".3f"),
    ("vn_u_mps",           "f", 1.0,                ".3f"),
    ("nav_lat_deg",        "i", 1.0 / 10000000.0,   ".7f"),
    ("nav_lon_deg",        "i", 1.0 / 10000000.0,   ".7f"),
    ("nav_hgt_m",          "f", 1.0,                ".3f"),
    ("gps_lat_deg",        "i", 1.0 / 10000000.0,   ".7f"),
    ("gps_lon_deg",        "i", 1.0 / 10000000.0,   ".7f"),
    ("gps_hgt_m",          "f", 1.0,                ".3f"),
<<<<<<< Updated upstream
    # ("gps_ve_mps",         "f", 1.0,                ".3f"),
    # ("gps_vn_mps",         "f", 1.0,                ".3f"),
    # ("gps_vu_mps",         "f", 1.0,                ".3f"),
    # ("pos_fix",            "i", 1,                  "d"),
    # ("north_heading_deg",  "f", 1.0,                ".1f"),
    # ("ground_speed_mps",   "f", 1.0,                ".3f"),
=======
    ("gps_ve_mps",         "f", 1.0,                ".3f"),
    ("gps_vn_mps",         "f", 1.0,                ".3f"),
    ("gps_vu_mps",         "f", 1.0,                ".3f"),
    ("pos_fix",            "i", 1,                  "d"),
    ("north_heading_deg",  "f", 1.0,                ".1f"),
    ("ground_speed_mps",   "f", 1.0,                ".3f"),
    ("gyro_bias_err_x_dps", "f", 1.0,                ".6f"),
    ("gyro_bias_err_y_dps", "f", 1.0,                ".6f"),
    ("gyro_bias_err_z_dps", "f", 1.0,                ".6f"),
    ("accl_bias_err_x_mpss", "f", 1.0,                ".6f"),
    ("accl_bias_err_y_mpss", "f", 1.0,                ".6f"),
    ("accl_bias_err_z_mpss", "f", 1.0,                ".6f"),
>>>>>>> Stashed changes
]

PAYLOAD_STRUCT = struct.Struct("<" + "".join(field_type for _, field_type, _, _ in MESSAGE_FIELDS))
PAYLOAD_SIZE = PAYLOAD_STRUCT.size
FRAME_SIZE = 8
START_ID = 0x101
FRAME_COUNT = (PAYLOAD_SIZE + FRAME_SIZE - 1) // FRAME_SIZE
END_ID = START_ID + FRAME_COUNT - 1
REQUIRED_IDS = set(range(START_ID, END_ID + 1))
CHANNEL_NUMBER = 0
BITRATE = BITRATES["250K"]

now = time.localtime()
output_path = (
    f"LT_IEKF_REV_{now.tm_year % 100:02d}{now.tm_mon:02d}{now.tm_mday:02d}_"
    f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.txt"
)

MAX_FRAME_COUNT = 32
MAX_ID = START_ID + MAX_FRAME_COUNT - 1

rx_payload = bytearray(MAX_FRAME_COUNT * FRAME_SIZE)
received_ids = set()
packet_count = 0
short_frames = 0
layout_mismatch_count = 0
incomplete_packets = 0
old_sec = -1
old_mismatch_sec = -1
start_time = time.time()

with open(output_path, mode="w", newline="", encoding="utf-8") as output_file:
    writer = csv.writer(output_file)
    writer.writerow([name for name, _, _, _ in MESSAGE_FIELDS])
    output_file.flush()
    print(f"TXT logging to: {output_path}", flush=True)
    print(
        f"Expecting CAN IDs 0x{START_ID:03X}..0x{END_ID:03X}, "
        f"payload={PAYLOAD_SIZE}B, frames={FRAME_COUNT}",
        flush=True,
    )

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
                    if frame.id < START_ID or frame.id > MAX_ID:
                        continue

                    data = bytes(frame.data)
                    if len(data) < FRAME_SIZE:
                        short_frames += 1
                        continue

                    if frame.id == START_ID:
                        if received_ids:
                            max_id = max(received_ids)
                            observed_required_ids = set(range(START_ID, max_id + 1))
                            elapsed_sec = time.time() - start_time

                            if not observed_required_ids.issubset(received_ids):
                                incomplete_packets += 1
                            elif max_id != END_ID:
                                layout_mismatch_count += 1
                                sec = int(elapsed_sec)
                                if sec != old_mismatch_sec:
                                    print(
                                        f"[PAYLOAD_MISMATCH] expected CAN IDs "
                                        f"0x{START_ID:03X}..0x{END_ID:03X} "
                                        f"({FRAME_COUNT} frames, {PAYLOAD_SIZE}B), "
                                        f"but received 0x{START_ID:03X}..0x{max_id:03X} "
                                        f"({max_id - START_ID + 1} frames). "
                                        f"mismatch: {layout_mismatch_count}, "
                                        f"incomplete: {incomplete_packets}, "
                                        f"short: {short_frames}",
                                        flush=True,
                                    )
                                    old_mismatch_sec = sec
                            elif received_ids == REQUIRED_IDS:
                                raw_values = PAYLOAD_STRUCT.unpack(bytes(rx_payload[:PAYLOAD_SIZE]))
                                values = {
                                    name: raw_value * scale
                                    for raw_value, (name, _, scale, _) in zip(raw_values, MESSAGE_FIELDS)
                                }

                                packet_count += 1

                                row = []
                                for name, _, _, output_format in MESSAGE_FIELDS:
                                    value = values[name]
                                    if output_format == "d":
                                        row.append(str(int(value)))
                                    else:
                                        row.append(f"{value:{output_format}}")
                                writer.writerow(row)
                                output_file.flush()

                                sec = int(elapsed_sec)
                                if sec != old_sec:
                                    get = values.get
                                    print(
                                        f"[{get('time_gps_hhmmss', 0)}] "
                                        f"[{get('timestamp_sec', 0.0):.1f}] "
<<<<<<< Updated upstream
                                        # f"[dt {get('dt_sec', 0.0):.6f}s] "
                                        f"[{int(get('kf_update', 0))}] "
                                        f"[gyro] {get('gyro_x_dps', 0.0):.3f}, "
                                        f"{get('gyro_y_dps', 0.0):.3f}, "
                                        f"{get('gyro_z_dps', 0.0):.3f}, "
                                        f"[acc] {get('acc_x_mpss', 0.0):.3f}, "
                                        f"{get('acc_y_mpss', 0.0):.3f}, "
                                        f"{get('acc_z_mpss', 0.0):.3f}, "
                                        f"[temp] {get('temp_c', 0.0):.2f}C, "
=======
                                        # f"[s0 dt] {get('sample0_dt_sec', 0.0):.6f}s, "
                                        # f"[s1 dt] {get('sample1_dt_sec', 0.0):.6f}s, "
                                        # f"[s0 gyro] {get('sample0_gyro_x_dps', 0.0):.3f}, "
                                        # f"{get('sample0_gyro_y_dps', 0.0):.3f}, "
                                        # f"{get('sample0_gyro_z_dps', 0.0):.3f}, "
                                        # f"[s1 gyro] {get('sample1_gyro_x_dps', 0.0):.3f}, "
                                        # f"{get('sample1_gyro_y_dps', 0.0):.3f}, "
                                        # f"{get('sample1_gyro_z_dps', 0.0):.3f}, "
                                        f"[temp] {get('sample1_temp_c', 0.0):.2f}C, "
>>>>>>> Stashed changes
                                        f"[att] {get('roll_deg', 0.0):.3f}, "
                                        f"{get('pitch_deg', 0.0):.3f}, "
                                        f"{get('yaw_deg', 0.0):.3f}, "
                                        f"[vn] {get('vn_e_mps', 0.0):.3f}, "
                                        f"{get('vn_n_mps', 0.0):.3f}, "
                                        f"{get('vn_u_mps', 0.0):.3f}, "
                                        f"[pos] {get('nav_lat_deg', 0.0):.7f}, "
                                        f"{get('nav_lon_deg', 0.0):.7f}, "
                                        f"{get('nav_hgt_m', 0.0):.3f}, "
                                        f"[g-pos] {get('gps_lat_deg', 0.0):.7f}, "
                                        f"{get('gps_lon_deg', 0.0):.7f}, "
                                        f"{get('gps_hgt_m', 0.0):.2f}, ",

                                        flush=True,
                                    )
                                    old_sec = sec

                        rx_payload[:] = b"\x00" * len(rx_payload)
                        received_ids.clear()

                    offset = (frame.id - START_ID) * FRAME_SIZE
                    rx_payload[offset:offset + FRAME_SIZE] = data[:FRAME_SIZE]
                    received_ids.add(frame.id)

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
