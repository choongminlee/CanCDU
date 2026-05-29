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
    (0x101, "lat_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x102, "lon_deg", "i", 1.0 / 10000000.0, ".7f"),
    (0x103, "hgt_m", "f", 1.0, ".3f"),
    (0x104, "roll_deg", "f", 1.0, ".3f"),
    (0x105, "pitch_deg", "f", 1.0, ".3f"),
    (0x106, "yaw_deg", "f", 1.0, ".3f"),
    (0x107, "speed_mps", "f", 1.0, ".3f"),
]

SIGNAL_BY_ID = {
    signal_id: (index, name, data_type, scale, output_format)
    for index, (signal_id, name, data_type, scale, output_format) in enumerate(SIGNALS)
}
REQUIRED_IDS = {signal_id for signal_id, _, _, _, _ in SIGNALS}
START_ID = SIGNALS[0][0]
END_ID = SIGNALS[-1][0]
CAN_CHANNEL_NUMBER = 0
BITRATE = BITRATES["250K"]

now = time.localtime()
output_path = (
    f"DT_LSMT_REV_{now.tm_year % 100:02d}{now.tm_mon:02d}{now.tm_mday:02d}_"
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
            ch = canlib.openChannel(CAN_CHANNEL_NUMBER, bitrate=BITRATE)
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
                        for index, _, _, _, output_format in [
                            (idx, name, data_type, scale, fmt)
                            for idx, (_, name, data_type, scale, fmt) in enumerate(SIGNALS)
                        ]:
                            row.append(f"{values[index]:{output_format}}")
                        writer.writerow(row)
                        output_file.flush()

                        sec = int(elapsed_sec)
                        if sec != old_sec:
                            lat_deg = values[SIGNAL_BY_ID[0x101][0]]
                            lon_deg = values[SIGNAL_BY_ID[0x102][0]]
                            hgt_m = values[SIGNAL_BY_ID[0x103][0]]
                            roll_deg = values[SIGNAL_BY_ID[0x104][0]]
                            pitch_deg = values[SIGNAL_BY_ID[0x105][0]]
                            yaw_deg = values[SIGNAL_BY_ID[0x106][0]]
                            speed_mps = values[SIGNAL_BY_ID[0x107][0]]
                            print(
                                f"Pos: {lat_deg:.7f}, {lon_deg:.7f}, {hgt_m:.3f}, "
                                f"Att: {roll_deg:.3f}, {pitch_deg:.3f}, {yaw_deg:.3f}, "
                                f"Speed: {speed_mps:.3f}m/s, Rows: {packet_count}",
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

