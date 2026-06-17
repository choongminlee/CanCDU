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

# Edit LSMTRON message items only here. type is Python struct format: f=float32, i=int32.
MESSAGE_FIELDS = [
    # name          type scale               csv_fmt
    ("lat_deg",     "i", 1.0 / 10000000.0,   ".7f"),
    ("lon_deg",     "i", 1.0 / 10000000.0,   ".7f"),
    ("hgt_m",       "f", 1.0,                ".3f"),
    ("roll_deg",    "f", 1.0,                ".3f"),
    ("pitch_deg",   "f", 1.0,                ".3f"),
    ("yaw_deg",     "f", 1.0,                ".3f"),
    ("speed_mps",   "f", 1.0,                ".3f"),
]

PAYLOAD_STRUCT = struct.Struct("<" + "".join(field_type for _, field_type, _, _ in MESSAGE_FIELDS))
PAYLOAD_SIZE = PAYLOAD_STRUCT.size
FRAME_SIZE = 8
START_ID = 0x101
FRAME_COUNT = (PAYLOAD_SIZE + FRAME_SIZE - 1) // FRAME_SIZE
END_ID = START_ID + FRAME_COUNT - 1
REQUIRED_IDS = set(range(START_ID, END_ID + 1))
CAN_CHANNEL_NUMBER = 0
BITRATE = BITRATES["250K"]

now = time.localtime()
output_path = (
    f"DT_LSMT_REV_{now.tm_year % 100:02d}{now.tm_mon:02d}{now.tm_mday:02d}_"
    f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.csv"
)

rx_payload = bytearray(FRAME_COUNT * FRAME_SIZE)
received_ids = set()
packet_count = 0
short_frames = 0
old_sec = -1
start_time = time.time()

with open(output_path, mode="w", newline="", encoding="utf-8") as output_file:
    writer = csv.writer(output_file)
    writer.writerow(["elapsed_sec"] + [name for name, _, _, _ in MESSAGE_FIELDS])
    output_file.flush()
    print(f"CSV logging to: {output_path}", flush=True)
    print(
        f"Expecting CAN IDs 0x{START_ID:03X}..0x{END_ID:03X}, "
        f"payload={PAYLOAD_SIZE}B, frames={FRAME_COUNT}",
        flush=True,
    )

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
                    if frame.id < START_ID or frame.id > END_ID:
                        continue

                    data = bytes(frame.data)
                    if len(data) < FRAME_SIZE:
                        short_frames += 1
                        continue

                    if frame.id == START_ID:
                        rx_payload[:] = b"\x00" * len(rx_payload)
                        received_ids.clear()

                    offset = (frame.id - START_ID) * FRAME_SIZE
                    rx_payload[offset:offset + FRAME_SIZE] = data[:FRAME_SIZE]
                    received_ids.add(frame.id)

                    if frame.id == END_ID and received_ids == REQUIRED_IDS:
                        raw_values = PAYLOAD_STRUCT.unpack(bytes(rx_payload[:PAYLOAD_SIZE]))
                        values = {
                            name: raw_value * scale
                            for raw_value, (name, _, scale, _) in zip(raw_values, MESSAGE_FIELDS)
                        }

                        elapsed_sec = time.time() - start_time
                        packet_count += 1

                        row = [f"{elapsed_sec:.3f}"]
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
                                f"Pos: {get('lat_deg', 0.0):.7f}, "
                                f"{get('lon_deg', 0.0):.7f}, "
                                f"{get('hgt_m', 0.0):.3f}, "
                                f"Att: {get('roll_deg', 0.0):.3f}, "
                                f"{get('pitch_deg', 0.0):.3f}, "
                                f"{get('yaw_deg', 0.0):.3f}, "
                                f"Speed: {get('speed_mps', 0.0):.3f}m/s, "
                                f"Rows: {packet_count}, short: {short_frames}",
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
