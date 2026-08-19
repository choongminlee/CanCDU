import csv
import os
import struct
import sys
import time


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


CAN_CHANNEL_NUMBER = 0
BITRATE = canlib.Bitrate.BITRATE_250K
FRAME_SIZE = 8

CAN_ID_POSITION = 0x101
CAN_ID_HEIGHT_ATTITUDE = 0x102
CAN_ID_SPEED_FIX_KF = 0x103
REQUIRED_IDS = {
    CAN_ID_POSITION,
    CAN_ID_HEIGHT_ATTITUDE,
    CAN_ID_SPEED_FIX_KF,
}

POSITION_STRUCT = struct.Struct("<ii")
HEIGHT_ATTITUDE_STRUCT = struct.Struct("<HHHH")
SPEED_FIX_KF_STRUCT = struct.Struct("<HHH")

CSV_COLUMNS = (
    "elapsed_sec",
    "lat_deg",
    "lon_deg",
    "hgt_m",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "speed_mps",
    "pos_fix",
    # "kf_update_raw",
)


def decode_frame(can_id, data):
    if len(data) < FRAME_SIZE:
        raise ValueError(f"CAN ID 0x{can_id:03X}: expected 8 bytes, got {len(data)}")

    if can_id == CAN_ID_POSITION:
        lat_raw, lon_raw = POSITION_STRUCT.unpack_from(data)
        return {
            "lat_deg": lat_raw * 0.0000001,
            "lon_deg": lon_raw * 0.0000001,
        }

    if can_id == CAN_ID_HEIGHT_ATTITUDE:
        hgt_raw, roll_raw, pitch_raw, yaw_raw = HEIGHT_ATTITUDE_STRUCT.unpack_from(data)
        return {
            "hgt_m": hgt_raw * 0.125 - 2500.0,
            "roll_deg": roll_raw * 0.002 - 64.0,
            "pitch_deg": pitch_raw * 0.002 - 64.0,
            "yaw_deg": yaw_raw * 0.00573,
        }

    if can_id == CAN_ID_SPEED_FIX_KF:
        speed_raw, pos_fix_raw, kf_update_raw = SPEED_FIX_KF_STRUCT.unpack_from(data)
        return {
            "speed_mps": speed_raw * 0.01,
            "pos_fix": pos_fix_raw,
            # "kf_update_raw": kf_update_raw,
        }

    return None


def make_output_path():
    now = time.localtime()
    return (
        f"DT_LSMT_REV_{now.tm_year % 100:02d}{now.tm_mon:02d}{now.tm_mday:02d}_"
        f"{now.tm_hour:02d}{now.tm_min:02d}{now.tm_sec:02d}.csv"
    )


def write_sample(writer, elapsed_sec, values):
    writer.writerow(
        [
            f"{elapsed_sec:.3f}",
            values["lat_deg"],
            values["lon_deg"],
            values["hgt_m"],
            values["roll_deg"],
            values["pitch_deg"],
            values["yaw_deg"],
            values["speed_mps"],
            values["pos_fix"],
            # values["kf_update_raw"],
        ]
    )


def main():
    output_path = make_output_path()
    start_time = time.time()
    old_sec = -1
    received_ids = set()
    values = {}

    with open(output_path, mode="w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(CSV_COLUMNS)
        output_file.flush()

        print(f"CSV logging to: {output_path}", flush=True)
        print("Expecting standard CAN IDs 0x101, 0x102, 0x103 (8 bytes each)", flush=True)

        while True:
            channel = None
            try:
                channel = canlib.openChannel(CAN_CHANNEL_NUMBER, bitrate=BITRATE)
                channel.setBusOutputControl(canlib.canDRIVER_NORMAL)
                channel.busOn()
                print("Connected to CAN channel.", flush=True)

                while True:
                    try:
                        frame = channel.read(timeout=50)
                    except canlib.CanNoMsg:
                        continue
                    except canlib.CanError as error:
                        print(f"CAN error: {error}", flush=True)
                        break

                    if frame.id not in REQUIRED_IDS:
                        continue

                    data = bytes(frame.data)
                    if len(data) < FRAME_SIZE:
                        continue

                    if frame.id == CAN_ID_POSITION:
                        received_ids.clear()
                        values.clear()

                    values.update(decode_frame(frame.id, data))
                    received_ids.add(frame.id)

                    if frame.id != CAN_ID_SPEED_FIX_KF or received_ids != REQUIRED_IDS:
                        continue

                    elapsed_sec = time.time() - start_time
                    write_sample(writer, elapsed_sec, values)
                    output_file.flush()

                    sec = int(elapsed_sec)
                    if sec != old_sec:
                        print(
                            f"[pos] {values['lat_deg']:.7f}, {values['lon_deg']:.7f}, "
                            f"{values['hgt_m']:.3f}, "
                            f"[att] {values['roll_deg']:.3f}, {values['pitch_deg']:.3f}, "
                            f"{values['yaw_deg']:.5f}, "
                            f"[speed] {values['speed_mps']:.2f}, "
                            f"[pos_fix] {values['pos_fix']}, "
                            # f"[kf_update_raw] {values['kf_update_raw']}"
                            ,
                            flush=True,
                        )
                        old_sec = sec

                    received_ids.clear()
                    values.clear()

            except canlib.CanError as error:
                print(f"Connection failed: {error}. Retrying in 5 seconds...", flush=True)
                time.sleep(5)
            finally:
                if channel is not None:
                    try:
                        channel.busOff()
                        channel.close()
                        print("Disconnected from CAN channel.", flush=True)
                    except Exception:
                        pass


if __name__ == "__main__":
    main()
