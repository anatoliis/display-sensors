#!/usr/bin/python3
import time

import serial
import serial.tools.list_ports

UPDATE_INTERVAL = 3
DEVICE_RECONNECT_DELAY = 4
DEVICES_IDS = ["USB VID:PID=1A86:55D4", "USB VID:PID=1A86:7523"]


def connect():
    print(f"Looking for devices: {DEVICES_IDS}")
    ports = serial.tools.list_ports.comports()
    device_port = None
    for port in ports:
        for device_id in DEVICES_IDS:
            if device_id in port.hwid:
                device_port = port.device
                print(f"Found device on {device_port}")
                break
    if not device_port:
        return None

    print("Connecting..")
    device = serial.Serial(port=device_port, baudrate=115200, timeout=2)
    if not device.isOpen():
        device.open()
    print("Connected.")
    return device


def read(file):
    try:
        file.seek(0)
        return round(float(file.read().strip()) / 100)
    except Exception:
        return -1


def close_files(*files):
    for file in files:
        if not file:
            continue
        try:
            file.close()
        except Exception:
            pass


def main():
    cpu_file = None
    gpu_file_1 = None
    gpu_file_2 = None
    gpu_file_3 = None
    nvme_file = None
    device = None

    try:
        while True:
            device = connect()
            if not device:
                print(f"Device not found. Retry in {DEVICE_RECONNECT_DELAY} seconds...")
                time.sleep(DEVICE_RECONNECT_DELAY)
                continue

            try:
                cpu_file = open(f"/sys/class/hwmon/hwmon2/temp1_input", "r")
                gpu_file_1 = open(f"/sys/class/hwmon/hwmon4/temp1_input", "r")
                gpu_file_2 = open(f"/sys/class/hwmon/hwmon4/temp2_input", "r")
                gpu_file_3 = open(f"/sys/class/hwmon/hwmon4/temp3_input", "r")
                nvme_file = open(f"/sys/class/hwmon/hwmon1/temp3_input", "r")
            except FileNotFoundError as e:
                raise RuntimeError(f"Failed to open: {e}")

            while True:
                try:
                    device.write(
                        (
                            f"{read(cpu_file)}"
                            f"{max(map(read, (gpu_file_1, gpu_file_2, gpu_file_3)))}"
                            f"{read(nvme_file)}"
                        ).encode("ascii")
                        + b"\x00"
                    )
                except Exception:
                    break
                time.sleep(UPDATE_INTERVAL)

            close_files(
                cpu_file,
                gpu_file_1,
                gpu_file_2,
                gpu_file_3,
                nvme_file,
            )
            if device and device.is_open:
                device.close()

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        close_files(
            cpu_file,
            gpu_file_1,
            gpu_file_2,
            gpu_file_3,
            nvme_file,
        )
        if device and device.is_open:
            device.close()


if __name__ == "__main__":
    main()
