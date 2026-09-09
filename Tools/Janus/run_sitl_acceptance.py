#!/usr/bin/env python3
"""Run the GPS-only Janus cargo-drop acceptance flight in ArduPilot SITL."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

from pymavlink import mavutil


TARGET_ALTITUDE_M = 50.0
WIND_SPEED_MS = 10.0
PAYLOAD_MASS_KG = 2.0
PAYLOAD_DRAG_COEFFICIENT = 1.0
PAYLOAD_AREA_M2 = 0.07
MAX_HORIZONTAL_SPEED_MS = 0.25
MAX_IMPACT_ERROR_M = 1.0


class AcceptanceError(RuntimeError):
    pass


def wait_message(link, message_type: str, timeout: float):
    message = link.recv_match(type=message_type, blocking=True, timeout=timeout)
    if message is None:
        raise AcceptanceError(f"timeout waiting for {message_type}")
    return message


def set_parameter(link, name: str, value: float, timeout: float = 8.0) -> None:
    encoded = name.encode("ascii")
    deadline = time.monotonic() + timeout
    last_value = None
    for _ in range(2000):
        if link.recv_match(blocking=False) is None:
            break
    while time.monotonic() < deadline:
        link.mav.param_set_send(
            link.target_system, link.target_component, encoded, float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        link.mav.param_request_read_send(link.target_system, link.target_component, encoded, -1)
        retry_until = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < retry_until:
            message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if message is None:
                continue
            parameter_id = message.param_id
            if isinstance(parameter_id, bytes):
                parameter_id = parameter_id.decode("ascii")
            if parameter_id.rstrip("\x00") == name:
                last_value = message.param_value
                if math.isclose(message.param_value, value, abs_tol=1.0e-3):
                    return
    raise AcceptanceError(f"parameter {name} did not accept {value} (last acknowledgement={last_value})")


def receive_parameters(link, timeout: float = 35.0) -> dict[str, float]:
    link.mav.param_request_list_send(link.target_system, link.target_component)
    values: dict[str, float] = {}
    expected = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is None:
            continue
        parameter_id = message.param_id
        if isinstance(parameter_id, bytes):
            parameter_id = parameter_id.decode("ascii")
        values[parameter_id.rstrip("\x00")] = message.param_value
        expected = message.param_count
        if expected and len(values) >= expected:
            return values
    if len(values) < 500:
        raise AcceptanceError(f"incomplete parameter list ({len(values)}/{expected})")
    return values


def send_screen_selection(link) -> None:
    channels = [65535] * 18
    channels[8] = 1800
    link.mav.rc_channels_override_send(link.target_system, link.target_component, *channels)


def receive_osd_marker(osd_socket: socket.socket, link, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    last_selection = 0.0
    packet_count = 0
    last_lines = []
    while time.monotonic() < deadline:
        if time.monotonic() - last_selection > 0.5:
            send_screen_selection(link)
            last_selection = time.monotonic()
        try:
            packet, _ = osd_socket.recvfrom(2048)
        except TimeoutError:
            continue
        if len(packet) < 12:
            continue
        magic, version, columns, rows, _reserved, sequence = struct.unpack("!IBBBBI", packet[:12])
        if magic != 0x444F5344 or version != 1 or len(packet) != 12 + columns * rows:
            continue
        packet_count += 1
        framebuffer = packet[12:]
        lines = [framebuffer[row * columns:(row + 1) * columns] for row in range(rows)]
        last_lines = lines
        for row_index, row in enumerate(lines):
            for centre in (b"+X+", b"+<+", b"+>+", b"+^+", b"+v+"):
                column = row.find(centre)
                if column >= 0 and row_index > 0 and row_index + 1 < rows:
                    if lines[row_index - 1][column:column + 3] == b" + " and lines[row_index + 1][column:column + 3] == b" + ":
                        return {
                            "sequence": sequence,
                            "columns": columns,
                            "rows": rows,
                            "marker_x": column + 1,
                            "marker_y": row_index,
                            "marker": centre.decode("ascii"),
                            "nonblank_cells": sum(byte != 0 for byte in framebuffer),
                            "frame_hex": framebuffer.hex(),
                        }
    printable = ["".join(chr(byte) if 32 <= byte < 127 else "." for byte in row) for row in last_lines]
    raise AcceptanceError(
        f"OSD2 cargo-impact marker was not visible before takeoff; packets={packet_count}; frame={printable}"
    )


def wait_for_gps_navigation(link, timeout: float = 25.0) -> None:
    for message_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    ):
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 100000, 0, 0, 0, 0, 0,
        )
    deadline = time.monotonic() + timeout
    gps_ready = False
    local_ready = False
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.3)
        if message is None:
            continue
        if message.get_type() == "GPS_RAW_INT" and message.fix_type >= 3:
            gps_ready = True
        elif message.get_type() == "LOCAL_POSITION_NED":
            local_ready = True
        if gps_ready and local_ready:
            return
    raise AcceptanceError("GPS/EKF local navigation was not ready")


def wait_for_altitude(link, minimum_altitude_m: float, timeout: float):
    deadline = time.monotonic() + timeout
    latest = None
    last_reported = -100.0
    while time.monotonic() < deadline:
        message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != 1:
            continue
        latest = message
        altitude_m = message.relative_alt * 0.001
        if altitude_m - last_reported >= 2.0:
            print(f"altitude={altitude_m:.1f}m", flush=True)
            last_reported = altitude_m
        if altitude_m >= minimum_altitude_m:
            return message
    raise AcceptanceError(f"vehicle did not reach {minimum_altitude_m:.1f}m (last={latest})")


def wait_armed(link, timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if heartbeat is not None and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return
    raise AcceptanceError("vehicle did not arm")


def wait_stationary_state(link, maximum_wait: float):
    deadline = time.monotonic() + maximum_wait
    best = None
    while time.monotonic() < deadline:
        message = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.3)
        if message is None:
            continue
        horizontal_speed = math.hypot(message.vx, message.vy)
        if best is None or horizontal_speed < best[0]:
            best = (horizontal_speed, message)
        if horizontal_speed <= MAX_HORIZONTAL_SPEED_MS:
            return message, horizontal_speed
    if best is None:
        raise AcceptanceError("no local position was available at release altitude")
    return best[1], best[0]


def gps_offset_m(origin, destination) -> tuple[float, float]:
    latitude_rad = math.radians(origin.lat * 1.0e-7)
    north = (destination.lat - origin.lat) * 1.0e-7 * 111319.49079327357
    east = (destination.lon - origin.lon) * 1.0e-7 * 111319.49079327357 * math.cos(latitude_rad)
    return north, east


def parse_prediction(log_path: Path, release_time_ms: int) -> dict:
    reader = mavutil.mavlink_connection(str(log_path))
    closest = None
    release_seen = False
    while True:
        message = reader.recv_match(blocking=False)
        if message is None:
            break
        if message.get_type() == "SLUR" and getattr(message, "Rel", 0) == 1:
            release_seen = True
        if message.get_type() != "CIMP" or getattr(message, "Status", 0) != 1:
            continue
        delta_ms = abs(message.TimeUS * 0.001 - release_time_ms)
        if closest is None or delta_ms < closest[0]:
            closest = (delta_ms, message)
    if not release_seen:
        raise AcceptanceError("the dataflash log contains no payload release transition")
    if closest is None or closest[0] > 1000.0:
        raise AcceptanceError("no valid CIMP prediction was logged near release")
    message = closest[1]
    return {
        "north_m": float(message.N), "east_m": float(message.E), "down_m": float(message.D),
        "time_to_impact_s": float(message.TTI), "uncertainty_m": float(message.Unc),
        "log_delta_ms": closest[0],
    }


def run(args) -> dict:
    source = args.source.resolve()
    binary = (args.binary or source / "build/sitl/bin/arducopter").resolve()
    if not binary.is_file():
        raise AcceptanceError(f"SITL binary does not exist: {binary}")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    parameters = {
        # WIND_T=1 disables the default altitude profile, so the same 10 m/s
        # air-mass velocity is applied from release height down to the ground.
        "SIM_WIND_SPD": WIND_SPEED_MS, "SIM_WIND_DIR": 0,
        "SIM_WIND_TURB": 0, "SIM_WIND_T": 1,
        "SIM_SLUP_ENABLE": 1, "SIM_SLUP_WEIGHT": PAYLOAD_MASS_KG, "SIM_SLUP_LINELEN": 0,
        "SIM_SLUP_DRAG": PAYLOAD_DRAG_COEFFICIENT, "SIM_SLUP_RELEASE": 0,
        "CIMP_ENABLE": 1, "CIMP_MODE": 2, "CIMP_MASS": PAYLOAD_MASS_KG,
        "CIMP_CDA": PAYLOAD_DRAG_COEFFICIENT * PAYLOAD_AREA_M2,
        "CIMP_DELAY": 0, "CIMP_RHO": 1.225, "CIMP_WIND_SRC": 1,
        # SIM_WIND_DIR is a meteorological from-direction.
        "CIMP_WIND_N": -WIND_SPEED_MS, "CIMP_WIND_E": 0,
        "VISO_TYPE": 0, "EK3_SRC1_POSXY": 3, "EK3_SRC1_VELXY": 3,
        "OSD_TYPE": 2, "OSD_CHAN": 9, "OSD_SW_METHOD": 1,
        "OSD1_ENABLE": 1, "OSD1_CHAN_MIN": 900, "OSD1_CHAN_MAX": 1300,
        "OSD2_ENABLE": 1, "OSD2_CHAN_MIN": 1300, "OSD2_CHAN_MAX": 2100,
        "WP_SPD_UP": 8,
    }
    defaults_path = output / "janus-acceptance.parm"
    defaults_path.write_text(
        "\n".join(f"{name} {value}" for name, value in parameters.items()) + "\n",
        encoding="ascii",
    )
    log_file = (output / "sitl-console.log").open("w", encoding="utf-8")
    osd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    osd_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    osd_socket.bind(("127.0.0.1", 14670))
    osd_socket.settimeout(0.25)
    payload = mavutil.mavlink_connection("tcpin:127.0.0.1:5763", source_system=255)
    environment = dict(os.environ)
    environment["AP_OSD_UDP_ONLY"] = "1"
    command = [
        str(binary), "--model", "quad", "--speedup", str(args.speedup), "--wipe",
        "--defaults", f"{source / 'Tools/autotest/default_params/copter.parm'},{defaults_path}",
        # SERIAL2 otherwise binds TCP 5763, which is the upstream slung-payload
        # telemetry port consumed by this acceptance runner.
        "--serial2", "none",
    ]
    process = subprocess.Popen(command, cwd=output, env=environment, stdout=log_file, stderr=subprocess.STDOUT)
    main = None
    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AcceptanceError(f"ArduCopter SITL exited with code {process.returncode}")
            try:
                main = mavutil.mavlink_connection("tcp:127.0.0.1:5760", autoreconnect=False)
                heartbeat = main.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if heartbeat is not None:
                    main.target_system = heartbeat.get_srcSystem()
                    main.target_component = heartbeat.get_srcComponent()
                    break
                main.close()
                main = None
            except OSError:
                time.sleep(0.25)
        if main is None or main.target_system == 0:
            raise AcceptanceError("could not connect to ArduCopter SITL")

        all_parameters = receive_parameters(main)
        missing = sorted(set(parameters) - set(all_parameters))
        if missing:
            slung_names = sorted(name for name in all_parameters if "SLU" in name)
            raise AcceptanceError(f"missing parameters {missing}; available slung parameters: {slung_names}")
        mismatched = {
            name: (all_parameters[name], value)
            for name, value in parameters.items()
            if not math.isclose(all_parameters[name], value, abs_tol=1.0e-3)
        }
        if mismatched:
            raise AcceptanceError(f"SITL did not load acceptance defaults: {mismatched}")

        osd2_items = sorted(name for name in all_parameters if name.startswith("OSD2_") and name.endswith("_EN"))
        for name in osd2_items:
            set_parameter(main, name, 1 if name in {"OSD2_ALTITUDE_EN", "OSD2_CIMP_MRK_EN"} else 0)
        if "OSD2_ALTITUDE_EN" not in osd2_items or "OSD2_CIMP_MRK_EN" not in osd2_items:
            raise AcceptanceError("OSD2 does not expose both required settings")

        if wait_message(payload, "HEARTBEAT", 15).get_srcSystem() != 2:
            raise AcceptanceError("unexpected simulated payload MAVLink system ID")
        ground_payload = wait_message(payload, "GLOBAL_POSITION_INT", 5)
        wait_for_gps_navigation(main)
        preflight_osd = receive_osd_marker(osd_socket, main, 15)
        print(f"preflight_marker=visible at ({preflight_osd['marker_x']},{preflight_osd['marker_y']})", flush=True)

        guided = main.mode_mapping().get("GUIDED")
        if guided is None:
            raise AcceptanceError("GUIDED mode is unavailable")
        main.mav.set_mode_send(main.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, guided)
        main.arducopter_arm()
        wait_armed(main)
        main.mav.command_long_send(
            main.target_system, main.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, TARGET_ALTITUDE_M,
        )
        wait_for_altitude(main, 49.0, 45)
        local_position, horizontal_speed = wait_stationary_state(main, 8.0)
        release_height = -local_position.z
        if not 49.0 <= release_height <= 51.0:
            raise AcceptanceError(f"release altitude outside tolerance: {release_height:.2f}m")
        if horizontal_speed > MAX_HORIZONTAL_SPEED_MS:
            raise AcceptanceError(f"vehicle was moving horizontally at {horizontal_speed:.2f}m/s")
        airborne_osd = receive_osd_marker(osd_socket, main, 5)
        release_time_ms = local_position.time_boot_ms
        set_parameter(main, "SIM_SLUP_RELEASE", 1)
        print(f"released altitude={release_height:.2f}m horizontal_speed={horizontal_speed:.2f}m/s", flush=True)

        lifted = False
        landing = None
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            message = payload.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if message is None or message.get_srcSystem() != 2:
                continue
            altitude = message.relative_alt * 0.001
            if altitude > 10.0:
                lifted = True
            if lifted and altitude <= 0.05 and abs(message.vz) <= 5:
                landing = message
                break
        if landing is None:
            raise AcceptanceError("simulated payload did not land")
        actual_north, actual_east = gps_offset_m(ground_payload, landing)
        main.arducopter_disarm()
        time.sleep(1)
    finally:
        if main is not None:
            main.close()
        payload.close()
        osd_socket.close()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log_file.close()

    logs = sorted(output.rglob("*.BIN"), key=lambda path: path.stat().st_mtime)
    if not logs:
        raise AcceptanceError("SITL produced no dataflash log")
    prediction = parse_prediction(logs[-1], release_time_ms)
    impact_error = math.hypot(prediction["north_m"] - actual_north, prediction["east_m"] - actual_east)
    if impact_error > MAX_IMPACT_ERROR_M:
        raise AcceptanceError(f"impact prediction error {impact_error:.2f}m exceeds {MAX_IMPACT_ERROR_M:.2f}m")
    result = {
        "schema": "janus-sitl-cargo-acceptance/v1", "status": "PASS",
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip(),
        "gps_only": True, "visual_odometry": False, "gazebo": False, "rviz": False,
        "wind": {"speed_m_s": WIND_SPEED_MS, "turbulence": 0, "constant": True},
        "release": {"target_altitude_m": TARGET_ALTITUDE_M, "actual_altitude_m": release_height,
                    "horizontal_speed_m_s": horizontal_speed, "payload_mass_kg": PAYLOAD_MASS_KG,
                    "cda_m2": PAYLOAD_DRAG_COEFFICIENT * PAYLOAD_AREA_M2},
        "osd": {"navigation_screen": 1, "assistant_screen": 2,
                "assistant_enabled_items": ["ALTITUDE", "CIMP_MRK"],
                "marker_visible_before_takeoff": True,
                "preflight_frame": preflight_osd, "airborne_frame": airborne_osd},
        "prediction": prediction,
        "actual_impact": {"north_m": actual_north, "east_m": actual_east},
        "horizontal_error_m": impact_error, "acceptance_limit_m": MAX_IMPACT_ERROR_M,
        "dataflash_log": str(logs[-1]),
    }
    (output / "acceptance.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path.cwd())
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speedup", type=float, default=4.0)
    result = run(parser.parse_args())
    print(json.dumps({
        "status": result["status"], "release": result["release"],
        "predicted_impact": result["prediction"], "actual_impact": result["actual_impact"],
        "horizontal_error_m": result["horizontal_error_m"],
    }, indent=2))


if __name__ == "__main__":
    main()
