#!/usr/bin/env python3
"""
Plane SITL: fly to a 50 km waypoint with GPS for the first 20 km, then
inertial-only for the remaining 30 km. Reports miss distance vs the target.

Requires a SITL already running with plane-gps-then-inertial.parm:

  Tools/autotest/sim_vehicle.py -v ArduPlane -f plane -w -N --no-mavproxy \\
    --speedup 20 -L CMAC \\
    --add-param-file=Tools/autotest/default_params/plane-gps-then-inertial.parm

  Tools/scripts/plane_gps_then_coast_sitl.py --fly
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

CMAC_LAT = -35.363261
CMAC_LON = 149.165230
GYRO_BIAS_DEGH = 0.6
GYRO_BIAS_RADS = GYRO_BIAS_DEGH * math.pi / 180.0 / 3600.0


def gps_distance_m(lat1, lon1, lat2, lon2):
    r = 6378100.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def wait_heartbeat(mav, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        if mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1) is not None:
            return
    raise TimeoutError("no heartbeat")


def set_mode(mav, mode, timeout=15):
    mapping = mav.mode_mapping()
    if mapping is None or mode not in mapping:
        raise RuntimeError("mode map missing %s" % mode)
    mav.set_mode(mode)
    start = time.time()
    while time.time() - start < timeout:
        mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if mav.flightmode == mode:
            return
    raise TimeoutError("failed to enter %s (have %s)" % (mode, mav.flightmode))


def guided_wp(mav, lat, lon, alt_m):
    mav.mav.mission_item_int_send(
        mav.target_system,
        mav.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,
        0,
        0, 0, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
    )


def set_param(mav, name, value, timeout=5):
    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode("ascii"), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    start = time.time()
    while time.time() - start < timeout:
        m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is None:
            continue
        pname = m.param_id
        if isinstance(pname, bytes):
            pname = pname.decode("ascii", "replace")
        pname = pname.split("\x00", 1)[0]
        if pname == name:
            print("PARAM %s = %s" % (name, m.param_value))
            return m.param_value
    print("WARN: no PARAM_VALUE for %s" % name)
    return None


def get_param(mav, name, timeout=5):
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        name.encode("ascii"), -1)
    start = time.time()
    while time.time() - start < timeout:
        m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is None:
            continue
        pname = m.param_id
        if isinstance(pname, bytes):
            pname = pname.decode("ascii", "replace")
        pname = pname.split("\x00", 1)[0]
        if pname == name:
            return m.param_value
    return None


def rc_override(mav, throttle=1000):
    mav.mav.rc_channels_override_send(
        mav.target_system, mav.target_component,
        1500, 1500, throttle, 1500, 0, 0, 0, 0)


def arm_vehicle(mav, timeout=30):
    rc_override(mav, 1000)
    time.sleep(0.5)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0)
    start = time.time()
    while time.time() - start < timeout:
        rc_override(mav, 1000)
        m = mav.recv_match(blocking=True, timeout=0.5)
        if m is None:
            continue
        t = m.get_type()
        if t == "STATUSTEXT":
            text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            print("STATUSTEXT:", text)
        elif t == "COMMAND_ACK":
            print("ARM ack command=%s result=%s" % (m.command, m.result))
        elif t == "HEARTBEAT":
            if m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Armed")
                return
    raise TimeoutError("arm failed")


def snapshot(label, ahrs, sim, tgt, home, gps_fix, ekf_flags, extra=""):
    ahrs_lat, ahrs_lon = ahrs
    sim_lat, sim_lon = sim
    sim_home = gps_distance_m(home[0], home[1], sim_lat, sim_lon)
    ahrs_sim = gps_distance_m(ahrs_lat, ahrs_lon, sim_lat, sim_lon) if ahrs_lat else float("nan")
    sim_tgt = gps_distance_m(sim_lat, sim_lon, tgt[0], tgt[1])
    ahrs_tgt = gps_distance_m(ahrs_lat, ahrs_lon, tgt[0], tgt[1]) if ahrs_lat else float("nan")
    print("%s: SIM_from_home=%.1f m  AHRS_vs_SIM=%.2f m  SIM_vs_target=%.1f m  "
          "AHRS_vs_target=%.1f m  gps_fix=%s  ekf=0x%x %s" %
          (label, sim_home, ahrs_sim, sim_tgt, ahrs_tgt, gps_fix, ekf_flags, extra))
    return {
        "label": label,
        "sim_from_home": sim_home,
        "ahrs_vs_sim": ahrs_sim,
        "sim_vs_target": sim_tgt,
        "ahrs_vs_target": ahrs_tgt,
        "gps_fix": gps_fix,
        "ekf_flags": ekf_flags,
        "sim_lat": sim_lat,
        "sim_lon": sim_lon,
        "ahrs_lat": ahrs_lat,
        "ahrs_lon": ahrs_lon,
    }


def fly(args):
    mav = mavutil.mavlink_connection(args.master, autoreconnect=True)
    wait_heartbeat(mav)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    bias = get_param(mav, "SIM_GYR1_BIAS_X")
    opts = get_param(mav, "EK3_OPTIONS")
    print("SIM_GYR1_BIAS_X=%s (want %.6e rad/s = %.2f deg/h) EK3_OPTIONS=%s" %
          (bias, GYRO_BIAS_RADS, GYRO_BIAS_DEGH, opts))

    print("Waiting for GPS 3D fix and EKF position...")
    start = time.time()
    gps_ok = False
    ekf_ok = False
    while time.time() - start < args.align_timeout:
        m = mav.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        t = m.get_type()
        if t == "STATUSTEXT":
            text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            print("STATUSTEXT:", text)
        elif t == "GPS_RAW_INT" and m.fix_type >= 3:
            if not gps_ok:
                print("GPS 3D fix sats=%s" % m.satellites_visible)
            gps_ok = True
        elif t == "EKF_STATUS_REPORT":
            flags = m.flags
            if (flags & mavlink.EKF_ATTITUDE and
                    flags & mavlink.EKF_POS_HORIZ_ABS and
                    flags & mavlink.EKF_VELOCITY_HORIZ):
                ekf_ok = True
                print("EKF GPS-aided flags=0x%x" % flags)
        if gps_ok and ekf_ok:
            break
    else:
        raise TimeoutError("GPS/EKF not ready")

    tgt_lat, tgt_lon = offset_latlon(CMAC_LAT, CMAC_LON, args.total_km * 1000.0, 0.0)
    tgt = (tgt_lat, tgt_lon)
    home = (CMAC_LAT, CMAC_LON)
    print("Target %.0f km north: %.7f, %.7f" % (args.total_km, tgt_lat, tgt_lon))

    set_mode(mav, "TAKEOFF")
    arm_vehicle(mav)

    print("Waiting for takeoff altitude...")
    start = time.time()
    while time.time() - start < args.takeoff_timeout:
        rc_override(mav, 1500)
        m = mav.recv_match(blocking=True, timeout=0.5)
        if m is None:
            continue
        t = m.get_type()
        if t == "STATUSTEXT":
            text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            print("STATUSTEXT:", text)
        elif t == "GLOBAL_POSITION_INT" and m.relative_alt > 50 * 1000:
            print("Takeoff alt %.1f m" % (m.relative_alt / 1000.0))
            break
    else:
        raise TimeoutError("takeoff altitude not reached")

    print("GUIDED to target")
    set_mode(mav, "GUIDED")
    guided_wp(mav, tgt_lat, tgt_lon, args.alt)

    gps_loss_m = args.gps_km * 1000.0
    gps_disabled = False
    inertial_text = False
    saw_dcm = False
    saw_const_pos = False
    ahrs = (None, None)
    sim = (None, None)
    gps_fix = 0
    ekf_flags = 0
    gps_loss_snap = None
    arrive_snap = None
    closest_ahrs = None
    closest_sim_tgt = None
    start = time.time()
    next_report = start
    next_wp = start
    while time.time() - start < args.flight_timeout:
        rc_override(mav, 1600)
        m = mav.recv_match(blocking=True, timeout=0.5)
        if m is None:
            continue
        t = m.get_type()
        if t == "STATUSTEXT":
            text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            print("STATUSTEXT:", text)
            low = text.lower()
            if "using inertial nav" in low:
                inertial_text = True
            if "ahrs: dcm active" in low:
                saw_dcm = True
                print("FAIL: fell back to DCM")
        elif t == "EKF_STATUS_REPORT":
            ekf_flags = m.flags
            if flags_const_pos(m.flags):
                saw_const_pos = True
                print("FAIL: EKF const-pos mode flags=0x%x" % m.flags)
        elif t == "GPS_RAW_INT":
            gps_fix = m.fix_type
        elif t == "GLOBAL_POSITION_INT":
            lat = m.lat * 1e-7
            lon = m.lon * 1e-7
            if abs(lat) > 1.0 and abs(lon) > 1.0:
                ahrs = (lat, lon)
        elif t == "SIMSTATE":
            sim_lat = m.lat * 1e-7
            sim_lon = m.lng * 1e-7
            if abs(sim_lat) < 1.0:
                continue
            sim = (sim_lat, sim_lon)
            sim_home = gps_distance_m(home[0], home[1], sim_lat, sim_lon)

            if (not gps_disabled) and sim_home >= gps_loss_m:
                print("Reached %.1f km SIM — disabling GPS and switching to EK3 SRC2" %
                      (sim_home / 1000.0))
                gps_loss_snap = snapshot("GPS_LOSS", ahrs, sim, tgt, home, gps_fix, ekf_flags)
                set_param(mav, "SIM_GPS1_ENABLE", 0)
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
                    0, 2, 0, 0, 0, 0, 0, 0)
                gps_disabled = True
                continue

            if ahrs[0] is None:
                continue
            ahrs_tgt = gps_distance_m(ahrs[0], ahrs[1], tgt[0], tgt[1])
            sim_tgt = gps_distance_m(sim_lat, sim_lon, tgt[0], tgt[1])
            ahrs_sim = gps_distance_m(ahrs[0], ahrs[1], sim_lat, sim_lon)
            if closest_ahrs is None or ahrs_tgt < closest_ahrs["ahrs_vs_target"]:
                closest_ahrs = snapshot("CLOSEST_AHRS", ahrs, sim, tgt, home, gps_fix, ekf_flags)
            if closest_sim_tgt is None or sim_tgt < closest_sim_tgt["sim_vs_target"]:
                closest_sim_tgt = {
                    "sim_vs_target": sim_tgt,
                    "sim_from_home": sim_home,
                    "ahrs_vs_sim": ahrs_sim,
                    "ahrs_vs_target": ahrs_tgt,
                }

            if time.time() >= next_report:
                phase = "COAST" if gps_disabled else "GPS"
                print("%s SIM %.0f m  AHRS_vs_SIM %.1f m  SIM_vs_tgt %.0f m  "
                      "AHRS_vs_tgt %.0f m  fix=%s ekf=0x%x inertial=%s" %
                      (phase, sim_home, ahrs_sim, sim_tgt, ahrs_tgt,
                       gps_fix, ekf_flags, inertial_text))
                next_report = time.time() + 5
            if time.time() >= next_wp:
                guided_wp(mav, tgt_lat, tgt_lon, args.alt)
                next_wp = time.time() + 10

            arrived = (gps_disabled and ahrs_tgt < args.arrive_m and
                       sim_home > (args.total_km * 1000.0) * 0.6)
            overshot = sim_home >= args.total_km * 1000.0 * 1.15
            if arrived or overshot:
                arrive_snap = snapshot(
                    "ARRIVED" if arrived else "OVERSHOT",
                    ahrs, sim, tgt, home, gps_fix, ekf_flags,
                    extra=("inertial=%s dcm=%s const_pos=%s" %
                           (inertial_text, saw_dcm, saw_const_pos)))
                break

    mav.arducopter_disarm()
    time.sleep(1)
    print("\n========== RESULT ==========")
    print("IMU gyro bias: %.2f deg/h (%.6e rad/s on X/Y/Z)" %
          (GYRO_BIAS_DEGH, GYRO_BIAS_RADS))
    print("Plan: %.0f km GPS then %.0f km inertial, target %.0f km north of CMAC" %
          (args.gps_km, args.total_km - args.gps_km, args.total_km))
    if gps_loss_snap:
        print("At GPS loss: SIM_from_home=%.0f m  AHRS_vs_SIM=%.2f m  SIM_vs_target=%.0f m" %
              (gps_loss_snap["sim_from_home"], gps_loss_snap["ahrs_vs_sim"],
               gps_loss_snap["sim_vs_target"]))
    if closest_ahrs:
        print("When AHRS was closest to target (%.1f m AHRS):" %
              closest_ahrs["ahrs_vs_target"])
        print("  TRUE miss (SIM vs target) = %.1f m" % closest_ahrs["sim_vs_target"])
        print("  AHRS vs SIM               = %.1f m" % closest_ahrs["ahrs_vs_sim"])
        print("  SIM from home             = %.0f m" % closest_ahrs["sim_from_home"])
    if closest_sim_tgt:
        print("Closest SIM approach to target: %.1f m (SIM_from_home=%.0f m, AHRS_vs_SIM=%.1f m)" %
              (closest_sim_tgt["sim_vs_target"], closest_sim_tgt["sim_from_home"],
               closest_sim_tgt["ahrs_vs_sim"]))
    if arrive_snap:
        print("End (%s): SIM_vs_target=%.1f m  AHRS_vs_target=%.1f m  AHRS_vs_SIM=%.1f m" %
              (arrive_snap["label"], arrive_snap["sim_vs_target"],
               arrive_snap["ahrs_vs_target"], arrive_snap["ahrs_vs_sim"]))
    print("inertial_nav_msg=%s  dcm_fallback=%s  const_pos=%s" %
          (inertial_text, saw_dcm, saw_const_pos))
    miss = None
    if closest_ahrs and gps_disabled:
        miss = closest_ahrs["sim_vs_target"]
    print("DEVIATION FROM TARGET (SIM when AHRS closest) = %s m" %
          ("%.1f" % miss if miss is not None else "n/a"))
    return 0 if (gps_disabled and not saw_dcm and not saw_const_pos) else 1


def flags_const_pos(flags):
    return bool(flags & mavlink.EKF_CONST_POS_MODE)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--master", default="tcp:127.0.0.1:5760")
    p.add_argument("--fly", action="store_true")
    p.add_argument("--total-km", type=float, default=50.0)
    p.add_argument("--gps-km", type=float, default=20.0)
    p.add_argument("--alt", type=float, default=80.0)
    p.add_argument("--arrive-m", type=float, default=150.0)
    p.add_argument("--align-timeout", type=float, default=90)
    p.add_argument("--takeoff-timeout", type=float, default=90)
    p.add_argument("--flight-timeout", type=float, default=600)
    args = p.parse_args()
    if not args.fly:
        p.print_help()
        return 2
    return fly(args)


if __name__ == "__main__":
    raise SystemExit(main())
