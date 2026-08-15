#!/usr/bin/env python3
"""
Run Plane SITL without GPS (EKF3 inertial nav) and summarise log health.

  Tools/autotest/sim_vehicle.py -v ArduPlane -f plane -w --speedup 20 \
    --add-param-file=Tools/autotest/default_params/plane-inertial-nav.parm

  Tools/scripts/plane_inertial_nav_sitl.py --fly --distance 100000

  Tools/scripts/plane_inertial_nav_sitl.py --log path/to/00000001.BIN
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

# XKF4.SS bits from libraries/AP_NavEKF/AP_Nav_Common.h
SS_CONST_POS = 1 << 7
SS_USING_GPS = 1 << 13
SS_HORIZ_POS_ABS = 1 << 4
SS_ATTITUDE = 1 << 0

CMAC_LAT = -35.363261
CMAC_LON = 149.165230


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


def wait_text(mav, needle, timeout=60):
    start = time.time()
    while time.time() - start < timeout:
        m = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if m is None:
            continue
        text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
        print("STATUSTEXT:", text)
        if needle.lower() in text.lower():
            return text
    raise TimeoutError("did not see %r" % needle)


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
        2,  # current
        0,
        0, 0, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
    )


def fly(args):
    mav = mavutil.mavlink_connection(args.master, autoreconnect=True)
    wait_heartbeat(mav)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    print("Waiting for inertial nav...")
    wait_text(mav, "using inertial nav", timeout=args.align_timeout)

    set_mode(mav, "TAKEOFF")
    mav.arducopter_arm()
    start = time.time()
    while time.time() - start < 30:
        m = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m and (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
    else:
        raise TimeoutError("arm failed")

    print("Waiting for altitude...")
    start = time.time()
    while time.time() - start < args.takeoff_timeout:
        m = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m and m.relative_alt > 40 * 1000:
            break
    else:
        raise TimeoutError("takeoff altitude not reached")

    dest_lat, dest_lon = offset_latlon(CMAC_LAT, CMAC_LON, args.distance, 0)
    set_mode(mav, "GUIDED")
    guided_wp(mav, dest_lat, dest_lon, 100)
    print("Guided to %.6f,%.6f (%.0f m north)" % (dest_lat, dest_lon, args.distance))

    start_lat = CMAC_LAT
    start_lon = CMAC_LON
    max_err = 0.0
    dist_flown = 0.0
    last_sim = None
    saw_const_pos = False
    saw_using_gps = False
    start = time.time()
    while time.time() - start < args.flight_timeout:
        m = mav.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        t = m.get_type()
        if t == "EKF_STATUS_REPORT":
            flags = m.flags
            if flags & mavlink.EKF_CONST_POS_MODE:
                saw_const_pos = True
                print("FAIL: EKF const-pos mode")
            # using_gps is not in EKF_STATUS_REPORT; check later in the log
        elif t == "GPS_RAW_INT":
            if m.fix_type >= 2:
                saw_using_gps = True
                print("FAIL: GPS fix_type=%u" % m.fix_type)
        elif t == "GLOBAL_POSITION_INT":
            dist_flown = gps_distance_m(start_lat, start_lon,
                                        m.lat * 1e-7, m.lon * 1e-7)
            if int(dist_flown) % 1000 < 50:
                print("AHRS dist from origin %.0f m" % dist_flown)
            if dist_flown >= args.distance * 0.98:
                print("Reached target distance")
                break
        elif t == "STATUSTEXT":
            text = m.text if isinstance(m.text, str) else m.text.decode("utf-8", "replace")
            print("STATUSTEXT:", text)
            if "DCM" in text and "active" in text.lower():
                print("FAIL: fell back to DCM")
            if "using GPS" in text:
                print("FAIL: EKF using GPS")

    mav.arducopter_disarm()
    time.sleep(2)
    print("Flight done. dist=%.0f m const_pos=%s gps_fix=%s" %
          (dist_flown, saw_const_pos, saw_using_gps))
    return 1 if (saw_const_pos or saw_using_gps) else 0


def analyse_log(path):
    print("Analysing", path)
    conn = mavutil.mavlink_connection(path)
    ss_const = 0
    ss_gps = 0
    ss_abs = 0
    ss_att = 0
    ss_n = 0
    gps_fix_max = 0
    dcm_msgs = []
    inertial_msgs = []
    gps_msgs = []
    max_horiz_err = 0.0
    dist_flown = 0.0
    last_sim = None
    origin_sim = None
    n_err = 0
    armed_err = False

    types = set(["XKF4", "GPS", "MSG", "SIM", "POS", "AHR2", "ATT", "XKF1", "MODE", "EV"])
    while True:
        m = conn.recv_match(type=list(types))
        if m is None:
            break
        t = m.get_type()
        if t == "XKF4" and getattr(m, "C", 0) == 0:
            ss_n += 1
            if m.SS & SS_CONST_POS:
                ss_const += 1
            if m.SS & SS_USING_GPS:
                ss_gps += 1
            if m.SS & SS_HORIZ_POS_ABS:
                ss_abs += 1
            if m.SS & SS_ATTITUDE:
                ss_att += 1
        elif t == "GPS":
            st = getattr(m, "Status", getattr(m, "FixType", 0))
            gps_fix_max = max(gps_fix_max, int(st))
        elif t == "MSG":
            msg = getattr(m, "Message", "")
            if "DCM" in msg:
                dcm_msgs.append(msg)
            if "inertial nav" in msg.lower():
                inertial_msgs.append(msg)
            if "using GPS" in msg:
                gps_msgs.append(msg)
        elif t == "SIM":
            sim_lat = m.Lat
            sim_lng = m.Lng
            if origin_sim is None:
                origin_sim = (sim_lat, sim_lng)
            if last_sim is not None:
                dist_flown += gps_distance_m(last_sim[0], last_sim[1], sim_lat, sim_lng)
            last_sim = (sim_lat, sim_lng)
        elif t == "POS" and last_sim is not None:
            err = gps_distance_m(m.Lat, m.Lng, last_sim[0], last_sim[1])
            max_horiz_err = max(max_horiz_err, err)
            n_err += 1

    print("XKF4 samples: %u" % ss_n)
    print("  attitude:        %u (%.1f%%)" % (ss_att, 100.0 * ss_att / max(1, ss_n)))
    print("  horiz_pos_abs:   %u (%.1f%%)" % (ss_abs, 100.0 * ss_abs / max(1, ss_n)))
    print("  const_pos_mode:  %u (%.1f%%)  (must be 0 after alignment)" %
          (ss_const, 100.0 * ss_const / max(1, ss_n)))
    print("  using_gps:       %u (%.1f%%)  (must be 0)" %
          (ss_gps, 100.0 * ss_gps / max(1, ss_n)))
    print("GPS max Status/FixType: %u (0 means no GPS)" % gps_fix_max)
    print("Inertial-nav MSG:", inertial_msgs[:5] or "NONE")
    print("DCM MSG:", dcm_msgs[:8] or "none")
    print("Using-GPS MSG:", gps_msgs[:5] or "none")
    print("Distance flown (SIM path): %.1f m" % dist_flown)
    print("Max AHRS vs SIM horiz error: %.2f m (%u samples)" % (max_horiz_err, n_err))

    ok = (ss_n > 0 and ss_gps == 0 and gps_fix_max == 0 and
          any("inertial nav" in x.lower() for x in inertial_msgs) and
          not any("AHRS: DCM active" in x for x in dcm_msgs))
    # const-pos is expected before inertial nav starts
    print("PASS" if ok else "CHECK FAILED")
    return 0 if ok else 1


def latest_bin(search_dirs):
    bins = []
    for d in search_dirs:
        bins.extend(glob.glob(os.path.join(d, "*.BIN")))
        bins.extend(glob.glob(os.path.join(d, "*.bin")))
    if not bins:
        return None
    return max(bins, key=os.path.getmtime)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--master", default="tcp:127.0.0.1:5760")
    p.add_argument("--fly", action="store_true")
    p.add_argument("--log", default=None)
    p.add_argument("--distance", type=float, default=100000.0,
                   help="northward guided distance, metres")
    p.add_argument("--align-timeout", type=float, default=90)
    p.add_argument("--takeoff-timeout", type=float, default=90)
    p.add_argument("--flight-timeout", type=float, default=7200)
    args = p.parse_args()
    rc = 0
    if args.fly:
        rc = fly(args)
    log = args.log
    if log is None and not args.fly:
        log = latest_bin(["logs", "Tools/autotest/logs", "."])
    if log:
        rc = analyse_log(log) or rc
    elif not args.fly:
        print("No log given; pass --log or --fly", file=sys.stderr)
        rc = 2
    sys.exit(rc)


if __name__ == "__main__":
    main()
