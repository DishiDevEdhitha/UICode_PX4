# filename: automation_px4.py (python)
import time
import logging
import math
from pymavlink import mavutil

# Keep using your modules for drop and any other mission-specific commands.
# Ensure modules.mavlink_commands.drop exists and is PX4-compatible. If not,
# you'll need to adapt that function as well (see notes below).
from modules.mavlink_commands import drop_actuator  # previously used drop() from the ArduPilot code
from modules.distcalc import *

# Operational constants
alti = 30           # desired altitude in meters
drop_time = 15      # wait time after drop in seconds
ACCURACY_METERS = 0.1
SETPOINT_SEND_RATE_HZ = 5  # how often to re-send setpoints (PX4 requires frequent setpoints in OFFBOARD)

# Helper: correct haversine using radians for cos()
def distance_lat_lon(lat1, lon1, lat2, lon2):
    """Return distance (meters) between two lat/lon pairs (decimal degrees)."""
    # convert degrees to radians
    rlat1 = math.radians(lat1)
    rlat2 = math.radians(lat2)
    dLat = rlat2 - rlat1
    dLon = math.radians(lon2 - lon1)
    a = math.sin(0.5 * dLat) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(0.5 * dLon) ** 2
    c = 2.0 * math.atan2(math.sqrt(abs(a)), math.sqrt(abs(1.0 - a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist


# PX4-specific helper functions ------------------------------------------------

def px4_set_mode_offboard(the_connection):
    """
    Request PX4 to switch to OFFBOARD mode. OFFBOARD lets an external companion computer
    send position setpoints. PX4 requires repeated setpoints once OFFBOARD is active.
    """
    # MAV_CMD_DO_SET_MODE = 176 (MAVLink standard)
    # base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1 << 7 is internal flag; use 1 here for clarity)
    # custom mode value for PX4 OFFBOARD differs by PX4 version; many setups use custom_mode = 6 for OFFBOARD.
    # We'll try to request OFFBOARD via COMMAND_LONG as a generic method; verify custom_mode for your PX4 build.
    # NOTE: depending on PX4 version you may need to use QGroundControl or param lookup to find the exact custom_mode int.
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            176,            # MAV_CMD_DO_SET_MODE
            0,
            1,              # base_mode (we're requesting custom mode enable)
            6,              # custom_mode (6 is commonly OFFBOARD on PX4 builds; verify for your PX4)
            0, 0, 0, 0, 0
        )
        # Blocking wait for ACK (optional, but helpful)
        ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        logging.getLogger().info("OFFBOARD mode request ACK: %s", ack)
    except Exception as e:
        logging.getLogger().error("Error requesting OFFBOARD mode: %s", e)


def px4_set_mode_auto(the_connection):
    """Switch PX4 to AUTO (mission) mode. Implementation depends on PX4 mapping."""
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            176, 0,
            1,      # base_mode
            4,      # custom_mode (4 often maps to AUTO.MISSION on common PX4 builds)
            0, 0, 0, 0, 0
        )
        ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        logging.getLogger().info("AUTO mode request ACK: %s", ack)
    except Exception as e:
        logging.getLogger().error("Error requesting AUTO mode: %s", e)


def px4_set_mode_loiter(the_connection):
    """Switch PX4 to LOITER/HOLD equivalent. Exact custom_mode integer may vary by PX4 version."""
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            176, 0,
            1,
            5,     # custom_mode (5 often maps to LOITER/POSCTL family on many PX4 builds)
            0, 0, 0, 0, 0
        )
        ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        logging.getLogger().info("LOITER mode request ACK: %s", ack)
    except Exception as e:
        logging.getLogger().error("Error requesting LOITER mode: %s", e)


# Main automation --------------------------------------------------------------

def automation(lati, longi, target_no, the_connection):
    """
    Move aircraft to the requested lat/lon (integer micro-degrees in same format you provided),
    wait until within accuracy, then call drop(target_no), wait drop_time, and optionally
    switch to AUTO/MISSION afterwards.

    Inputs:
      lati, longi : integers in micro-degrees (like GLOBAL_POSITION_INT uses)
      target_no    : user-facing ID for map/logs
      the_connection: pymavlink connection object
    """

    logging.getLogger().info("Repositioning to target: %s", target_no)

    # Read a fresh GPS message
    gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if gps is None:
        logging.getLogger().warning("No GLOBAL_POSITION_INT received; aborting automation.")
        return

    logging.getLogger().info("GPS during calculation: %s", gps)

    # Convert lati/longi (micro-degrees) to decimal degrees for distance calc
    desired_lat = lati / 1e7
    desired_lon = longi / 1e7
    current_lat = gps.lat / 1e7
    current_lon = gps.lon / 1e7

    calculated_distance = distance_lat_lon(desired_lat, desired_lon, current_lat, current_lon)
    logging.getLogger().info("Real_distance between lats and lons (m): %s", calculated_distance)

    # Read another GPS for altitude & freshness
    gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if gps is None:
        logging.getLogger().warning("No second GLOBAL_POSITION_INT received; continuing with previous GPS.")
    kurrentalti = (gps.relative_alt / 1000.0) if gps else None  # relative_alt is in millimeters

    logging.getLogger().info("Current relative altitude (m): %s", kurrentalti)
    logging.getLogger().info("Requesting OFFBOARD (guided) for position setpoints...")

    # PX4: prefer OFFBOARD for external setpoints (closest to ArduPilot GUIDED)
    px4_set_mode_offboard(the_connection)
    time.sleep(1.0)

    # Prepare and send a position setpoint. Use the MAV_FRAME_GLOBAL_INT constant from pymavlink
    # Keep sending the setpoint repeatedly until the vehicle reports it is within ACCURACY_METERS.
    # PX4 requires continuous setpoints to remain in OFFBOARD mode (or it will exit OFFBOARD).
    msg_time_ms = int(time.time() * 1000) & 0xFFFFFFFF

    # Build the set_position message once and re-send as needed
    def send_position_setpoint():
        the_connection.mav.set_position_target_global_int_send(
            msg_time_ms,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,   # global int frame
            0b0000111111111000,  # type_mask: ignore velocities/accels, use pos only (PX4 expects a bitmask)
            int(lati),          # lat_int (microdegrees)
            int(longi),         # lon_int (microdegrees)
            float(alti),        # altitude (m)
            0, 0, 0,            # vx, vy, vz (not used)
            0, 0, 0,            # afx, afy, afz (not used)
            0, 0               # yaw, yaw_rate (optional)
        )

    # initial setpoint
    send_position_setpoint()
    logging.getLogger().info("Sent initial setpoint to lat=%s lon=%s alt=%s", desired_lat, desired_lon, alti)
    msgs = "Repositioning for target: " + str(target_no)

    # Wait until the the_connection reaches the target location
    try:
        while True:
            # Send setpoint repeatedly (required in PX4 OFFBOARD)
            send_position_setpoint()
            # Sleep enough to maintain SETPOINT_SEND_RATE_HZ
            time.sleep(1.0 / SETPOINT_SEND_RATE_HZ)

            msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg is None:
                # No new GPS — continue sending setpoints
                continue

            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            distance = distance_lat_lon(current_lat, current_lon, desired_lat, desired_lon)

            # Save a human-readable string if other parts of app read target_dist
            global target_dist
            target_dist = "distance = " + str(distance)

            logging.getLogger().debug("Distance to target (m): %s", distance)
            if distance <= ACCURACY_METERS:
                logging.getLogger().info("Reached target (within %s m). Starting drop...", ACCURACY_METERS)
                msgs = "starting drop"
                time.sleep(1.0)
                # Call your drop implementation (make sure it's PX4 compatible)
                drop(target_no)

                time.sleep(drop_time)
                logging.getLogger().info("Drop complete. Drop location lat: %s lon: %s", msg.lat / 1e7, msg.lon / 1e7)

                # Switch back to AUTO (or whatever you prefer) after the drop
                px4_set_mode_auto(the_connection)
                break
    except Exception as e:
        logging.getLogger().exception("Exception in automation loop: %s", e)
        # Do minimal safe cleanup — don't crash the whole companion process.
        try:
            px4_set_mode_loiter(the_connection)
        except Exception:
            pass
