from pymavlink import mavutil
import time
import logging
from flask import jsonify

global is_connected
the_connection = None
is_connected = False

# -------------------------
# Connection
# -------------------------
def toggle_connection():
    print("Toggle connection was called")
    global the_connection, is_connected

    if is_connected:
        if the_connection:
            the_connection.close()
            the_connection = None
            is_connected = False
            logging.getLogger().info("Disconnected from PX4")
            return the_connection, is_connected

    try:
        the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        logging.getLogger().info("Connecting to PX4...")
        the_connection.wait_heartbeat()
        logging.getLogger().info(f"Heartbeat from system {the_connection.target_system}, component {the_connection.target_component}")
        is_connected = True
        return the_connection, is_connected
    except Exception as e:
        logging.getLogger().error(f"Failed to connect: {str(e)}")
        return None, False


# -------------------------
# Arm / Disarm
# -------------------------
def arm():
    global the_connection
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        return f"Armed: {msg}"
    except Exception as e:
        return f"Error in arm: {str(e)}"


def disarm():
    global the_connection
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        return f"Disarmed: {msg}"
    except Exception as e:
        return f"Error in disarm: {str(e)}"


# -------------------------
# Flight Commands
# -------------------------
def takeoffcommand(alt=10):
    global the_connection
    try:
        # Must be in Takeoff mode in PX4
        set_mode_px4("TAKEOFF")

        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        return jsonify({'status': 'success', 'message': str(msg)}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


def rtlcommand():
    global the_connection
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        return jsonify({'status': 'success', 'message': str(msg)}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


# -------------------------
# Mode Setting (PX4-specific)
# -------------------------
def set_mode_px4(mode_name):
    """
    Set PX4 flight mode
    """
    mode_map = {
        "STABILIZED": (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0),  # custom=0
        "TAKEOFF": (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1),     # custom=1
        "MISSION": (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4),     # auto-mission
        "RTL": (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6),         # RTL
        "LOITER": (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5),      # Hold/Loiter
    }

    if mode_name not in mode_map:
        return f"Unknown PX4 mode: {mode_name}"

    base_mode, custom_mode = mode_map[mode_name]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        base_mode,
        custom_mode
    )
    logging.getLogger().info(f"Mode set to {mode_name}")
    return f"Mode set to {mode_name}"


# -------------------------
# Servo/Actuator Control
# -------------------------
def drop_actuator(actuator_index, value):
    """
    PX4 alternative to ArduPilot DO_SET_SERVO
    actuator_index: which actuator (0-based)
    value: normalized [-1..1] or PWM if configured
    """
    global the_connection
    try:
        the_connection.mav.command_long_send(
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR,
            0, actuator_index, value, 0, 0, 0, 0, 0
        )
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        return f"Actuator {actuator_index} set to {value}: {msg}"
    except Exception as e:
        return f"Error setting actuator: {str(e)}"
