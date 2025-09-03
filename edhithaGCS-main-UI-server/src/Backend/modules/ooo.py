from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:192.168.1.50:14551')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# PX4: Switch to MISSION (AUTO) mode
the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1 = base_mode
    4,  # param2 = PX4 custom mode (4 = MISSION)
    0, 0, 0, 0, 0
)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
