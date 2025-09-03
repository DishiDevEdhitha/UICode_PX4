from pymavlink import mavutil

print("Connecting to PX4 SITL on udp:127.0.0.1:14550...")
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Waiting for multiple heartbeats...")
for i in range(5):
    msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    print(f"Heartbeat {i+1}: system={msg.get_srcSystem()}, component={msg.get_srcComponent()}")
