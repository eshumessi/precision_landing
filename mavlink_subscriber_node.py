#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pymavlink import mavutil
MAX_VEL = 5.0


def send_velocity_cmd(vx, vy, vz):
    # Cap velocities if necessary (adjust as per your requirements)
    vx_capped = min(MAX_VEL, max(-MAX_VEL, vx))
    vy_capped = min(MAX_VEL, max(-MAX_VEL, vy))
    
    msg = master.mav.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        master.target_system,  # target system
        master.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx_capped, vy_capped, vz,  # x, y, z velocity in m/s (capped)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    master.mav.send(msg)

def velocity_setpoint_callback(vel_sp_msg):
    # Receive Twist message and send velocity commands via MAVLink
    vx = vel_sp_msg.linear.x
    vy = vel_sp_msg.linear.y
    vz = vel_sp_msg.linear.z
    send_velocity_cmd(vx, vy, vz)

if __name__ == '__main__':
    rospy.init_node('mavlink_subscriber_node', anonymous=True)
    rospy.Subscriber('/velocity_setpoint', Twist, velocity_setpoint_callback)
    
    # Establish a connection to the SITL simulation via UDP
    master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Example connection string for UDP and port 14551
    print("establishing connection")
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    rospy.spin()
