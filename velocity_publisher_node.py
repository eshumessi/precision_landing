#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose

# Constants for PID control (adjust these values as per your application)
Kp = 0.2  # Proportional gain (adjust as needed)
Kd = 0.1  # Derivative gain (adjust as needed)
MAX_VEL = 5.0  # Maximum allowed velocity ±5 m/s

# Publisher for velocity setpoints
vel_setpoint_pub = rospy.Publisher('/velocity_setpoint', Twist, queue_size=10)

# Global variables for error calculation
positions_x = []
positions_y = []
timestamps = []
landing_pos_x = 50
landing_pos_y = -50

def calculate_velocity_setpoint():
    # Ensure enough data points for derivative calculation
    if len(positions_x) > 1 and len(positions_y) > 1:
        err_X = landing_pos_x - positions_x[-1]  # Calculate error with correct sign
        err_Y = landing_pos_y - positions_y[-1]  # Calculate error with correct sign

        # Calculate derivative of errors (if enough data points)
        deriv_X = (err_X - (landing_pos_x - positions_x[-2])) / 0.1 if len(timestamps) > 1 else 0
        deriv_Y = (err_Y - (landing_pos_y - positions_y[-2])) / 0.1 if len(timestamps) > 1 else 0

        # Calculate velocity setpoints (PD control)
        vel_sp_X = Kp * err_X + Kd * deriv_X
        vel_sp_Y = Kp * err_Y + Kd * deriv_Y

        # Publish velocity setpoints as Twist message
        vel_sp = Twist()
        vel_sp.linear.x = vel_sp_X
        vel_sp.linear.y = vel_sp_Y
        vel_sp.linear.z = 0.0  # Assuming no vertical movement for simplicity
        vel_setpoint_pub.publish(vel_sp)

        # Print data (for debugging)
        rospy.loginfo(f"Error X: {err_X}, Error Y: {err_Y}, Vel SP X: {vel_sp_X}, Vel SP Y: {vel_sp_Y}")

def position_error_callback(msg):
    # Receive position error messages and store data
    pos_X = msg.position.x
    pos_Y = msg.position.y
    positions_x.append(pos_X)
    positions_y.append(pos_Y)
    timestamps.append(rospy.get_time())

    # Calculate velocity setpoints
    calculate_velocity_setpoint()

if __name__ == '__main__':
    rospy.init_node('velocity_publisher_node', anonymous=True)
    rospy.Subscriber('/position_current', Pose, position_error_callback)
    rospy.spin()
