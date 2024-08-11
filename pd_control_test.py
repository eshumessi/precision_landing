from pymavlink import mavutil
import threading
import time
import matplotlib.pyplot as plt

# Global variables to store position data
positions_x = []
positions_y = []
timestamps = []
landing_pos_x = 50
landing_pos_y = -50

# Constants for PID control (adjust these values as per your simulation dynamics)
Kp = 0.2  # Proportional gain (adjust as needed)
Kd = 0.1  # Derivative gain (adjust as needed)

# Maximum allowed velocity
MAX_VEL = 5.0  # ±5 m/s (reduce from previous ±10 m/s for stability)

# Establish a connection to the SITL simulation via UDP
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Example connection string for UDP and port 14551

# Function to send velocity command with velocity cap
def send_velocity_cmd(vx, vy, vz):
    # Cap velocities
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

# Function for PID control calculations
def calculate_control():
    while True:
        try:
            if timestamps:
                # Calculate errors from landing position
                err_X = landing_pos_x - positions_x[-1]  # Calculate error with correct sign
                err_Y = landing_pos_y - positions_y[-1]  # Calculate error with correct sign

                # Calculate derivative of errors (if enough data points)
                deriv_X = (err_X - (landing_pos_x - positions_x[-2])) / 0.1 if len(timestamps) > 1 else 0
                deriv_Y = (err_Y - (landing_pos_y - positions_y[-2])) / 0.1 if len(timestamps) > 1 else 0

                # Calculate velocity setpoints (PD control)
                vel_sp_X = Kp * err_X + Kd * deriv_X
                vel_sp_Y = Kp * err_Y + Kd * deriv_Y

                # Send velocity commands based on setpoints
                send_velocity_cmd(vel_sp_X, vel_sp_Y, 0)  # Sending x and y velocities, z velocity is 0

                # Print data (for debugging)
                print(f"Time: {timestamps[-1]}, Error X: {err_X}, Error Y: {err_Y}, Vel SP X: {vel_sp_X}, Vel SP Y: {vel_sp_Y}")

            time.sleep(0.1)  # Adjust as needed for control rate

        except KeyboardInterrupt:
            print("Stopping control calculations.")
            break

# Start a thread for PID control calculations
control_thread = threading.Thread(target=calculate_control, daemon=True)
control_thread.start()

# Function to handle simulated EKF position messages
def handle_position_messages():
    while True:
        try:
            msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=1.0)  # Receive LOCAL_POSITION_NED messages
            if msg is None:
                continue

            timestamp = time.time()  # current time in seconds since epoch
            pos_X = msg.x
            pos_Y = msg.y
            vel_X = msg.vx
            vel_Y = msg.vy

            # Store data in lists
            positions_x.append(pos_X)  # Store x position
            positions_y.append(pos_Y)  # Store y position
            timestamps.append(timestamp)

            # Print data (for debugging)
            print(f"Time: {timestamp}, Position X: {pos_X}, Position Y: {pos_Y}, Velocity X: {vel_X}, Velocity Y: {vel_Y}")

        except KeyboardInterrupt:
            print("Stopping position data reception.")
            break

# Start a thread for receiving position data
recv_thread = threading.Thread(target=handle_position_messages, daemon=True)
recv_thread.start()

# Main loop for plotting
try:
    plt.ion()  # Turn on interactive plotting mode
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    while True:
        # Update plot for X position and setpoint
        ax1.clear()
        if timestamps:
            ax1.plot(timestamps, positions_x, label='X Position')
            ax1.plot(timestamps, [landing_pos_x for _ in timestamps], '--', label='X Desired Position', color='r')
            ax1.set_xlabel('Time (seconds)')
            ax1.set_ylabel('Position (m)')
            ax1.set_title('X Position vs Time')
            ax1.legend()
            ax1.grid(True)

        # Update plot for Y position and setpoint
        ax2.clear()
        if timestamps:
            ax2.plot(timestamps, positions_y, label='Y Position')
            ax2.plot(timestamps, [landing_pos_y for _ in timestamps], '--', label='Y Desired Position', color='r')
            ax2.set_xlabel('Time (seconds)')
            ax2.set_ylabel('Position (m)')
            ax2.set_title('Y Position vs Time')
            ax2.legend()
            ax2.grid(True)

        plt.tight_layout()
        plt.pause(0.01)  # Pause to update plot

except KeyboardInterrupt:
    print("Keyboard interrupt detected, stopping the simulation.")

finally:
    control_thread.join()  # Wait for control thread to finish
    recv_thread.join()  # Wait for receive thread to finish
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep plots open after the program ends
