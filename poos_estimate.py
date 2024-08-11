import time
from pymavlink import mavutil

# Main function to connect and receive messages
def main():
    # Create a connection to the autopilot (Pixhawk, etc.) via pymavlink
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # Wait for the heartbeat message to find the system ID
    master.wait_heartbeat()

    print("Waiting for position messages...")

    # Listen for position messages
    while True:
        try:
            msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            # Extracting position in NED coordinates
            x = msg.x  # in meters
            y = msg.y  # in meters
            z = msg.z  # in meters
            print(f"Current Position (NED): x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

        time.sleep(0.1)  # small delay to control the loop rate

if __name__ == "__main__":
    main()
