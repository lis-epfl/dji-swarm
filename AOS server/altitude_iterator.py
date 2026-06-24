import time
import ds_wrapper as w

print("Starting altitude iteration test...")

drone_id = 1
altitude_increment = 10  # meters
max_altitude = 30  # meters above starting point
min_altitude = 5  # meters (starting altitude)
update_interval = 0.5  # seconds
waypoint_num = 1

try:
    # Get initial altitude
    image_telemetry_data = w.getImageAndTelemetryData(drone_id)
    telemetry_data = bytearray(image_telemetry_data[3110408:]).decode()
    telemetry_elements = telemetry_data.split(':')
    
    starting_altitude = float(telemetry_elements[2])
    print(f"Starting altitude: {starting_altitude} m")
    
    # Initialize current altitude offset
    altitude_offset = 0
    
    while True:
        # Get the current telemetry data
        image_telemetry_data = w.getImageAndTelemetryData(drone_id)
        telemetry_data = bytearray(image_telemetry_data[3110408:]).decode()
        telemetry_elements = telemetry_data.split(':')

        print(f"Telemetry: {telemetry_elements}")
    
        latitude = telemetry_elements[0]
        longitude = telemetry_elements[1]
        current_altitude = telemetry_elements[2]
        heading = telemetry_elements[3]
        gimbal_pitch = telemetry_elements[4]
        gimbal_yaw = telemetry_elements[6]
        waypoint_check = telemetry_elements[14]
        next_waypoint = telemetry_elements[15]

        print(f"Current altitude offset: {altitude_offset} m")

        altitude_offset += altitude_increment

        if altitude_offset > max_altitude or altitude_offset < min_altitude:
            altitude_increment = -altitude_increment
            altitude_offset += altitude_increment

        new_altitude = starting_altitude + altitude_offset
        print(f"Setting altitude to: {new_altitude} m (offset: {altitude_offset} m)")

        # Create waypoint data string
        # Format: lat:lon:alt:heading:speed:waypoint_num:threshold:gimbal_pitch:gimbal_yaw:camera
        new_waypoint = f"{latitude}:{longitude}:{str(new_altitude)}:{heading}:2:{waypoint_num}:4:{gimbal_pitch}:{gimbal_yaw}:90"

        result = w.sendWayPointData(new_waypoint, drone_id)
        
        print(f"Telemetry waypoint check: {waypoint_check}, next waypoint: {next_waypoint}, sent waypoint num: {waypoint_num}")

        waypoint_num += 1

        time.sleep(update_interval)
        print("-"*40)

except KeyboardInterrupt:
    print("Altitude iteration test stopped by user.")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    print("Altitude iteration test finished.")