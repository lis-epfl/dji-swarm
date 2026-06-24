import time
import ds_wrapper as w

print("Starting altitude test...")

drone_id = 1
altitude_increment = 8  # meters to increase altitude
wait_time = 20  # seconds to wait after sending command
check_interval = 2  # seconds
waypoint_num = 1

try:
    # Get the current telemetry data
    image_telemetry_data = w.getImageAndTelemetryData(drone_id)
    telemetry_data = bytearray(image_telemetry_data[3110408:]).decode()
    telemetry_elements = telemetry_data.split(':')

    print(f"Telemetry: {telemetry_elements}")

    latitude = telemetry_elements[0]
    longitude = telemetry_elements[1]
    current_altitude = float(telemetry_elements[2])
    heading = telemetry_elements[3]
    gimbal_pitch = telemetry_elements[4]
    gimbal_yaw = telemetry_elements[6]

    print(f"\nCurrent Position:")
    print(f"  Latitude: {latitude}")
    print(f"  Longitude: {longitude}")
    print(f"  Altitude: {current_altitude} m")
    print(f"  Heading: {heading}")

    # Calculate new altitude
    new_altitude = current_altitude + altitude_increment
    print(f"\nNew altitude: {new_altitude} m (increased by {altitude_increment} m)")

    new_gimbal_pitch = 15

    # Create waypoint data string
    # Format: lat:lon:alt:heading:speed:waypoint_num:threshold:gimbal_pitch:gimbal_yaw:camera
    waypoint_data = f"{latitude}:{longitude}:{str(new_altitude)}:{heading}:2:{waypoint_num}:4:{str(new_gimbal_pitch)}:{gimbal_yaw}:90"

    print(f"\nSending waypoint data...")
    result = w.sendWayPointData(waypoint_data, drone_id)
    print(f"Waypoint data sent result: {result}")
    
    waypoint_check = telemetry_elements[14]
    next_waypoint = telemetry_elements[15]
    print(f"Telemetry waypoint check: {waypoint_check}, next waypoint: {next_waypoint}, sent waypoint num: {waypoint_num}")

    waypoint_num += 1

    print(f"\nWaiting {wait_time} seconds and monitoring telemetry...")
    
    # Monitor telemetry during wait
    elapsed_time = 0
    
    while elapsed_time < wait_time:
        time.sleep(check_interval)
        elapsed_time += check_interval
        
        # Get updated telemetry
        image_telemetry_data = w.getImageAndTelemetryData(drone_id)
        telemetry_data = bytearray(image_telemetry_data[3110408:]).decode()
        telemetry_elements = telemetry_data.split(':')
        
        # print telemetry waypoint check
        waypoint_check = telemetry_elements[14]
        next_waypoint = telemetry_elements[15]
        print(f"Telemetry waypoint check: {waypoint_check}, next waypoint: {next_waypoint}, sent waypoint num: {waypoint_num}")
        
        # Print current altitude
        current_alt = telemetry_elements[2]
        print(f"[{elapsed_time}s] Current altitude: {current_alt} m")
    
    print("\nAltitude test completed.")

except KeyboardInterrupt:
    print("\nAltitude test stopped by user.")
except Exception as e:
    print(f"\nAn error occurred: {e}")
finally:
    print("Altitude test finished.")