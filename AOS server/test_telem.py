import ds_wrapper


data = ds_wrapper.getImageAndTelemetryData(1)
telemetry = data[3110408:].decode('utf-8', errors='ignore').strip('\x00')
print(telemetry)
