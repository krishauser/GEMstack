import carla

host = 'localhost'
port = 2000
client = carla.Client(host, port)
client.set_timeout(2000.0)  # Setting a timeout of 2000 seconds

# Get information from the recorder log file
log = client.show_recorder_file_info("/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log",show_all=True)

# Path where you want to save the log file
output_file_path = "/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.txt"

# Write the log data to a text file
with open(output_file_path, 'w') as file:
    file.write(log)

print(f"Log data has been saved to {output_file_path}")
