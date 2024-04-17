import carla

host = 'localhost'
port = 2000
client = carla.Client(host, port)
client.set_timeout(2000.0)  # Setting a timeout of 2000 seconds
filename = "/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log"

# Get information from the recorder log file
log = client.show_recorder_file_info(filename,
                                     show_all=False)

# Path where you want to save the log file
output_file_path = filename.replace('.log', '.txt')
weather_time_file_path = filename.replace('recording.log', 'weather_time.txt')
# read weather and time data
weather_time_data = ""
with open(weather_time_file_path, 'r') as file:
    weather_time_data = file.read()

# Write the log data to a text file
with open(output_file_path, 'w') as file:
    file.write(weather_time_data)
    file.write("\n")
    file.write(log)
client.start_recorder("/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log", True)
print(f"Log data has been saved to {output_file_path}")
