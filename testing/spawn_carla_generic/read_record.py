import carla

import match

host = 'localhost'
port = 2000
client = carla.Client(host, port)
client.set_timeout(2000.0)  # Setting a timeout of 2000 seconds
filename = "/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log"

log_simple = client.show_recorder_file_info(filename, show_all=False)
results = match.do_match(log_simple)
vehicle_id = results[0][0]

# Get information from the recorder log file
log_full = client.show_recorder_file_info(filename, show_all=True)
results = match.do_match_full(log_full, vehicle_id)

# Path where you want to save the log file
output_file_path = filename.replace('.log', '.txt')
output_file_full_path = filename.replace('.log', '_full.txt')

weather_time_file_path = filename.replace('recording.log', 'weather_time.txt')
# read weather and time data
weather_time_data = ""
with open(weather_time_file_path, 'r') as file:
    weather_time_data = file.read()

# Write the log data to a text file
with open(output_file_path, 'w') as file:
    file.write(log_simple)

with open(output_file_full_path, 'w') as file:
    file.write(log_full)

output_file_final_path = filename.replace('.log', '_final.txt')
with open(output_file_final_path, 'w') as file:
    file.write(log_simple.split('\n')[1])
    file.write("\n")
    file.write(weather_time_data)
    file.write("\n")
    for result in results:
        file.write(result)
        file.write("\n")
