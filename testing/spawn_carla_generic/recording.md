manual_control_with_rec.py
line 1261 for start recording and line 1310 for stop recording
client.start_recorder("/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log", True)
The first argument is the file and the second True means full recording. More info: https://carla.readthedocs.io/en/latest/adv_recorder/

read_record.py is the sample script to read the logged recording file. The next steps are to check and read from the log, and have custom start and ending points.