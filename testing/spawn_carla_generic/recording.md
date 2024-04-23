manual_control_with_rec.py
line 1261 for start recording and line 1310 for stop recording
client.start_recorder("/home/hb-station1/Documents/GEMstack/testing/spawn_carla_generic/recording.log", True)
The first argument is the file and the second True means full recording. More info: https://carla.readthedocs.io/en/latest/adv_recorder/

read_record.py is the sample script to read the logged recording file. The next steps are to check and read from the log, and have custom start and ending points.

recording_final is as follows. when it is stopped, we keep only the location after movement and give the time stopped at that place.

Map: Town10HD_Opt
weather_time: 0
Frame 2 at 0.0162233 seconds. Stopped time: 1.2001066999999999s
  Id: 191 Location: (5554.23, 13046, 0.0272141) Rotation: (0, 0, -179.68)
Frame 80 at 1.21633 seconds
  Id: 191 Location: (5554.23, 13046, 0.0236549) Rotation: (0, 0, -179.68)