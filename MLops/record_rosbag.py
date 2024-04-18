import subprocess
import signal
import os
import sys

if len(sys.argv) != 4:
    print("Usage: python record_rosbag.py <topics_filename> <rosbag_filename> <keep_rosbag_file>")
    exit(1)

topics_filename = sys.argv[1]+".txt"
rosbag_filename = sys.argv[2]+".bag"
keep_rosbag = sys.argv[3]

if not os.path.exists(topics_filename):
    print(f"Topics file {topics_filename} does not exist")
    exit(1)

with open(topics_filename) as f:
    topics = [line.strip() for line in f.readlines()]

if not topics:
    print(f"No topics found in {topics_filename}")
    exit(1)

topics = " ".join(topics)

command = f"rosbag record {topics} -O {rosbag_filename}"
# command = "ping 127.0.0.1 -t"

process = subprocess.Popen(command, shell=True)

def stop_recording(sig, frame):
    print("Stopping rosbag recording")
    process.terminate()
    process.wait()
    print("Recording stopped")
    # send rosbag file through api
    # Add to here
    if keep_rosbag == "no":
        print("Removing rosbag file")
        subprocess.run(f"rm {rosbag_filename}", shell=True)
    exit(0)

signal.signal(signal.SIGINT, stop_recording)

print("Recording rosbag. Press Ctrl+C to stop recording")

process.wait()