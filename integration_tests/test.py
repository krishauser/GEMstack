import subprocess
import time
import subprocess
import os
import yaml
import time
import signal

config_path = "launch/fixed_route.yaml"
process = subprocess.Popen(["python3", "main.py", "--variant=sim", config_path], text=True,  stdout=subprocess.PIPE, stderr=subprocess.PIPE)
# for line in iter(process.stdout.readline, ''):
#     print(line, end='')
# for line in iter(process.stderr.readline, ''):
#     print(line, end='')
# process.wait()
time.sleep(30)  # Give it some time to start

print("Sending SIGINT to process...")
os.kill(process.pid, signal.SIGINT)  # Equivalent to Ctrl+C