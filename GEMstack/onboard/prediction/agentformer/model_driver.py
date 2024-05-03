import subprocess
import sys
import time

def start_model_process(model_path, env_path):
    # python_path = '/root/anaconda3/envs/AgentFormer/bin/python'
    python_path = '/home/tyler/anaconda3/envs/AgentFormer/bin/python'
    return subprocess.Popen(
        [python_path, model_path], #["conda", "run", "-n", env_path, "python", model_path],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        encoding="utf-8",
    )
def run_model(process):
    process.stdin.write("GET PREDICTIONS" + "\n")
    process.stdin.flush()

    while True:
        start_up_message = model1_process.stdout.readline().strip()
        print(start_up_message)
        if start_up_message == "READY":
            break
    return "BRUH"



if __name__ == "__main__":
    model1_path = "agentformer_process.py"
    model1_env = 'AgentFormer'
    # Start model1 process
    model1_process = start_model_process(model1_path, model1_env)
    print("Model 1 process: ", model1_process)
    while(True):
        input(f"new data is ready")
        # Communicate with model1
        output_data1 = run_model(model1_process)
        print("Model1 output:", output_data1)

    # Close the processes
    model1_process.stdin.close()
    model1_process.wait()