import unittest
import subprocess
import os
import json
from parameterized import parameterized
import time
import signal
from abc import ABC, abstractmethod
# Configuration of each test case. 
TEST_CONFIGS = [
    ("test1", "integration_tests/launch/test1.yaml", 10),
    ("test2", "integration_tests/launch/test2.yaml", 10),
]

import os

def get_last_modified_folder(parent_path):
    subdirs = [entry.path for entry in os.scandir(parent_path) if entry.is_dir()]
    if not subdirs:
        return None 
    last_modified_folder = max(subdirs, key=os.path.getmtime)
    return last_modified_folder


class BaseLogValidator(ABC):

    @abstractmethod
    def validate(self, log_dir):
        """Each test case must implement this method"""
        pass

class ValidateTestCase1(BaseLogValidator):
    def validate(self, log_dir):
        log_file = os.path.join(log_dir, "behavior.json")
        with open(log_file, "r") as file:
            for index, line in enumerate(file):
                try:
                    record = json.loads(line)
                    acceleration = record.get("vehicle", {}).get("data", {}).get("acceleration", 0)
                    assert abs(acceleration) <= .11, f"Record {index} has acceleration {acceleration} > 1"
                except json.JSONDecodeError:
                    raise AssertionError(f"Malformed JSON in log file at line {index}")

class ValidateTestCase2(BaseLogValidator):
    def validate(self, log_dir):
        log_file = os.path.join(log_dir, "behavior.json")


class IntegrationTestSuite(unittest.TestCase):
    VALIDATORS = {
        "test1": ValidateTestCase1(),
        "test2": ValidateTestCase2(),
    }
    def setUp(self):
        pass
    def tearDown(self):
        """Clean up after each test if necessary."""
        pass 

    @parameterized.expand(TEST_CONFIGS)
    def test_command_execution(self, name, config_path, runtime):
        process = subprocess.Popen(["python3", "main.py", "--variant=sim", config_path], text=True,  stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # for line in iter(process.stdout.readline, ''):
        #     print(line, end='') 
        time.sleep(runtime) 
        print("Stopping GEMStack...")
        os.kill(process.pid, signal.SIGINT) 
        process.wait()

        # Validate logs
        validator = self.VALIDATORS.get(name)
        if validator is None:
            self.fail(f"no validator found for {name}")
        validator.validate(get_last_modified_folder('logs'))

        

if __name__ == "__main__":
    unittest.main()
