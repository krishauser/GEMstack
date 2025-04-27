# End-to-End Testing

## Writing Tests

Under the GEMStack/GEMStack/integration_tests directory, there are two example test cases (log folders still need to be specified)


Each test case is associated with a test config, which is written in the order:

>Name of test, variant of sim to run, launch file location, runtime of test

The test must also be written with a custom validation function that inherits from the base validator and mapped to the name in the test suite:

```python
VALIDATORS = {
        "test1": ValidateTestCase1(),
        "test2": ValidateTestCase2(),
}
```
 
 Each test will run in sequential order and start GEMStack in a separate process over the course of the specified runtime.
 
 The custom validation funtion will then run on the latest log folder generated (This part needs to be made more robust later).
 
 The validation function is open ended so there is flexibility to parse the behavior_json file, rosbag, or stdout/stderr files.
 
  All results and the runtime will be displayed in the terminal at the end.
  
 ## Running the test suite:
 
 python3 integration\_tests/run\_tests.py
 


