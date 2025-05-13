# Component Replay and Rostopic Replay

## Component Replay
Component replay can now play back states from behavior.json. Only the component outputs can be played back. (The vehicle_interface commands cannot be played back). Note that the current simulator maps need to be updated to handle real GEM data

## Rostopic Replay

Will create new publishers from the specified topics in the vehicle.bag. The messages will be published with the GEM interface's time, to keep sync with the rest of the execution. This means that the rest of the GEMstack execution must also be running (will not publish on detector_only variant)



## Usage

Add topics and components to replay in the yaml file:

```yaml
replay:  # Add items here to set certain topics / inputs to be replayed from logs
    # Specify which log folder to replay from
    log: 
    ros_topics : []
    components : []
```


