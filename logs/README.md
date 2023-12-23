Default logging directory.

Logs are placed within a timestamped folder.

Within each folder, the file structure is:

- meta.yaml: metadata for the run, including events, termination result, git branch and commit ID.
- settings.yaml: the entire settings dictionary for the run.
- vehicle.bag: rosbag file for vehicle and sensor messages.
- vehicle.json: logged readings and commands on the vehicle object, if logging was enabled.
- behavior.json: logged messages within the behavior stack.  An endline-separated sequence of JSON objects. 
- state.json: logged AllState values.  An endline-separated sequence of JSON objects.
