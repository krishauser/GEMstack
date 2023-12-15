Default logging directory.

Logs are placed within a timestamped folder.

Within each folder, the file structure is:

- meta.yaml: metadata for the run, including events, termination result, git branch and commit ID.
- settings.yaml: the entire settings dictionary for the run.
- vehicle.bag: rosbag file for vehicle messages
- behavior.json: logged messages within the behavior stack.
