# Topic Store

ROS package used for serialising common ROS messages to a database or filesystem.

# Usage

1. Create a scenario file by following the documentation in [scenarios](./scenarios.md) or by following the example file
[default_config.yaml](../default_config.yaml).
2. Launch your data collection scenario `roslaunch topic_store run_scenario.launch scenario_file:="path/to/your/file.yaml"`.
3. Enjoy!

# Roadmap

- [x] Implement auto-subscribers and auto-data loggers
- [x] Serialisation from genpy.Message to python types
- [x] Parser for type compatibility between genpy.Message<->Python<->BSON<->MongoDB
- [x] Scenario files for describing data collection behaviours
- [x] File system storage method as a ROS bag replacement for better compatibility.
- [ ] Add rosbag compatibility
- [ ] Add database storage method to scenarios (need to determine best way to define DB connection)
- [ ] Integration of https://github.com/DreamingRaven/python-ezdb