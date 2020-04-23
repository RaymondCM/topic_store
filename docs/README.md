# Topic Store

ROS package used for serialising common ROS messages to a database or filesystem.

# Usage

Create a scenario file by following the documentation in [scenarios](./scenarios.md) or by following the example file
[default_config.yaml](../scenarios/default_config.yaml). The example below will save a history of all messages sent to the ros log topic.

```yaml
# Save to /path/to/your/scenario/file.yaml. This collection behaviour will save your log history.
context: "save_ros_logs"

storage_method: "filesystem" 

store_topics: { 
  "ros_msg": "/rosout", 
}

collection_method: 
  name: "event" 
  watch_topic: "/rosout"
```

Launch your data collection scenario! 

```
roslaunch topic_store run_scenario.launch scenario_file:="/path/to/your/scenario/file.yaml"
```

Enjoy!

# Examples

## Convert to ROS bags

`.topic_store` files can be converted to ROS bags.

```bash
rosrun topic_store convert.py -i input.topic_store -o output.bag
```

## Filesystem data

The below example shows how to load and use `.topic_store` files, saved from when scenarios are ran with the 
`storage_method="filesystem"` option.

```python
from topic_store import load
from topic_store import MongoDBParser
messages = load("/path/to/file.topic_store")
parser = MongoDBParser()

for item in messages:
    print("As Python Types", item.dict)
    print("As ROS Msgs", item.msgs)
    print("As MongoDB Doc", item.to_dict(parser))  # or parser(item.dict)
```
# Implementation Road Map

- [x] Implement auto-subscribers and auto-data loggers
- [x] Tree based Subscribers and Message history
- [x] Serialisation from genpy.Message to python types
- [x] Parser for type compatibility between genpy.Message<->Python<->BSON<->MongoDB
- [x] Scenario files for describing data collection behaviours
- [x] File system storage method as a ROS bag replacement for better compatibility.
- [x] Add rosbag compatibility (via convert.py)
- [ ] Add covert to database compatibility (via convert.py)
- [ ] Add database storage method to scenarios (need to determine best way to define DB connection)
- [ ] Integration of https://github.com/DreamingRaven/python-ezdb