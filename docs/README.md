# Topic Store

ROS package used for serialising common ROS messages to a database or filesystem.

# Usage

Create a scenario file by following the documentation in [scenarios](./scenarios.md) or by following the example file
[default_config.yaml](../scenarios/default_config.yaml). The example below will save a history of all messages sent to the ros log topic.

```yaml
# Save to /path/to/your/scenario/file.yaml. This collection behaviour will save your log history.
context: "save_ros_logs"

storage: 
  "method": "filesystem"
  "location": "default" 

data: { 
  "ros_msg": "/rosout", 
}

collection: 
  method: "event" 
  watch_topic: "/rosout"
```

If ```storage.method``` is database ensure that your database is accessible at ```storage.uri```. 
Only URI based connections are currently supported.

Launch your data collection scenario! 

```
roslaunch topic_store run_scenario.launch scenario_file:="/path/to/your/scenario/file.yaml"
```

# Examples

## Database data

The below example shows how to load and use files stored in a database. You can see that the interface for working with 
databases and the filesystem are the same. All stored documents in the database are easily de-serialised for use.

```python
from topic_store import MongoClient

client = MongoClient()

collection = "your_collection"
stored_topics = client.find(collection)  # Return all the documents in this collection

for item in stored_topics:
    print("As ROS msg", item.msgs)
    print("As python dict", item.dict)
```

## Filesystem data

The below example shows how to load and use `.topic_store` files, saved from when scenarios are ran with the 
`storage_method="filesystem"` option.

```python
from topic_store import load

stored_topics = load("/path/to/file.topic_store")

for item in stored_topics:
    print("As Python Types", item.dict)
    print("As ROS Msgs", item.msgs)
```

## Convert Filesystem to ROS bags

`.topic_store` files can be converted to ROS bags.

```bash
rosrun topic_store convert.py -i input.topic_store -o output.bag
```

## Convert Filesystem to Database

`.topic_store` files can be migrated to a MongoDB database. Pass the scenario file that contains the database connection information as the output file.

```bash
rosrun topic_store convert.py -i input.topic_store -o scenario_config.yaml
```


# Implementation Road Map

- [x] Implement auto-subscribers and auto-data loggers
- [x] Tree based Subscribers and Message history
- [x] Serialisation from genpy.Message to python types
- [x] Parser for type compatibility between genpy.Message<->Python<->BSON<->MongoDB
- [x] Scenario files for describing data collection behaviours
- [x] File system storage method as a ROS bag replacement for better compatibility.
- [x] Add rosbag compatibility (via convert.py)
- [x] Add database storage method to scenarios
- [x] Add covert to database compatibility (via convert.py)
- [ ] Add credentials to database connection method
- [ ] Integration of https://github.com/DreamingRaven/python-ezdb