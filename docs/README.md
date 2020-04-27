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
Only URI based connections are currently supported. To launch the default database 
```roslaunch topic_store start_database.launch```, the default database creates/uses a Mongo 4.2 server instance in the 
default ```$(find topic_store)/stored_topics/database``` folder exposed on ```localhost:65530```. Docker is used to bring up the server instance to avoid version 
conflicts with system mongo.

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

your_context = "your_collection"
client = MongoClient(collection=your_context)

stored_topics = client.find()  # Return all the documents in this collection

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

## Convert to ROS bags

Filesystem `.topic_store` files can be converted to ROS bags.

```bash
rosrun topic_store convert.py -i input.topic_store -o output.bag
```

Database collections can be also be converted to ROS bags. Pass the scenario file that contains the database 
connection/collection information as the input file.

```bash
rosrun topic_store convert.py -i scenario_config.yaml -o output.bag
```


## Convert between Filesystem and Database

Filesystem `.topic_store` files can be migrated to a MongoDB database. Pass the scenario file that contains the database 
connection information as the output file.

```bash
rosrun topic_store convert.py -i input.topic_store -o scenario_config.yaml
```

Database collections can be converted to a filesystem `.topic_store` file. Pass the scenario file that contains the database 
connection information as the input file.

```bash
rosrun topic_store convert.py -i scenario_config.yaml -o output.topic_store
```

# Implementation Road Map

- [x] Implement auto-subscribers and auto-data loggers
- [x] Tree based Subscribers and Message history
- [x] Serialisation from genpy.Message to python types
- [x] Parser for type compatibility between genpy.Message<->Python<->BSON<->MongoDB
- [x] Scenario files for describing data collection behaviours
- [x] File system storage method as a ROS bag replacement for better compatibility.
- [x] Added database storage method to scenarios
- [x] Added convert to database from filesystem compatibility (via convert.py)
- [x] Added convert to filesystem from database compatibility (via convert.py)
- [x] Added convert to ROS bag from filesystem compatibility (via convert.py)
- [x] Added convert to ROS bag from database compatibility (via convert.py)
- [x] Added credentials to database connection method via URI
- [ ] Integration of https://github.com/DreamingRaven/python-ezdb