# Topic Store

[![building](https://lcas.lincoln.ac.uk/buildfarm/job/Mdev__topic_store__ubuntu_bionic_amd64/badge/icon)](https://lcas.lincoln.ac.uk/buildfarm/job/Mdev__topic_store__ubuntu_bionic_amd64/lastBuild/)
![CI](https://github.com/RaymondKirk/topic_store/workflows/Topic%20Store/badge.svg?branch=master)
[![PyPi](http://badge.fury.io/py/topic-store.svg)](https://pypi.org/project/topic-store/)

ROS package used for serialising common ROS messages to a database or filesystem.

# Installation
### ROS 

```bash
# From source
cd catkin_ws/src
git clone https://github.com/RaymondKirk/topic_store 
catkin build topic_store

# From apt
sudo apt install ros-melodic-topic-store  # you need to add the L-CAS ros source
```
### Installing without ROS

You can install topic_store as an independent python2/3 package without a working ROS installation. 

```bash
# From source 
git clone https://github.com/RaymondKirk/topic_store
cd topic_store/src
pip install --extra-index-url https://rospypi.github.io/simple/ -e .

# From PyPi
pip install topic-store
```

To install other dependencies i.e. `ros_numpy` you can run the following

```bash
pip install --extra-index-url https://rospypi.github.io/simple/ ros_numpy sensor_msgs geometry_msgs nav_msgs                                                       130 python3-library!+?
```

# Usage

[![asciicast](https://asciinema.org/a/Cq9i3a41fzuULw52tRLkHvBQS.svg)](https://asciinema.org/a/Cq9i3a41fzuULw52tRLkHvBQS)

Create a scenario file by following the documentation in [scenarios](https://github.com/RaymondKirk/topic_store/wiki/Scenarios) or by following the example file
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

If ```storage.method``` is database ensure that your database is accessible at the host/port in the config file found at
 ```storage.config```. To launch the default database ```roslaunch topic_store start_database.launch```, the default 
database creates/uses a Mongo 4.2 server instance in the default ```${HOME}/.ros/topic_store/database``` 
folder exposed on ```localhost:65530``` (defined in  ```storage.config``` of the scenario file). 
Docker is a requirement to use a database backend to avoid conflicts with system mongo. A utility script is provided in 
`scripts/install_docker.sh` if you do not have it installed.

Launch your data collection scenario! 

```
roslaunch topic_store run_scenario.launch scenario_file:="/path/to/your/scenario/file.yaml"
```

# Examples

## Database data

The below example shows how to load and use files stored in a database. 

```python
import topic_store as ts

# Read data
storage = ts.load("/path/to/scenario/file/containing/db/connection/info.yaml")
for item in storage:
    print("As Python Dict", item.dict)  # or item["key"]
    print("As ROS Msgs", item.msgs)  # or item("key")

# Write data
storage.insert_one({"important_data": "topic store is great!"})
```

## Filesystem data

The below example shows how to load and use `.topic_store` files, saved from when scenarios are ran with the 
`storage_method="filesystem"` option.

```python
import topic_store as ts

# Read data
storage = ts.load("/path/to/file.topic_store")
for item in storage:
    print("As Python Dict", item.dict)  # or item["key"]
    print("As ROS Msgs", item.msgs)  # or item("key")

# Write data
storage.insert_one({"important_data": "topic store is great!"})
```

## Launch a database

When launching a data collection scenario where ```storage.method==database``` you must also launch the database or 
ensure it's already running at the uri defined in the host/port parameters of the config file located at ```storage.config```.

To launch a database prior to running a data collection scenario.

```bash
roslaunch topic_store start_database.launch scenario_file:="/path/to/your/scenario/file.yaml"
```
 
## Convert to ROS bags

Filesystem `.topic_store` files and database collections can be converted to ROS bags.

```bash
# Convert filesystem files to rosbags
rosrun topic_store convert.py -i input.topic_store -o output.bag

# Convert database collections to rosbags 
# Pass scenario file containing connection and collection info as the input
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

## Export from Mongodb query into rosbag/filesystem

Example call:
  
* With typical mongodb URI for SSL and authentication
* On database `ff_rasberry` and collection `2020_riseholme_framos_cameras`
* Specify database connection URI (`-i`) with db name `authSource=database`
* Including a query (`-q`) for a specific document
* Add projection to return only sub-documents (`-p`)

```bash
convert.py -i "mongodb://USER:PASS@HOST:PORT/?authSource=ff_rasberry&tls=true&tlsAllowInvalidCertificates=true" -c 2020_riseholme_framos_cameras -q '{"_id":"ObjectId(5f115ee6af915351df739757)"}' -p '{"cameras.top.color":1, "robot": 1}' -o out.bag
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
- [x] Added support for complex database creations via mongo configs
- [x] Added URI inference from mongo configs to make API simpler
- [x] Support for GridFS or document splitting via list declaration in the scenario files.
- [x] ~~Integration of https://github.com/DreamingRaven/python-ezdb~~
- [x] Added partial support for TLS/Auth in MongoClient (use uri arg or convert.py with -u)
- [ ] Added support for TLS/Auth in MongoClient and infer from mongo configs
