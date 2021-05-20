# Topic Store

![CI](https://github.com/RaymondKirk/topic_store/workflows/Topic%20Store/badge.svg?branch=master)
[![PyPi](http://badge.fury.io/py/topic-store.svg)](https://pypi.org/project/topic-store/)

Topic Store is a ROS package for storing ROS messages to a database or filesystem.

Unlike ROS bags Topic Store adds flexibility by serialising all messages into a data hierarchy that's easily 
searchable with database queries and allows for remote storage. 

You can also use Topic Store as a standalone python package to read and write data without a ROS installation!

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
pip install --extra-index-url https://rospypi.github.io/simple/ topic-store
```

To install other dependencies i.e. `ros_numpy` you can run the following

```bash
pip install --extra-index-url https://rospypi.github.io/simple/ ros_numpy
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

## Storing high bandwidth image data

To store a lot of image data you will be limited to the IO of your machine. 
[Topic Compression](https://github.com/RaymondKirk/topic_compression) offers a compression solution to increase capture performance.

```bash
# Install the ROS package
cd catkin_ws/src
git clone https://github.com/RaymondKirk/topic_compression
catkin build topic_compression

# Compression/Decompression is chosen automatically so just pass the input topic name and optionally the out topic name
rosrun topic_compression run in:=/camera/colour/image_raw  # out=/camera/colour/image_raw/compressed 
rosrun topic_compression run in:=/camera/depth/image_raw  # out=/camera/depth/image_raw/compressed 
```