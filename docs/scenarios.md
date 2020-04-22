# Scenarios

Scenario files are stored in the root [scenarios](../scenarios) folder. 
These files are used to create a plan/describe a behaviour for data collection.
Each file is comprised of four sections, each detailing a separate component of the data collection process.
Once these sections are defined the [run_scenario.launch](../launch/run_scenario.launch) can be used to execute the plan.

Each section is documented below and complete file example is presented at the bottom.

## 1) Context

The `context` field is a string of any length.
This parameter can be thought of as a unique identifier to describe the type/location/time of your data collection plan.
 
```yaml
context: "context-here"
```

When storage method is filesystem, all collected data is placed under a with the same name as context. When storage method

## 2) Storage Method

The `storage_method` field can be one of two possible strings `"database"` or `"filesystem"`.
This parameter tells the data logger where to store the data. 

If `"filesystem` is set, data will be stored in the 
location defined by the `save_location` parameter in [run_scenario.launch](../launch/run_scenario.launch), the default storage structure is documented [here](../stored_topics/readme.md) . This mode is
useful for offline use, where a database isn't available. 
A utility is provided in [migrate_data.py](../src/topic_store/migrate_data.py) to migrate the data to a database later on.

```yaml
storage_method: "filesystem"
```

If `"database"` is set, data will be stored in the database. 
Currently this is restricted to a local database until database credentials are parameters of the scenario file. To use this storage method the database must be accessible or ran i.e. `roslaunch topic_store start_database.launch db_path:=/some_folder/db`.
 
```yaml
storage_method: "database"
```

## 3) Store Topics

The `"store_topics"` field is a n level dictionary. The values of the keys in this dictionary will be stored 
in the exported data. If the value is a string that starts with the character `/`, then the value will be populated with
the respective ros topic information.
In the example below the `"/clock"` topic information is stored in a `ros_clock_topic` container.

```yaml
store_topics: {
  int_example: 5,
  float_example: 4,
  str_example: "ID: example",
  list_example: ["hello", "world"],
  ros_clock_topic: "/clock",
}
```

A more complicated example below will group localisation data from multiple robot topics in a single `localisation` container.

```yaml
store_topics: { # Store the ros topic information in this key value container structure
  localisation: {
    laser_scan: "/scan",
    current_edge: "/current_edge",
    current_node: "/current_node",
    closest_node: "/closest_node",
    amcl: "/amcl_pose",
    robot_pose: "/robot_pose",
  },
}
```

## 4) Collection Method

This is the most complex parameter since it describes the data collection behaviour. The `"collection_method"` field is 
a dictionary containing some required and optional parameters depending upon which parameter is set for `name`. 

The `name` sub-field defines the exploration behaviour. 
There are five possible options described below: 

 - `service`: Will wait until [actionlib](http://wiki.ros.org/actionlib/Tutorials) requests are received on 
 `{name}collect_data` where name is the `runner_name` parameter in `run_scenario.[launch|py]`. 
 To test you can run to get a graphical request system `rosrun actionlib axclient.py '''/collect_data'''`. 
 - `timer`: Captures data every `capture_delay` seconds. The `capture_delay` field is integer. 
 Ideal for when the robot is in continuous operation in unmapped environments.
 ```yaml
collection_method:    # These parameters describe the robot data capture behaviour
  name: "timer"       # timer will wait n seconds between captures 
                        #   - Requires 'capture_delay' field (number of seconds between captures)
  capture_delay: 3  # The delay between timer captures
```
 - `event`: Captures data every time `wait_topic` publishes a new message. The `wait_topic` field is string that must 
 be a valid rostopic. 
 Ideal for capturing video sequences (of rgb frames for example)
 ```yaml
collection_method:  # These parameters describe the robot data capture behaviour
  name: "event"      # event will until the 'watch_topic' topic publishes a new message (useful for filming)
                      #   - Requires 'watch_topic' field (topic name to wait for an update on)
  watch_topic: "/realsense_camera/color/image_raw"
```

## Example file 

The below yaml file is a complete example which will capture camera and robot odometry data when a request is received.

```yaml
context: "documentation_example"

storage_method: "filesystem"

store_topics: {
  localisation: {
    laser_scan: "/scan", current_edge: "/current_edge", current_node: "/current_node",
    closest_node: "/closest_node", amcl: "/amcl_pose", robot_pose: "/robot_pose"
  },
  rgb: {
    image: "/doc_cam/color/image_raw", intrinsics: "/doc_cam/color/camera_info"
  },
}

collection_method:
  name: "service"
```