# Scenarios

Scenario files are stored in the root [scenarios](../scenarios) folder. 
These files are used to create a plan/describe a behaviour for data collection.
Each file is comprised of four sections, each detailing a separate component of the data collection process.
Once these sections are defined the [run_scenario.launch](../launch/run_scenario.launch) can be used to execute the plan.

Each section is documented below and complete file example is presented at the bottom.

Please refer to [default_config.yaml](../scenarios/default_config.yaml) for an implementation example.

## 1) Context

The `context` field is a string of any length.
This parameter can be thought of as a unique identifier to describe the type/location/time of your data collection plan.
 
```yaml
context: "context-here"
```

When storage method is filesystem, all collected data is placed under a with the same name as context. 
When storage method is database the data is stored in a collection of name context.

## 2) Storage

The `storage.method` field can be one of two possible strings `"database"` or `"filesystem"`.
This parameter tells the data logger where to store the data. 

If `"filesystem"` is set, data will be stored in the location defined by the `storage.location` parameter.
This mode is useful for offline use, where a database isn't available. 

A utility provided in [convert.py](../scripts/convert.py) to migrate the data to a database later on.

```yaml
storage: 
    method: "filesystem"
    location: "default"
```

If `"database"` is set, data will be stored in the database under a collection named context. 
The database connection config must be specified so, a URI can be inferred from it the default is 
[default_db_config.yaml](../config/default_db_config.yaml).
 
```yaml
storage: 
    method: "database"
    config: "default" 
```

## 3) Data

The `"data"` field is a n level dictionary. The values of the keys in this dictionary will be stored 
in the exported data. If the value is a string that is also currently a published topic, then the value will be populated with
the respective ros topic information.
In the example below the `"/clock"` topic information is stored in a `ros_clock_topic` container.

```yaml
data: {
    int_example: 5,
    float_example: 4,
    str_example: "ID: example",
    list_example: ["hello", "world"],
    ros_clock_topic: "/clock",
}
```

A more complicated example below will group localisation data from multiple robot topics in a single `localisation` container.

```yaml
data: { # Store the ros topic information in this key value container structure
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

## 4) Collection

This is the most complex parameter since it describes the data collection behaviour. The `"collection"` field is 
a dictionary containing some required and optional parameters depending upon which parameter is set for `name`. 

The `method` sub-field defines the exploration behaviour. 
There are five possible options described below: 

 - `action_server`: Will wait until [actionlib](http://wiki.ros.org/actionlib/Tutorials) requests are received on 
 `collection.action_server_name`. To test you can run to get a graphical request system `rosrun actionlib axclient.py '''/collect_data'''`. 
```yaml
collection:
  method: "action_server"
  action_server_name: "collect_data"
```
 - `action_server_video`: Will wait until [actionlib](http://wiki.ros.org/actionlib/Tutorials) requests are received on 
 `collection.action_server_name` with a request to start/stop data collection. In the action server request you
  can specify `start|stop|true|false|t|f` to start or stop capturing data every time `wait_topic` publishes a new message. 
  The `wait_topic` field is string that must be a valid rostopic. To test you can run to get a graphical request system
   `rosrun actionlib axclient.py '''/collect_data'''`. 
```yaml
collection:
  method: "action_server_video"
  action_server_name: "collect_data"
  watch_topic: "/realsense_camera/color/image_raw"
```
 - `timer`: Captures data every `timer_delay` seconds. The `timer_delay` field is integer. 
 Ideal for when the robot is in continuous operation in unmapped environments.
```yaml
collection:
    method: "timer"
    timer_delay: 3
```
 - `event`: Captures data every time `wait_topic` publishes a new message. The `wait_topic` field is string that must 
 be a valid rostopic. 
 Ideal for capturing video sequences (of rgb frames for example)
```yaml
collection:
    method: "event" 
    watch_topic: "/realsense_camera/color/image_raw"
```

## Example file 

The below yaml file is a complete example which will capture camera and robot odometry data when a request is received.

```yaml
context: "documentation_example"

storage: 
    method: "filesystem"
    location: "default"

data: {
    localisation: {
      laser_scan: "/scan", current_edge: "/current_edge", current_node: "/current_node",
      closest_node: "/closest_node", amcl: "/amcl_pose", robot_pose: "/robot_pose"
    },
    rgb: {
      image: "/doc_cam/color/image_raw", intrinsics: "/doc_cam/color/camera_info"
    },
}

collection:
    method: "action_server"
    action_server_name: "collect_data"
```