This is the default storage folder for topics ran with run_scenario.launch/run_scenario.py. 

##### Default Folder Structure

    .
    ├── topic_store                 # Package directory
    │   ├── stored_topics           # Default storage location for "filesystem" based scenarios
    │   │   ├── filesystem          # Filesystem based scenarios are split into context/date structures
    │   │   │   ├── context_name    # ...
    │   │   │   ├── ...             # Other saved contexts
    │   │   ├── database            # Default database path used to with start_database.launch
    │   ├── ...                 # Other package folders src/script/msg/launch/action etc.
    │   └── docs                # Consult when in trouble
