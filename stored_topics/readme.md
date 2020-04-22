This is the default storage folder for topics ran with run_scenario[.launch|.py]. 

##### Default Folder Structure

    .
    ├── topic_store             # Package directory
    │   ├── stored_topics       # Default storage location for "filesystem" based scenarios
    │   │   ├── context_name    # Filesystem based scenarios are split into context/date structures
    │   │   ├── ...             # Other saved contexts
    │   ├── ...                 # Other package folders src/script/msg/launch/action etc.
    │   └── docs                # Consult when in trouble
