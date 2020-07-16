^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topic_store
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#6 <https://github.com/RaymondKirk/topic_store/issues/6>`_ from RaymondKirk/fix_for_msg_depth_bug
  GridFS improvements and fix for nested permutations of (dict -> list -> object) such as TF messages
* Updated README to reflect recent changes
* Store original document and session ID in file meta of GridFS documents (for cleanup)
* Fix for incompatible parsing of a dict -> list -> dict -> etc type nests
  Search depth first (takes much longer though) but TF and other messages now supported
* Fixes to conversion script (to rosbag from mongo/topic storage)
* Attempt to fix nested dict recursion to fix ROS messages
  Still having issues with TF messages but most others work. Problem is nested dicts of lists of dicts aren't parsed correctly with the _ros_meta field.
* Contributors: Raymond Tunstill (Kirk), RaymondKirk

0.0.9 (2020-07-15)
------------------
* Removed GridFS package.xml entry (included in pymongo)
* Contributors: RaymondKirk

0.0.8 (2020-07-15)
------------------
* Added tests for >16MB document
* Merge pull request `#5 <https://github.com/RaymondKirk/topic_store/issues/5>`_ from RaymondKirk/gridfs
  Added GridFS support with zero API change `#4 <https://github.com/RaymondKirk/topic_store/issues/4>`_
* Somewhat support deleting documents and gridfs associated objects
* Len is now removed from TopicStorage
* Initial commit of GridFS support (all bson.Binary types are stored in chunks)
  Transparent dicts, so easily searchable by meta data.
  The objects are automatically stored/retrieved from GridFS with no API changes.
* Add GridFS dependency
* Remove len from TopicStorage objects
* By default don't print size due to heavy recursion of nested dicts
  Use print(topic_store.__repr_\_(print_size=True)) to print sizes
* Contributors: Raymond Tunstill (Kirk), RaymondKirk

0.0.7 (2020-07-10)
------------------
* Added verbose print to TopicStore objects (ROSType and Document Size)
* Force subscriber when data storage key starts with '/'
  Otherwise if topics don't exist when data collection starts then the topics will never be subscribed to
* Contributors: RaymondKirk

0.0.6 (2020-07-03)
------------------
* Updated requirements.txt
* Remove README.md typo
* Ensure rospy is initialised in convert.py script
* Contributors: RaymondKirk

0.0.5 (2020-05-16)
------------------
* Reduced required TQDM version
* Contributors: RaymondKirk

0.0.4 (2020-05-15)
------------------
* Enable start_database.launch to use system mongo for CI and tests
* Contributors: RaymondKirk

0.0.3 (2020-05-15)
------------------
* Added docker instructions and install script
* Contributors: RaymondKirk

0.0.2 (2020-05-15)
------------------
* Added package is install CMake args
* Fix type coercion tests so keys must be strings
* Fixed support for convert and ensure all document keys are strings
* Added testing suite for pytest and fixed DB config paths to resolve locally not in the docker container
* Uploaded usage example
* No messages in the repo anymore removed add_message line
* Point badge to last build
* Added building badge
* Added build dependency
* Added python package dependencies from deb and fixes for L-CAS rosdistro
* Remove instead of escaping bash symbols in STD_OUT parser
* Removed old formatting lines from the old parser
* Moved yaml parser to topic_store package and added file checks
* Unified start_database interface to go from scenario files rather than mongo configs directly
* Swap strategy to just escape special characters
  Trust that users won't abuse the unified api, change later to parse everything before
* Updated parser so spaces are supported in variables
* Fixed action server and filesystem location
* Merge pull request `#1 <https://github.com/RaymondKirk/topic_store/issues/1>`_ from RaymondKirk/database_config_over_uri
  [WIP] Support mongodb configs for databases with Auth/TLS/Non-Local
* Added URI overload for usability (to support username/password)
* Implemented mongodb configs to infer URI and setup more complex databases
* Added 16MB doc limit TODO to README.md
* Added session based DB to ROS bag support
* Fix for cursor returning all objects view and added session based search
* Added default stabilise_time to run_scenario.launch
* Removed redundant recursive function in place of generator
* Save python dict not serialised class object and deprecate getitem api
* Completed TODOs and updated SubscriberTree doc
* Added topic name lookup rather than starts with '/' to check if topic or string
* Added session property and fixed ros_time
* Unification of API and added load overload for MongoStorage connections
* Updated docs for iterators
* Added TopicStore typed API and session IDs
* Clear up documentation
* Defined storage API for unification
* Removed pointless assert
* Changed the tests files to support pytest
  Run pytest tests/ -v from project root
* Use of all parsers is now implicit the type coercion is now automatically handled
* Force type conversion for all TopicStore objects and serialisation version
* Added warning for conversion from ROS bag to fs/db
* Added warning for conversion from ROS bag to fs/db
* Added ability to convert between filesystem<->database and convert either to ROS bags using unified API
* Added reverse parser to go from MongoDB types (i.e unicode->str) to python types
* Added reverse mongodb parser for python 2.7 support
* Database now fully supported as a storage method and API is unified
* Major API improvements for mongo db interface
* Added default MongoDB server (start_database.launch) to safely bring up a dockerised mongo db server instance.
  Will not conflict with any current system requirements or legacy MongoDB versions.
* Added conversion from .topic_store files to mongodb databases
* Added ROSBag conversions for new interface
* Updated README.md to reflect repo changes
* Added MongoDB loading usage to README.md
* Added basic MongoDB compatibility
* Added support for genpy.Time and genpy.Duration
* Major scenario file upgrades ready for database support
* By default assign BSON.ObjectIDs to all TopicStore items
* Added conversion utility to ROSBags
* Added examples
* Implemented single storage container for filesystem for future ROS bag support and easier loading
* Added type coercion tests
* Added float epoch time functions
* Cleaner type cohesion API
  parser = DefaultTypeParser()
  parse_this = [{"0": 0}, {"1": 1}]
  parsed = parser(parse_this)
* Updated README.md to better document launch
* Added roadmap
* Added some scenario documentation
* Implemented ActionLib interface for collecting data
  Test with `rosrun actionlib axclient.py '''/collect_data'''`
* Added .gitignore
* Implemented initial version of scenario parser and runner
  Will now be based on service, timer or event strategies. Other implementations such as Thorvald going to way points will be high level control done using an action server.
* Added tests for serialisation API
* Initial commit of topic storage package based on RaymondKirk LCAS/rasberry_data_collection
* Contributors: Raymond Tunstill (Kirk), RaymondKirk
