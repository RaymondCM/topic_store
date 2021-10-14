^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topic_store
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add much faster method of session filtering
* Update default requirements
* Bump version
* Support for backwards bytes in py2/3
* basestring in py3 compat patch
* Hacky fix to ensure ROS builds contain the meta
* Enable uploads
* Add dist and wheel packaging and util script to release on PYPI
* Version increment for PyPi dist
* Ensure package data is respected in python lib
* Merge pull request `#22 <https://github.com/RaymondKirk/topic_store/issues/22>`_ from RaymondKirk/workshop
  Workshop improvements 2021
* Remove redundant info
* Bump version
* Fix nummpy types
* Remove import compression
* Updated README.md info
* Removed compression in favour of https://github.com/RaymondKirk/topic_compression
* Docker files update
* Docker files update
* Docker files update
* Removed GUI
* Fixed project meta data
* Updated README
* Merge branch 'master' of https://github.com/RaymondKirk/topic_store into workshop
* Updated README links
* Updated README and added error logging
* Update ready for launch files and for rosrun
* Added quantisation and compression utility for fast data storage
* No requirement for cv2 package
* Fix 16MB test
* Add compression.py file to handle on the fly compression
* Update LICENCE
* Added compression feature (experimentation flag in sanitation.py)
  Idea is to compress all Images using either jpeg (colour) or png (depth) encoding to CompressedImage encoding.
* Removes Jenkins Build Status (`#24 <https://github.com/RaymondKirk/topic_store/issues/24>`_)
* Full filesystem error message
* Defaults for run_scenario.py
* Add ability to run on single thread (threads=0) or offload jobs (threads>=1)
* Add ability to toggle GridFS
* Add message size to info topic
* Added topic monitoring script
* Update store with partial auto logger functions
* Customise log publisher topic
* Update year on licence
* Add parameters relating to thread management
* Improvements to load balancer (auto flag, worker closure, worker track)
* GridFS improvements (create index and default chunk size increase)
* Rename run_scenario node to anonymous (`#21 <https://github.com/RaymondKirk/topic_store/issues/21>`_)
* Temp fix for `#20 <https://github.com/RaymondKirk/topic_store/issues/20>`_ to allow filesystem conversion
* Merge pull request `#19 <https://github.com/RaymondKirk/topic_store/issues/19>`_ from RaymondKirk/new_sanitation_method
  New sanitation method
* Log IO Queue messages to info log to ensure user sees them
* Simple ROS types will not be converted anymore
  TopicStore officially now works only with hierarchical mappable types such as dict. To convert simple ROS messages/types please put in a dict.
* Added tests for sanitation methods
* Overhaul of sanitation methods
  Previous implementation was unreliable and had an issue with objects in different parts of the nested dict would be magically replaced randomly due to objects in python being created with the same id. Some unavoidable copy by reference errors in last implementation, now all are copied by value. Note it is still possible to mess this up by running convert dictionary functions on the same dict objects at different times.
* Data sanitation by default (instead of DefaultTypeParser)
  Types will always be dict so just use optimised sanitation method. Type parsers now only used for Mongo/Other quick and dirty conversions i.e binary to str.
* Add convert binary (copy of python script)
* Smart resolve scenario files (search in package root if doesn't exist)
* Fixed type coercion and sanitation methods
* Better logging if system has the support
* Added verbose flag to launch file
* Improvements to load balancer class
* Implemented automatic load balancing to handle disk/db IO
* Re-add of conversion utility
* Fixed sanitation of other genpy types
* Initial commit of a better data sanitation technique (covers all instances)
* Merge pull request `#18 <https://github.com/RaymondKirk/topic_store/issues/18>`_ from RaymondKirk/python3-library
  Added python package setup and ROS independence support
* Add conversion functionality from cli even when installed with PyPi
* Fixed installation instructions
* Fixed link to PyPi
* Fixed link to PyPi
* Added PyPi button
* Add non-ROS installation scripts and instructions
* Don't break if no ros install on machine
* Added better readme tutorial for reading data
* Python2/3 compatible filesystem loading
* Added temp scenario parser fix and TODO
* Fix iteritems function for py2/py3 compatibility and allow dict insert
* Fix unicode and is_string functions for py2/py3 compatibility
* Merge branch 'master' of https://github.com/RaymondKirk/topic_store into python3-library
* Merge pull request `#15 <https://github.com/RaymondKirk/topic_store/issues/15>`_ from pet1330/gh-ci
  Adds GitHub Action CI
* Update topic_store.test
* Update CI.yml
  Remove unneeded step
* Update CI Badge
* assert database connection before tests
* Update CI config values
* Adds GitHub CI
* Cleaned GUI files and added visualiser base
* Initial commit of basic GUI
* Merge pull request `#13 <https://github.com/RaymondKirk/topic_store/issues/13>`_ from RaymondKirk/mongo_uid_fix
  Add defaults for UID:GID and use the curent user for topic_store
* Squeeze in wiki updates
* checks for interative env to protect ci failing
* Added warning message for UID > 1000
* Removed comments
* Add defaults for UID:GID and use the curent user for topic_store
* Publish topic_store logs to a topic
* Auto catch errors in the MongoDB convert utility and skip
* Added ability to skip documents that error (catch exceptions)
* Added utility function to get flat {ros_topic: ros_topic_value, ...} dict
* More robust check for kwargs
* Fix for `#12 <https://github.com/RaymondKirk/topic_store/issues/12>`_ to always return _ts_meta
* Merge pull request `#11 <https://github.com/RaymondKirk/topic_store/issues/11>`_ from RaymondKirk/database_fixes
  Fixes for database (description in commit info)
* Fixes for database (description in commit info)
  If projection field is present and _ts_meta isn't specified then TopicStore will assume it is a new message and reconstruct now. So we force it to always retrieve this meta data to ensure messages are always reconstructed correctly.
  On slow connections auto-fetching gridFS or blob data can be a dealbreaker so a "skip_fetch_binary" flag has been added.
* Contributors: Nikos Tsagkopoulos, Peter Lightbody, Raymond Tunstill (Kirk), RaymondKirk

0.1.2 (2020-10-19)
------------------
* Merge pull request `#8 <https://github.com/RaymondKirk/topic_store/issues/8>`_ from RaymondKirk/generic_db_rosbag_support
  Generic stream from DB uri and query support to rosbag and topic objects
* Add missing projection kwargs from tests
* Raise warn for slot errors
* Updated README.md to include road map and topic_store
* Move rostopic import to AutoSubscriber msg definition ready for PyPi
* Projection improvements (include file system and get by session)
* Remove default query cli arg
* Merge pull request `#10 <https://github.com/RaymondKirk/topic_store/issues/10>`_ from marc-hanheide/feature_projection
  Feature projection
* added missing bits of documentation
* added first documentation for mongo->rosbag
* added projection for mongo->rosbag
* ignore errors in slot filling
  that are due to an outdated ROS message definition
* Fixed options parsing to not break on malformed uris
* Better authSource parsing
* Initial support for generic DB queries and URI inputs to ROSbag and topic objects
* Allow db namme specification in database.py
* Contributors: Marc Hanheide, Raymond Tunstill (Kirk), RaymondKirk

0.1.1 (2020-08-26)
------------------
* Merge pull request `#7 <https://github.com/RaymondKirk/topic_store/issues/7>`_ from RaymondKirk/add_action_server_video
  Added ability to collect sequences with the action server
* Added ability to collect sequences with the action server
* Contributors: Raymond Tunstill (Kirk), RaymondKirk

0.1.0 (2020-07-16)
------------------
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
