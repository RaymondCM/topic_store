#  Raymond Kirk (Tunstill) Copyright (c) 2019
#  Email: ray.tunstill@gmail.com

# Provides conversion methods between different storage types

from __future__ import absolute_import, division, print_function

import json
import sys
import rospy
import rosbag
import pathlib
import pymongo
import argparse
from tqdm import tqdm
from datetime import datetime
from pymongo.collection import ObjectId

from topic_store.file_parsers import resolve_scenario_yaml

try:
    from urlparse import urlparse
except ImportError:  # Py3
    from urllib.parse import urlparse

try:
    input = raw_input  # Py2
except NameError:
    pass

from topic_store.database import MongoStorage, TopicStoreCursor
from topic_store.filesystem import TopicStorage


def topic_store_to_mongodb(topic_store_file, scenario_file):
    client = MongoStorage.load(scenario_file)
    print("Converting '{}' to MongoDB '{}'".format(topic_store_file.name, client.uri))

    storage = TopicStorage.load(topic_store_file)
    with tqdm() as progress_bar:
        for item in storage:
            try:
                client.insert_one(item)
            except pymongo.errors.DuplicateKeyError:
                print("Storage Item '_id: {}' already exists in the '{}/{}' collection".format(item.id, client.name,
                                                                                               client.collection_name))
            progress_bar.update()


def get_mongo_storage_by_session(client, *args, **kwargs):
    sessions = client.get_unique_sessions()
    if len(sessions) > 1:
        s_lut = sorted([{
            "id": sid, "on": datetime.fromtimestamp(data["time"]).strftime('%Y-%m-%d %H:%M:%S'),
            "float_time": data["time"], "count": data["count"]
        } for sid, data in sessions.items()], key=lambda x: x["float_time"])
        print("Collection {}/{} contains data from:\n{}".format(client.name, client.collection_name, ''.join(
            ["\t{}. Session {} on {} containing {} documents\n".format(i, s["id"], s["on"], s["count"]) for i, s in
             enumerate(s_lut)])))
        while True:
            try:
                char = input("Please enter a number or enter for all: ")
                if char is "":
                    return client.find(*args, **kwargs)
                return client.find_by_session_id(s_lut[int(char)]["id"], *args, **kwargs), s_lut[int(char)]["count"]
            except (EOFError, ValueError, IndexError):
                print("Please choose an appropriate option")
                continue

    query = kwargs.get("query", None) or args[0] if len(args) > 0 and isinstance(args[0], dict) else None
    return client.find(*args, **kwargs), client.count(query, estimate=not bool(query))


def count_mongodb_items(storage, query=None, estimate=False):
    if isinstance(storage, TopicStoreCursor) and sys.version_info[0] == 2:
        return storage.cursor.count()
    if isinstance(storage, MongoStorage):
        return storage.count(query, estimate=estimate)
    raise TypeError("Unsupported storage type: {}".format(type(storage)))


def get_mongodb_storage(mongodb_client, query=None, projection=None, **kwargs):
    if query is None or not isinstance(query, dict):
        storage, count = get_mongo_storage_by_session(mongodb_client, query, skip_on_error=True,
                                                      projection=projection)
    else:
        storage = mongodb_client.find(query, skip_on_error=True, projection=projection, **kwargs)
        count = mongodb_client.count(query, estimate=not bool(query))
    return storage, count


def mongodb_to_topic_store(mongodb_client, topic_store_file, query=None, projection=None):
    print("Converting MongoDB '{}' to '{}'".format(mongodb_client.uri, topic_store_file.name))

    storage, count = get_mongodb_storage(mongodb_client, query=query, projection=projection)

    topic_storage = TopicStorage(topic_store_file)

    with tqdm(total=count) as progress_bar:
        for item in storage:
            topic_storage.insert_one(item)
            progress_bar.update()


def mongodb_to_mongodb(mongodb_from, mongodb_to, query=None, projection=None):
    print("Converting MongoDB '{}' to '{}'".format(mongodb_from.uri, mongodb_to.uri))

    storage, count = get_mongodb_storage(mongodb_from, query=query, projection=projection)
    database_name, collection_name = mongodb_to.name, mongodb_to.collection_name

    errors = {}

    with tqdm(total=count) as progress_bar:
        for item in storage:
            try:
                mongodb_to.insert_one(item)
            except Exception as e:
                exception_name = type(e).__name__
                if exception_name not in errors:
                    errors[exception_name] = []
                errors[exception_name].append(item.id)

            if errors:
                error_str = ", ".join("{} ({})".format(k, len(v)) for k, v in errors.items())
                progress_bar.set_postfix_str(error_str)
            progress_bar.update()

    skipped = sum(len(v) for v in errors.values())
    print("Copied {} items, skipped {} items:".format(count - skipped, skipped))

def mongodb_to_mongodb_clone_fast(mongodb_from, mongodb_to, query=None, projection=None):
    print("Converting MongoDB '{}' to '{}'".format(private_srv(mongodb_from.uri), private_srv(mongodb_to.uri)))
    id_only_projection = projection or {"_id": 1}
    fast_id_only_kwargs = dict(projection=id_only_projection, include_ts_meta=False, sort=[("_id", 1)], raw_cursor=True)
    existing_ids = set(x["_id"] for x in mongodb_to.find({}, **fast_id_only_kwargs))

    # Create query for only IDs but not ids that already exist
    id_only_query = {"_id": {"$nin": list(existing_ids)}}
    query_mb = sys.getsizeof(json.dumps(id_only_query, default=str)) / 1024 / 1024
    if query_mb >= 15:
        print("Query is {}MB, this is too large to be sent to the server, defaulting to naive query".format(query_mb))
        id_only_query = {}

    fast_id_only_kwargs = dict(projection=id_only_projection, include_ts_meta=False, sort=[("_id", -1)], raw_cursor=True)
    storage = mongodb_from.find(id_only_query, **fast_id_only_kwargs)
    count = mongodb_from.count(id_only_query, estimate=not bool(id_only_query))

    errors = {"DuplicateKeyError": list(existing_ids)}
    postfix = lambda x, y: f"{x + ' - ' if x else ''}{'Errors: ' + y if y else ''}"

    with tqdm(total=count) as progress_bar:
        for item in storage:
            neg_str = ", ".join("{} ({})".format(k, len(v)) for k, v in errors.items()) if errors else ""
            item_id = item["_id"]
            if item_id not in existing_ids:
                try:
                    pos_str = f"Retrieving {item_id}"
                    progress_bar.set_postfix_str(postfix(pos_str, neg_str))
                    full_item = mongodb_from.find_by_id(item_id)
                    pos_str = f"Inserting {item_id}"
                    progress_bar.set_postfix_str(postfix(pos_str, neg_str))
                    mongodb_to.insert_one(full_item)
                except Exception as e:
                    pos_str = ""
                    exception_name = type(e).__name__
                    if exception_name not in errors:
                        errors[exception_name] = []
                    errors[exception_name].append(item_id)
            else:
                exception_name = "DuplicateKeyError"
                if exception_name not in errors:
                    errors[exception_name] = []
                pos_str = f"Skipping {item_id}"
                progress_bar.set_postfix_str(postfix(pos_str, neg_str))
                errors[exception_name].append(item_id)

            neg_str = ", ".join("{} ({})".format(k, len(v)) for k, v in errors.items()) if errors else ""
            progress_bar.set_postfix_str(postfix(pos_str, neg_str))
            progress_bar.update()

    skipped = sum(len(v) for v in errors.values())
    print("Copied {} items, skipped {} items:".format(count - skipped, skipped))


def mongodb_to_ros_bag(mongodb_client, output_file, query=None, projection=None):
    print("Converting MongoDB '{}' to ROS bag '{}'".format(mongodb_client.uri, output_file.name))

    storage, count = get_mongodb_storage(mongodb_client, query=query, projection=projection)

    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm(total=count) as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    if hasattr(msg, "_connection_header"):
                        source = getattr(msg, "_connection_header")["topic"]
                        if source:
                            try:
                                ros_bag.write(source, msg, time)
                            except Exception as e:
                                print("Could not write", source, 'because', e.message)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def topic_store_to_ros_bag(topic_store_file, output_file):
    print("Converting '{}' to ROS bag '{}'".format(topic_store_file.name, output_file.name))
    storage = TopicStorage.load(topic_store_file)
    ros_bag = rosbag.Bag(str(output_file), 'w')

    try:
        with tqdm() as progress_bar:
            for item in storage:
                msgs = item.to_ros_msg_list()
                time = rospy.Time.from_sec(item["_ts_meta"]["ros_time"])
                for msg in msgs:
                    if hasattr(msg, "_connection_header"):
                        source = getattr(msg, "_connection_header")["topic"]
                        if source:
                            try:
                                ros_bag.write(source, msg, time)
                            except Exception as e:
                                print("Could not write", source, 'because', e.message)
                progress_bar.update()
    finally:
        print("Closing the ROS bag '{}'".format(output_file))
        ros_bag.close()


def is_uri(uri):
    return isinstance(uri, str) and uri.startswith("mongodb://")


def client_from_uri(uri, collection):
    if is_uri(uri):
        # DB name will usually be specified as authSource in the URI, if not present use default=topic_store
        db_name = None
        if "authSource" in uri:
            # get options from URI
            options = dict([x.split("=") for x in uri.split("?")[1].split("&")])
            if "authSource" in options:
                db_name = options["authSource"]
        client = MongoStorage(collection=collection, uri=uri, db_name=db_name)
        return client
    else:
        raise ValueError("Not a valid URI: {}".format(uri))


def private_srv(srv):
    original_type = type(srv)
    srv = str(srv)
    if ":" in srv and "@" in srv:
        srv = f"mongodb://****:****@" + srv.split("@")[1]
    return original_type(srv)


def _convert_cli():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="Input File", type=str, required=True)
    parser.add_argument("-o", "--output", help="Output File", type=str, required=True)
    parser.add_argument("-c", "--collection", help="MongoDB collection to use if URI passed as --input", type=str,
                        required=False)
    parser.add_argument("-q", "--query", help='MongoDB input query as dict (example: -q '
                                              '\'{"_id": "ObjectId(5f718a354e5e8239dcd1eca1)"}\'', type=str,
                        required=False, default=None)
    parser.add_argument("-p", "--projection", help='MongoDB input projection as dict (example: -p \'{"name": 1}\'',
                        type=str, required=False, default=None)
    args = parser.parse_args()

    rospy.init_node("topic_store_convert", anonymous=True)
    input_path = pathlib.Path(args.input)
    output_path = pathlib.Path(args.output)

    # if not input_path.exists():
    #     raise IOError("Input file '{}' does not exist".format(input_path))

    if input_path.suffix == ".bag":
        raise NotImplementedError("Converting from ROS bags is not currently supported. "
                                  "The conversion to ROS bags is lossy and requires adding meta data to reconstruct"
                                  "the original .topic_store or database documents")
    elif input_path.suffix == TopicStorage.suffix and output_path.suffix == ".bag":
        topic_store_to_ros_bag(input_path, output_path)
    elif input_path.suffix == ".yaml" and output_path.suffix == TopicStorage.suffix:
        input_path = resolve_scenario_yaml(input_path)
        mongodb_to_topic_store(MongoStorage.load(input_path), output_path)
    elif input_path.suffix == ".yaml" and output_path.suffix == ".bag":
        input_path = resolve_scenario_yaml(input_path)
        mongodb_to_ros_bag(MongoStorage.load(input_path), output_path)
    elif input_path.suffix == TopicStorage.suffix and output_path.suffix == ".yaml":
        output_path = resolve_scenario_yaml(output_path)
        topic_store_to_mongodb(input_path, output_path)
    elif is_uri(args.input):
        srv = args.input
        collection = args.collection
        query = args.query
        projection = args.projection

        # if not hasattr(args, "query") or not args.query:
        #     raise ValueError("If input is a MongoDB URI you must specify a DB query -q/--query to export data")
        if not hasattr(args, "collection") or not args.collection:
            raise ValueError("If input is a MongoDB URI you must specify a DB collection -c/--collection to query data")

        # Try to parse a query/projection string to a dict and perform some basic cleaning
        # The query string will filter the db documents by client.find(query)
        if query is not None:
            try:
                query, projection = [x if x is None else json.loads(x) for x in [args.query, args.projection]]
            except ValueError:
                print("Query/Projection parameter cannot be parsed as a python dict \nQ: '{}'\nP: '{}'".format(
                    args.query, args.projection))
                raise

        # Some simple rules to support searching by ID from console
        if query:
            for k, v in query.items():
                try:
                    unicode
                except NameError:
                    unicode = str
                if isinstance(v, (str, unicode)) and (v.startswith('ObjectId(') and v.endswith(')')):
                    print("Converting query field '{}' to ObjectId".format(k))
                    query[k] = ObjectId(str(v[9:-1]))

        client = client_from_uri(srv, collection=collection)

        if is_uri(args.output):
            db2 = client_from_uri(args.output, collection=collection)
            mongodb_to_mongodb(client, db2, query=query, projection=projection)
        elif output_path.suffix == ".bag":
            mongodb_to_ros_bag(client, output_path, query=query, projection=projection)
        elif output_path.suffix == TopicStorage.suffix:
            mongodb_to_topic_store(client, output_path, query=query, projection=projection)
        else:
            raise ValueError("No valid conversion from Mongo URI '{}' to '{}' file".format(client.uri, output_path))
    elif input_path.suffix == output_path.suffix:
        print("No conversion or migration for '{}' to '{}'".format(input_path, output_path))
        print("If you would like to copy the file please use 'cp {} {}'".format(input_path, output_path))
    else:
        print("No conversion or migration for '{}' to '{}'".format(input_path, output_path))


if __name__ == '__main__':
    _convert_cli()
