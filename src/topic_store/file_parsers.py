#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

import yaml

import pathlib

__all__ = ["load_yaml_file", "ScenarioFileParser"]


def load_yaml_file(file_path):
    if isinstance(file_path, str):
        file_path = pathlib.Path(file_path)
    with file_path.open("r") as file_handle:
        try:
            contents = yaml.safe_load(file_handle)
        except yaml.YAMLError as exc:
            raise IOError(exc)
    return contents


class ScenarioFileParser:
    __field_meta = {
        "context": "",
        "collection": {
            "action_server_video": ["method", "action_server_name", "watch_topic"],
            "action_server": ["method", "action_server_name"],
            "timer": ["method", "timer_delay"],
            "event": ["method", "watch_topic"]
        },
        "storage": {
            "database": ["method", "config"],
            "filesystem": ["method", "location"]
        },
        "data": {}
    }

    def __init__(self, scenario_file):
        scenario = load_yaml_file(scenario_file)

        # Perform file checks (ensure all four sections exist and are the right types)
        for field in self.__field_meta:
            if field not in scenario:
                raise Exception("'{}' field missing from scenario file: {}".format(field, scenario_file))
            if not isinstance(scenario[field], type(self.__field_meta[field])):
                raise Exception("'{}' field should be type '{}' not '{}'".format(field, type(self.__field_meta[field]),
                                                                                 type(scenario[field])))
        # Parse context info
        self.context = scenario["context"]

        # Parse storage Info
        self.storage = scenario["storage"]
        if "method" not in self.storage:
            raise Exception("storage.method must be either " + ', '.join(list(self.__field_meta["storage"].keys())))
        for required_parameter in self.__field_meta["storage"][self.storage["method"]]:
            if required_parameter not in self.storage:
                raise Exception("Storage field in YAML file must have the parameter '{}' when method=='{}'".format(
                    required_parameter, self.storage["method"]
                ))
        # TODO: This errors if using a filesystem config for db scenario or visa-versa
        all_storage_keys = self.storage.keys()
        for parameter in all_storage_keys:  # Delete parameters that won't be used
            if parameter not in self.__field_meta["storage"][self.storage["method"]]:
                del self.storage[parameter]

        # Data should just be a dict of key: data lookups
        self.data = scenario["data"]

        # Parse collection info
        self.collection = scenario["collection"]
        if "method" not in self.collection:
            raise Exception("collection.method must be either " + ', '.join(self.__field_meta["collection"].keys()))
        for required_parameter in self.__field_meta["collection"][self.collection["method"]]:
            if required_parameter not in self.collection:
                raise Exception("Collection field in YAML file must have the parameter '{}' when method=='{}'".format(
                    required_parameter, self.collection["method"]
                ))
        for parameter in self.collection.keys():  # Delete parameters that won't be used
            if parameter not in self.__field_meta["collection"][self.collection["method"]]:
                del self.collection[parameter]

    def require_database(self):
        if self.storage["method"] != "database":
            raise ValueError("Scenario file is not configured for database connection as storage.method=={}".format(
                self.storage["method"]
            ))
        return self

    def require_filesystem(self):
        if self.storage["method"] != "filesystem":
            raise ValueError("Scenario file is not configured for filesystem storage as storage.method=={}".format(
                self.storage["method"]
            ))
        return self

    @staticmethod
    def cmd_line(file_path, prefix, sep, require_db=False):
        """Imported in 'bash' to evaluate YAML files"""
        file_path = pathlib.Path(str(file_path))
        if not file_path.is_file():
            raise IOError("'{}' is not a valid file".format(file_path))
        # These characters need to be removed (or escaped) in when these vars evaluated
        _remove_symbols = list('$\\`!()#*&\t[]{}|;\'"\n<>?')

        def rec_print_dict(d, previous_key_str=""):
            for yaml_key, yaml_value in d.items():
                if isinstance(yaml_value, dict):
                    rec_print_dict(yaml_value, previous_key_str + yaml_key + sep)
                else:
                    yaml_value = ''.join([c for c in str(yaml_value) if c not in _remove_symbols])
                    print('{}="{}"'.format(prefix + sep + previous_key_str + yaml_key, yaml_value))

        doc = load_yaml_file(file_path)
        if require_db:
            try:
                ScenarioFileParser(file_path).require_database()
            except Exception:
                raise Exception("File '{}' is not a valid Scenario Database Config!".format(file_path))
        rec_print_dict(doc)
