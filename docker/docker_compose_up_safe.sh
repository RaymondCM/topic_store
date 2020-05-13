#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# File to safely bring up a mongod instance inside a docker container and load environment variables from YAML config
# Not for direct use, instead use roslaunch topic_store start_database.launch or topic_store.MongoClient()
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"
set -e

error_str="\e[1m\e[41m\e[97mERROR:\e[0m"
script_usage="\e[1m./docker_compose_up_safe.sh /path/to/scenario_file.yaml\e[0m"

function join_path() {
    echo "${1:+$1/}$2" | sed 's#//#/#g'
}

# Will exit if rospack not available/not sourced in topic root workspace
pkg_root=$(rospack find topic_store)

# Get scenario file from arg $1
scenario_file=${1:-"$(join_path "$pkg_root" "/scenarios/database_default.yaml")"}
if ! [ -f "${scenario_file}" ]; then
    echo -e "\n$error_str The scenario file '\e[96m${scenario_file}\e[0m' does not exist"
    exit 1
fi

# Load mongo config path from the scenario file
source parse_yaml.sh # TODO: Using eval can lead to weird results/evaluations if $ or \ symbols used in the YAML values, fix this
eval "$(parse_yaml "${scenario_file}" "TS_SCENARIO" "_" "1")"

# If config is any of the magic strings replace with default config file similar to how MongoClient will
if [[ "default topic_store auto" =~ (^|[[:space:]])"$TS_SCENARIO_storage_config"($|[[:space:]]) ]] ; then
    TS_SCENARIO_storage_config="$(join_path "$pkg_root" "/config/default_db_config.yaml")"
fi

# Check environment variable has been set if not print usage
if [ -z "$TS_SCENARIO_storage_config" ]; then
    echo -e "\n$error_str You must set the storage.config variable in the scenario containing a database config file."
    echo -e "$error_str An example config can be found at '\e[96m$pkg_root/config/default_db_config.yaml\e[0m'."
    echo -e "\n\e[1m\e[41m\e[97mScript Usage:\e[0m '$script_usage' where the storage.method==database\n"
    exit 1
fi

if ! [ -f "${TS_SCENARIO_storage_config}" ]; then
    echo -e "\n$error_str The database config file '\e[96m${TS_SCENARIO_storage_config}\e[0m' does not exist"
    echo -e "$error_str An example config can be found at '\e[96m$pkg_root/config/default_db_config.yaml\e[0m'."
    echo -e "\n\e[1m\e[41m\e[97mScript Usage:\e[0m '$script_usage' where the storage.method==database\n"
    exit 1
fi

# Default storage path in in the current sourced package
MONGO_storage_dbPath=$(join_path "$pkg_root" "/stored_topics/database")

# Load environment variables from ${TS_SCENARIO_storage_config}
eval "$(parse_yaml "${TS_SCENARIO_storage_config}" "MONGO" "_")"

# Export variables needed by docker-compose and ensure no leading comments
export TOPIC_STORE_ROOT=${pkg_root}
export TS_SCENARIO_storage_config=$TS_SCENARIO_storage_config
export MONGO_storage_dbPath=$MONGO_storage_dbPath
export MONGO_net_port=$MONGO_net_port

# Log to console
echo -e "Starting \e[93mMongoDB Server\e[0m:\n"\
        "\t- Using scenario '\e[96m${scenario_file}\e[0m'.\n"\
        "\t- Using config from scenario '\e[96m${TS_SCENARIO_storage_config}\e[0m'.\n"\
        "\t- Docker port '\e[96m${MONGO_net_port}\e[0m' bound to local '\e[96m${MONGO_net_port}\e[0m'.\n"\
        "\t- At '\e[96m${MONGO_storage_dbPath}\e[0m' local mount point.\e[0m\n"

# Check exports are set and correct
if [[ -z "$TOPIC_STORE_ROOT" || -z "$TS_SCENARIO_storage_config" || -z "$MONGO_storage_dbPath" || -z "$MONGO_net_port" ]]; then
    echo -e "\n$error_str The above parameters are invalid"
    echo -e "\n\e[1m\e[41m\e[97mScript Usage:\e[0m '$script_usage' where the storage.method==database\n"
    exit 1
fi

# Function to stop the database on exit sig_(exit int term)
function cleanup {
    docker-compose down
    exit 0
}
trap cleanup INT TERM EXIT

# Bring container down if already up (avoid corruption)
docker-compose down

# Bring container up and block execution
docker-compose up