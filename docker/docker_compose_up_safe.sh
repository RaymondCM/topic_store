#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# File to safely bring up a mongod instance inside a docker container and load environment variables from YAML config
# Not for direct use, instead use roslaunch topic_store start_database.launch or topic_store.MongoClient()
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"
set -e

# Will exit if rospack not available/not sourced in topic root workspace
pkg_root=$(rospack find topic_store)

error_str="\e[1m\e[41m\e[97mERROR:\e[0m"
script_usage="\e[1mMONGO_DB_CONFIG_PATH=<path> ./docker_compose_up_safe.sh\e[0m"

# Check environment variable has been set if not print usage
if [ -z "$MONGO_DB_CONFIG_PATH" ]; then
    echo -e "\n$error_str You must set the MONGO_DB_CONFIG_PATH environment variable containing a database config file."
    echo -e "$error_str An example config can be found at '\e[96m$pkg_root/config/default_db_config.yaml\e[0m'."
    echo -e "\n\e[1m\e[41m\e[97mScript Usage:\e[0m '$script_usage'\n"
    exit 1
fi

# Default storage path in in the current sourced package
function join_path() {
    echo "${1:+$1/}$2" | sed 's#//#/#g'
}
MONGO_storage_dbPath=$(join_path "$pkg_root" "/stored_topics/database")

# Load environment variables from ${MONGO_DB_CONFIG_PATH}
source parse_yaml.sh
eval "$(parse_yaml ../config/default_db_config.yaml "MONGO_")"

# Ensure no leading comments/spaces from parser
MONGO_storage_dbPath=$(echo "${MONGO_storage_dbPath}"  | cut -f1 -d"#")
MONGO_net_port=$(echo "${MONGO_net_port}" | cut -f1 -d" ")

# Export variables needed by docker-compose and ensure no leading comments
export TOPIC_STORE_ROOT=${pkg_root}
export MONGO_DB_CONFIG_PATH=$MONGO_DB_CONFIG_PATH
export MONGO_storage_dbPath=$MONGO_storage_dbPath
export MONGO_net_port=$MONGO_net_port

# Log to console
echo -e "Starting \e[93mMongoDB Server\e[0m:\n"\
        "\t- Using config '\e[96m${MONGO_DB_CONFIG_PATH}\e[0m'.\n"\
        "\t- Docker port '\e[96m${MONGO_net_port}\e[0m' bound to local '\e[96m${MONGO_net_port}\e[0m'.\n"\
        "\t- At '\e[96m${MONGO_storage_dbPath}\e[0m' local mount point.\e[0m\n"

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