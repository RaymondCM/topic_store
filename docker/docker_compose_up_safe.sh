#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# File to safely bring up a mongod instance inside a docker container
cd "$(cd -P -- "$(dirname -- "$0")" && pwd -P)"
set -e

error_str="\n\e[1m\e[41m\e[97mERROR:\e[0m"
script_usage="\e[1mMONGO_DB_PATH=<path> ./docker_compose_up_safe.sh \e[0m"

function join_path() {
    echo "${1:+$1/}$2" | sed 's#//#/#g'
}

# Check environment variable has been set if not print usage
if [ -z "$MONGO_DB_PATH" ]; then
    echo -e "$error_str You must set the MONGO_DB_PATH environment variable or call $script_usage\n"
    exit 1
fi

# Log to console
echo -e "Starting \e[93mMongoDB Docker Server\e[0m:\n\t- At '\e[96m${MONGO_DB_PATH}\e[0m' local mount point.\e[0m\n"

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