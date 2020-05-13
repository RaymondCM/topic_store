#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Python function to load yaml and print to STD_OUT
_get_yaml_python=$(cat <<'EOF'
import sys
try:
    from topic_store.file_parsers import ScenarioFileParser
except Exception:
    raise Exception("The topic_store package is not available in your environment, have you built/installed it?")
file_path, prefix, sep, require_db = sys.argv[1:]
ScenarioFileParser.cmd_line(file_path, prefix, sep, require_db)
EOF
)

# Usage:
#   To print: echo $(parse_yaml "$file_path" "$prefix" "$sep" "$require_db")
#   As variables: eval $(parse_yaml "$file_path" "$prefix" "$sep" "$require_db")
function parse_yaml() {
    file_path=${1}
    prefix_str=${2:-""}
    sep_str=${3:-"_"}
    require_db=${4:-""}
    python -c "$_get_yaml_python" "${file_path}" "${prefix_str}" "${sep_str}" "${require_db}"
}