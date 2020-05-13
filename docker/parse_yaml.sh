#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Python function to load yaml and print to STD_OUT
_get_yaml_python=$(cat <<'EOF'
import yaml, sys, os
file_path, prefix, sep = sys.argv[1], sys.argv[2], sys.argv[3]
if not os.path.isfile(file_path):
    raise IOError("'{}' is not a valid file".format(file_path))
# These characters need to be escaped in when these vars are evaluated (`~!#$&)
_invalid = ["$", "\\", "`", "!", "(", ")", "#", "&", "*", "&", "\t", "[", "]", "{", "}", "|", ";", "'", '"', "\n",
            "<", ">", "?"]
def rec_print_dict(d, p=""):
    for k, v in d.items():
        if isinstance(v, dict):
            rec_print_dict(v, p+k+sep)
        else:
            v = str(v)
#            if any(c in _invalid for c in v):
#                raise ValueError("Warning in file '{}' the '{}'='{}' variable has '{}' symbols in which can be dangerous".format(file_path, k, v, ', '.join(_invalid)))
            for i in _invalid:
                v = v.replace(i, "\\" + i)
            print('{}="{}"'.format(prefix+sep+p+k, v))
with open(file_path, "r") as f:
    doc = yaml.load(f)
    rec_print_dict(doc)
EOF
)

# Usage:
#   To print: parse_yaml "$file_path" "$prefix" "$sep"
#   As variables: eval $(parse_yaml "$file_path" "$prefix" "$sep")
function parse_yaml() {
    file_path=${1}
    prefix_str=${2:-""}
    sep_str=${3:-"_"}
    local fs=$(echo @ | tr @ '\034')
    echo "$(python -c "$_get_yaml_python" "${file_path}" "${prefix_str}" "${sep_str}")"
}