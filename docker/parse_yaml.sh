#!/usr/bin/env bash
#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# Inspired by: https://github.com/mrbaseman/parse_yaml.git (changed since the git version hangs)

function parse_yaml {
   local prefix=$2
   local s='[[:space:]]*'
   local w='[a-zA-Z0-9_]*'
   local fs=$(echo @ | tr @ '\034')
   sed -ne "s|,$s\]$s\$|]|" \
        -e ":1;s|^\($s\)\($w\)$s:$s\[$s\(.*\)$s,$s\(.*\)$s\]|\1\2: [\3]\n\1  - \4|;t1" \
        -e "s|^\($s\)\($w\)$s:$s\[$s\(.*\)$s\]|\1\2:\n\1  - \3|;p" $1 | \
   sed -ne "s|,$s}$s\$|}|" \
        -e ":1;s|^\($s\)-$s{$s\(.*\)$s,$s\($w\)$s:$s\(.*\)$s}|\1- {\2}\n\1  \3: \4|;t1" \
        -e    "s|^\($s\)-$s{$s\(.*\)$s}|\1-\n\1  \2|;p" | \
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)-${s}[\"']\(.*\)[\"']$s\$|\1$fs$fs\2|p" \
        -e "s|^\($s\)-$s\(.*\)$s\$|\1$fs$fs\2|p" \
        -e "s|^\($s\)\($w\)$s:${s}[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p" | \
   awk -F"${fs}" '{
      indent = length($1)/2;
      yaml_var_name[indent] = $2;
      for (i in yaml_var_name) {if (i > indent) {delete yaml_var_name[i]; idx[i]=0}}
      if(length($2)== 0){  yaml_var_name[indent]= ++idx[indent] };
      if (length($3) > 0) {
         vn=""; for (i=0; i<indent; i++) { vn=(vn)(yaml_var_name[i])("_")}
         printf("%s%s%s=\"%s\"\n", "'"${prefix}"'",vn, yaml_var_name[indent], $3);
      }
   }'
}