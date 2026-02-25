#!/bin/bash
rm -f "$HOME/AVROS/src/avros_sim/worlds/cpp_campus_osm.wbt"

WEBOTS_HOME="$HOME/webots" python3 "$HOME/webots/resources/osm_importer/importer.py" \
  --input="$HOME/AVROS/src/avros_sim/worlds/cpp_campus.osm" \
  --output="$HOME/AVROS/src/avros_sim/worlds/cpp_campus_osm.wbt" \
  --no-intersection-road-lines
