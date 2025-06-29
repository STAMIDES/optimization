#!/bin/bash

set -e

# Download the OSM data
mkdir -p /data && cd /data
curl -O http://download.geofabrik.de/south-america/uruguay-latest.osm.pbf

# Extract and contract the OSM data
osrm-extract -p /opt/car.lua /data/uruguay-latest.osm.pbf
osrm-contract /data/uruguay-latest.osrm
