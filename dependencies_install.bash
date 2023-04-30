#!/bin/bash

# Install both Python Libraries and ROS Packages

# Search for requirements.txt files in all subdirectories of the src folder and install the dependencies using pip3

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

find $DIR/src -name requirements.txt -type f | while read package; do
    echo "Installing dependencies for $package"
    pip3 install -r "$package"
done

rosdep update
rosdep install --from-paths $DIR/src -y --ignore-src