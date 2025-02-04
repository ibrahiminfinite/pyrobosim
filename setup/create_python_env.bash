#!/bin/bash

# Sets up the pyrobosim virtual environment

# User variables
VIRTUALENV_FOLDER=~/python-virtualenvs/pyrobosim

# Create a Python virtual environment
[ ! -d "$VIRTUALENV_FOLDER" ] && mkdir -p $VIRTUALENV_FOLDER
python3 -m venv $VIRTUALENV_FOLDER
source $VIRTUALENV_FOLDER/bin/activate
echo "Created Python virtual environment in $VIRTUALENV_FOLDER"

# Install all the Python packages required
# Note that these overlay over whatever ROS 2 already contains
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
pushd $SCRIPT_DIR/..
python -m pip install --upgrade pip
# Install catkin-pkg because https://github.com/colcon/colcon-ros/issues/118
pip install catkin-pkg empy lark pytest pytest-dependency pytest-html wheel
pip install -e pyrobosim
popd
echo "Installed Python packages"
