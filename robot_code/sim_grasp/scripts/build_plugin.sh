#!/bin/bash
set -x
BASE=$HOME/gazebo-pkgs
cd $BASE/gazebo_grasp_plugin/src
g++ -shared -fPIC -std=c++17 \
  -I$BASE/gazebo_grasp_plugin/include \
  -I$BASE/gazebo_version_helpers/include \
  -I$BASE/gazebo_grasp_plugin \
  $(pkg-config --cflags gazebo) \
  GazeboGraspFix.cpp GazeboGraspGripper.cpp \
  $BASE/gazebo_version_helpers/src/GazeboVersionHelpers.cpp \
  $BASE/gazebo_grasp_plugin/msgs/grasp_event.pb.cc \
  $(pkg-config --libs gazebo) -lprotobuf \
  -o $HOME/gazebo_plugins/libgazebo_grasp_fix.so 2>&1 | tail -30
echo "BUILD_RC=$?"
ls -la $HOME/gazebo_plugins/libgazebo_grasp_fix.so
