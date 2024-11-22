#!/bin/bash

ROS_WS_PATH=`echo $ROS_PACKAGE_PATH | cut -d':' -f1`
argos3 -c $ROS_WS_PATH/../build/test1/scripts/configuration.argos