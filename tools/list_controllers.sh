#!/usr/bin/env bash
# Prints "<controller> <state>" per line. Calls the controller_manager service directly:
# `ros2 control list_controllers` goes through the ros2 CLI daemon, which wedges after the
# first call in this WSL setup and then hangs every later call (ignoring SIGTERM).
timeout -k 2 8 ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers 2>/dev/null \
  | grep -oE "name='[a-z_]+', state='[a-z]+'" | sed -E "s/name='([a-z_]+)', state='([a-z]+)'/\1 \2/"
