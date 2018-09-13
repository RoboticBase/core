#!/bin/bash

PID_DIAGNOSTICS=$(ps aux | grep "__name:=turtlebot3_diagnostics" | grep -v grep | aws '{ print $2 }')
PID_LDS=$(ps aux | grep "__name:=turtlebot3_lds" | grep -v grep | awk '{ print $2 }')
PID_CORE=$(ps aux | grep "__name:=turtlebot3_core" | grep -v grep | aws '{ print $2 }')

echo "send SIGINT to PID_DIAGNOSTICS=${PID_DIAGNOSTICS}, PID_LDS=${PID_LDS} PID_CORE=${PID_CORE}"
kill -INT ${PID_DIAGNOSTICS}
kill -INT ${PID_LDS}
kill -INT ${PID_CORE}
