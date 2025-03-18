#!/bin/bash

# force kills all the processes so that run.sh can be re run
echo "killing all processes.."
killall -9 bridge_server
echo "processes killed successfuly."