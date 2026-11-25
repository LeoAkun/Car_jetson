#!/bin/bash

PROCESS_NAME="app_multiprocessing.py"
PIDS=$(ps aux | grep "$PROCESS_NAME" | grep -v "grep" | awk '{print $2}')
if [ -z "$PIDS" ]; then
    echo "No process found with name: $PROCESS_NAME"
else
    echo "Found processes with PIDs:"
    echo "$PIDS"
    # 结束进程
    for PID in $PIDS; do
        echo "Terminating process with PID: $PID"
        kill -9 $PID
        if [ $? -eq 0 ]; then
            echo "Process $PID terminated successfully."
        else
            echo "Failed to terminate process $PID."
        fi
    done
fi