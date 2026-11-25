cd ../mjpg-streamer

PROCESS_NAME="./mjpg_streamer"
PIDS=$(ps aux | grep "$PROCESS_NAME" | grep -v "grep" | awk '{print $2}')
if [ -z "$PIDS" ]; then
    ./mjpg_streamer -i "./input_file.so -f /home/dlinano/workspace/traffic-light/save -n output.jpg -d 0.01" -o "./output_http.so -w ./www -p 8080" &
else
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