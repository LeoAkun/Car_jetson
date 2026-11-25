#!/bin/bash

cd /home/dlinano/workspace/traffic-light
./app_update.sh
/home/dlinano/miniforge3/envs/TensorRT/bin/python app_multiprocessing.py &
