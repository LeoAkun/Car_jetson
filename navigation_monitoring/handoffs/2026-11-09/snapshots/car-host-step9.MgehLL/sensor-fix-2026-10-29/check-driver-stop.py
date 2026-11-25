import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

out = Path(__file__).parent
results = []
for name, command in [
    ('scan-no-data', ['ros2','run','lakibeam1','lakibeam1_scan_node','--ros-args',
                      '-p','hostip:=127.0.0.1','-p','port:="62368"','-p','frame_id:=stop_test',
                      '-p','output_topic:=/stop_test_scan']),
    ('rslidar-no-data', ['ros2','run','rslidar_sdk','rslidar_sdk_node'])]:
    log = (out/f'{name}.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(2)
    before = time.monotonic()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
    forced = False
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        forced = True
        os.killpg(process.pid,signal.SIGKILL)
        process.wait(timeout=3)
    elapsed = time.monotonic()-before
    log.close()
    result = {'name':name,'exit':process.returncode,'stop_seconds':elapsed,'forced':forced}
    results.append(result)
    print(json.dumps(result),flush=True)
(out/'stop-result.json').write_text(json.dumps(results,indent=2)+'\n')
