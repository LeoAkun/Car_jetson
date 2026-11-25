import json
import sys
import time
from pathlib import Path

import cv2

device = sys.argv[1]
out = Path(__file__).parent
cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)
result = {'device': device, 'opened': cap.isOpened()}
times = []
frame = None
try:
    for _ in range(90):
        ok, image = cap.read()
        if not ok:
            result['read_failed'] = True
            break
        times.append(time.monotonic())
        frame = image
finally:
    cap.release()
result['frames'] = len(times)
if len(times) > 1:
    result['received_hz'] = (len(times) - 1) / (times[-1] - times[0])
if frame is not None:
    result.update(shape=list(frame.shape), mean=float(frame.mean()), std=float(frame.std()))
    cv2.imwrite(str(out / (Path(device).name + '.jpg')), frame)
(out / (Path(device).name + '-result.json')).write_text(json.dumps(result, indent=2))
print(json.dumps(result))
