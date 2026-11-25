import json
from pathlib import Path
import sys
import time

import cv2

cv2.setNumThreads(1)
out=Path(__file__).resolve().parent
device=sys.argv[1]
camera=cv2.VideoCapture(device,cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
camera.set(cv2.CAP_PROP_FPS,30)
result={'device':device,'opened':camera.isOpened()}
times=[]
image=None
try:
    deadline=time.monotonic()+40
    while camera.isOpened() and time.monotonic()<deadline:
        ok,image=camera.read()
        if not ok:
            result['read_failed']=True
            break
        times.append(time.monotonic())
finally:
    camera.release()
result['frames']=len(times)
if len(times)>1:
    result['received_hz']=(len(times)-1)/(times[-1]-times[0])
    result['max_gap_seconds']=max(b-a for a,b in zip(times,times[1:]))
if image is not None:
    result.update(shape=list(image.shape),mean=float(image.mean()),std=float(image.std()))
    cv2.imwrite(str(out/f'{Path(device).name}.jpg'),image)
(out/f'{Path(device).name}-result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
