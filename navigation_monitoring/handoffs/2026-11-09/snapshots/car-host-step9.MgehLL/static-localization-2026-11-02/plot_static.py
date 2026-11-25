"""Render measured point-map overlap and stationary drift, not simulated data."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

OUT = Path(__file__).resolve().parent
result = json.loads((OUT/'map-match-result.json').read_text())
records = json.loads((OUT/'tf-records.json').read_text())
best = result['best']
pose = np.asarray(best['pose'])
message = deserialize_message((OUT/'cloud-10s.cdr').read_bytes(), PointCloud2)
fields = {f.name: f for f in message.fields}
scan = np.column_stack([np.ndarray((message.width*message.height,), dtype='<f4', buffer=message.data,
                       offset=fields[name].offset, strides=(message.point_step,)) for name in ['x', 'y', 'z']])
scan = scan[np.isfinite(scan).all(axis=1)]
scan = scan @ Rotation.from_quat(pose[3:]).as_matrix().T + pose[:3]
map_points = np.load(OUT/'map-neighborhood.npy')
fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
overlap, drift = axes
for points, color, label, size in [(map_points, '#9da6ad', 'Existing 3b2mus map', .7),
                                    (scan, '#1971c2', 'Measured RSHELIOS scan', 1.1)]:
    mask = (np.abs(points[:, 0]-pose[0]) < 28.) & (np.abs(points[:, 1]-pose[1]) < 28.)
    mask &= (points[:, 2] > pose[2]-1.) & (points[:, 2] < pose[2]+3.)
    selected = points[mask]
    if len(selected) > 90000:
        selected = selected[::int(np.ceil(len(selected)/90000))]
    overlap.scatter(selected[:, 0], selected[:, 1], s=size, color=color, alpha=.5, label=label, rasterized=True)
overlap.scatter([pose[0]], [pose[1]], marker='x', s=80, color='#c92a2a', label='LiDAR position')
yaw = Rotation.from_quat(pose[3:]).as_euler('xyz')[2]
overlap.arrow(pose[0], pose[1], np.cos(yaw)*4., np.sin(yaw)*4., width=.12, head_width=1., color='#c92a2a')
overlap.set(xlim=(pose[0]-28.,pose[0]+28.), ylim=(pose[1]-28.,pose[1]+28.),
            xlabel='Map x (m)', ylabel='Map y (m)', title='Real scan vs. 3b2mus map (height-filtered)')
overlap.set_aspect('equal')
overlap.legend(markerscale=1, fontsize=8, loc='lower right')
overlap.grid(alpha=.15)

rows = records['map->base_link']
values = np.asarray([r['pose'] for r in rows])
time = np.asarray([r['elapsed'] for r in rows])
offset = (values[:, :3]-values[0, :3])*100.
for index, color in enumerate(['#1971c2','#e8590c','#2b8a3e']):
    drift.plot(time-time[0], offset[:, index], linewidth=1., color=color, label=['x','y','z'][index])
drift.set(xlabel='Seconds after accepted localization', ylabel='Position change (cm)',
          title='Measured stationary base_link position')
drift.legend(title='Map axes')
drift.grid(alpha=.2)
fig.suptitle('Real vehicle stationary localization — 2026-11-02', fontsize=13)
fig.savefig(OUT/'static-localization.png', dpi=160)
plt.close(fig)
print(OUT/'static-localization.png')
