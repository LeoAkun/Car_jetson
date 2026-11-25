"""Plot recorded synthetic robot feedback from the task acceptance run."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
result = json.loads((root / 'task-navigation-result.json').read_text())
samples = np.asarray(result['trajectory'])
elapsed = samples[:, 0] - samples[0, 0]
fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
path, speed = axes
path.plot(samples[:, 1], samples[:, 2], color='#1864ab', linewidth=2, label='Synthetic robot feedback')
goals = np.asarray([[0., 0.], [1.6, .6], [3., .6]])
path.plot(goals[:, 0], goals[:, 1], '--', color='#999999', linewidth=1, label='Waypoint order')
path.scatter(goals[:, 0], goals[:, 1], marker='x', s=70, color='#c92a2a', label='Requested waypoints')
for index, (x, y) in enumerate(goals):
    path.annotate(str(index + 1), (x, y), xytext=(6, 8), textcoords='offset points')
end = result['completed_task']['position']
path.scatter(*end, s=45, color='#2b8a3e', label='Completed task position', zorder=5)
path.add_patch(plt.Circle((3., .6), .5, color='#2b8a3e', fill=False, linestyle=':'))
path.set(xlabel='Map x (m)', ylabel='Map y (m)', title='Three waypoints; end error %.3f m' % result['completed_task']['error_m'])
path.axis('equal')
path.grid(alpha=.25)
path.legend(fontsize=8, loc='lower right')
speed.plot(elapsed, samples[:, 4], color='#1864ab', label='Linear velocity (m/s)')
speed.plot(elapsed, samples[:, 5], color='#d9480f', alpha=.7, label='Angular velocity (rad/s)')
speed.axhline(0., linewidth=.8, color='#999999')
speed.set(xlabel='Elapsed time (s)', ylabel='Commanded velocity', title='Completion and injected fault both stop')
speed.grid(alpha=.25)
speed.legend(fontsize=8)
fig.suptitle('Navigation software test — unicycle model, no physical vehicle', fontsize=12)
fig.savefig(root / 'trajectory.png', dpi=160)
plt.close(fig)
print(root / 'trajectory.png')
