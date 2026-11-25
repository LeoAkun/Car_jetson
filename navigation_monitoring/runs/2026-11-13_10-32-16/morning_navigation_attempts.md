# Morning navigation attempts

| Task | Startup result | Failure stage | Primary finding |
|---|---|---|---|
| `task_1794537392` | Nav2 did not start | Initial pose / `map -> odom` | Vehicle sensor/status data stayed at zero and pose initialization waited for GPS. LIO-SAM and Nav2 were not reached. |
| `task_1794538015` | Nav2 started and accepted the goal | Global planning | The starting footprint overlapped a runtime lethal-cost area associated with a close obstacle behind the vehicle. |
| `task_1794539326` | Nav2 started and accepted the goal | Execution / recovery | Nav2 issued nonzero commands, but the chassis remained parked and speed stayed zero; the goal was eventually aborted. |
| `task_1794539938` | Nav2 did not start | Initial pose / `map -> odom` | Vehicle sensor/status data stayed at zero and pose initialization waited for GPS. LIO-SAM and Nav2 were not reached. |

The first and fourth submissions have the same startup failure pattern. If only the latest three submissions are counted, only the last one has this cause.
