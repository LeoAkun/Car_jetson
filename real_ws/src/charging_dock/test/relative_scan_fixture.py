# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
from charging_dock.relative_geometry import Pose2


def docking_parameters():
    """Exercise the shipped vehicle geometry, rather than a copied test config."""
    path = Path(__file__).resolve().parents[1] / 'config' / 'relative_docking.yaml'
    return yaml.safe_load(path.read_text())


def scan_faces(stamp, base=Pose2(), sensor=Pose2(), faces=((Pose2(-2.5), .185),),
               n=1440, angle_min=-math.pi, angle_span=2*math.pi):
    inc = angle_span/n
    angles = angle_min+np.arange(n)*inc
    ranges = np.full(n, np.inf)
    local_sensor = base.compose(sensor)
    for dock, width in faces:
        start = local_sensor.relative_to(dock)
        a = angles+local_sensor.yaw-dock.yaw
        dx, dy = np.cos(a), np.sin(a)
        with np.errstate(divide='ignore', invalid='ignore'):
            dist = -start.x/dx
            yy = start.y+dist*dy
        keep = (dist > .06) & (np.abs(yy) <= width/2)
        ranges = np.minimum(ranges, np.where(keep, dist, np.inf))
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
              angle_min=angle_min, angle_increment=inc, range_min=.05,
              range_max=30., ranges=ranges.tolist())
