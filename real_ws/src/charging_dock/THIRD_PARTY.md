# Third-party dependencies

The AprilTag detection layer uses the ROS 2 Humble binary packages from the
configured ROS apt repository. They were installed on 2026-08-07 with:

```bash
sudo apt-get install -y \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs \
  ros-humble-image-proc
```

Installed versions:

| Debian package | Version |
| --- | --- |
| `ros-humble-apriltag` | `3.4.5-1jammy.20260226.005100` |
| `ros-humble-apriltag-msgs` | `2.0.2-1jammy.20260605.124532` |
| `ros-humble-apriltag-ros` | `3.4.0-1jammy.20260609.121809` |
| `ros-humble-camera-ros` | `0.7.0-1jammy.20260609.120718` |
| `ros-humble-image-proc` | `3.0.9-1jammy.20260606.044028` |
| `ros-humble-libcamera` | `0.1.0-3jammy.20260226.011158` |

`apriltag_msgs/msg/AprilTagDetection` contains identity, quality, corners and
homography fields. Tag pose is published through TF by this version of
`apriltag_ros`; the project adapter must combine the detection timestamp with
the corresponding TF and publish `charging_dock_msgs` instead of exposing the
third-party message to later control nodes.
