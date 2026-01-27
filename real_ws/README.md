# 为了处理包与包之间的依赖关系，需要依次进行构建
```shell
colcon build --packages-select livox_ros2_driver serial
colcon build
```