# 1.可以使用发布/cmd_vel控制底盘
- 当运动时, linear.x为线速度， angular.z为角速度
- 当静止时可以原地打方向, linear.x为0.0， angular.z为转向角度
    - 其中angular.z有最大值OMEGA_FULL_STEER，当达到最大值后即为舵机最大转向角度MAX_STEER_DEG
    - 小于最大值时按照线性比例计算
    - 参见配置文件"car_control/cmdvel_2_ecu/config/config.yaml"

# 2.python发布话题示例
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    # 发布话题接口
    def pub(self, liner, angular):
        msg = Twist()
        msg.linear.x = liner   # 前进速度 0.2 m/s
        msg.angular.z = angular  # 不转弯
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()

    # 发布话题
    node.pub(0.0, 4.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```