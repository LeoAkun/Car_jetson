
from multi_map_navigation.process_manager.py import ProcessManager
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = ProcessManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    