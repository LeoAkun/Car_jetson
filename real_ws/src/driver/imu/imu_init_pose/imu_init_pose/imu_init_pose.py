import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np

class ImuResetNode(Node):
    def __init__(self):
        super().__init__('imu_reset_node')
        self.sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu_relative', 10)
        
        self.q_list = []               # 存储前 N 次 orientation
        self.q_ref = None              # 最终参考四元数
        self.num_samples = 10          # 要平均的前 N 次
        self.get_logger().info(f'节点启动，将收集前 {self.num_samples} 次 IMU 姿态取平均作为零位...')

    def imu_callback(self, msg):
        # 收集实时 orientation 四元数 [x, y, z, w]
        q_current = np.array([msg.orientation.x, msg.orientation.y,
                              msg.orientation.z, msg.orientation.w])
        
        if self.q_ref is None:
            # 1. 收集阶段
            self.q_list.append(q_current)
            self.get_logger().info(f'已收集 {len(self.q_list)}/{self.num_samples} 次姿态')
            
            if len(self.q_list) >= self.num_samples:
                # 2. 计算平均值阶段
                q_array = np.array(self.q_list)
                
                # 直接对分量求平均
                avg_q = np.mean(q_array, axis=0)
                
                # 归一化
                norm = np.linalg.norm(avg_q)
                if norm == 0:
                    self.q_ref = np.array([0.0, 0.0, 0.0, 1.0])
                else:
                    self.q_ref = avg_q / norm
                
                self.get_logger().info(f'校准完成，平均零位四元数: {self.q_ref}')
                self.q_list = []  # 清空列表
        else:
            # 3. 发布 & 打印阶段
            
            # --- 准备旋转对象 ---
            r_ref = R.from_quat(self.q_ref)       # 零点姿态
            r_current = R.from_quat(q_current)    # 实时姿态
            
            # --- 计算相对姿态 (Current * Ref_inv) ---
            r_rel = r_current * r_ref.inv()
            q_rel = r_rel.as_quat()

            # --- 计算欧拉角 (Roll, Pitch, Yaw) ---
            # 1. [相对] Euler
            rel_euler = r_rel.as_euler('xyz', degrees=True)
            rel_roll, rel_pitch, rel_yaw = rel_euler
            
            # 2. [实时] Euler (新增)
            curr_euler = r_current.as_euler('xyz', degrees=True)
            curr_roll, curr_pitch, curr_yaw = curr_euler

            # 3. [零点] Euler
            ref_euler = r_ref.as_euler('xyz', degrees=True)
            ref_roll, ref_pitch, ref_yaw = ref_euler

            # --- 发布消息 ---
            new_msg = Imu()
            new_msg.header = msg.header
            new_msg.header.frame_id = 'imu_relative_link'
            
            new_msg.orientation.x = q_rel[0]
            new_msg.orientation.y = q_rel[1]
            new_msg.orientation.z = q_rel[2]
            new_msg.orientation.w = q_rel[3]
            
            # 透传其他数据
            new_msg.orientation_covariance = msg.orientation_covariance
            new_msg.angular_velocity = msg.angular_velocity
            new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            new_msg.linear_acceleration = msg.linear_acceleration
            new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

            self.pub.publish(new_msg)

            # --- 打印详细信息 (分为三组) ---
            self.get_logger().info(
                f'\n'
                f'>>> [相对位姿 Relative] \n'
                f'    Quat : [{q_rel[0]:.3f}, {q_rel[1]:.3f}, {q_rel[2]:.3f}, {q_rel[3]:.3f}]\n'
                f'    Euler: R={rel_roll:.2f}, P={rel_pitch:.2f}, Y={rel_yaw:.2f}\n'
                f'>>> [实时位姿 Real-time] \n'
                f'    Quat : [{q_current[0]:.3f}, {q_current[1]:.3f}, {q_current[2]:.3f}, {q_current[3]:.3f}]\n'
                f'    Euler: R={curr_roll:.2f}, P={curr_pitch:.2f}, Y={curr_yaw:.2f}\n'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ImuResetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()