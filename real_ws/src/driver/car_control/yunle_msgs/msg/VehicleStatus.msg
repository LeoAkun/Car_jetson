std_msgs/Header header

float32 cur_speed  # 当前速度(km/h)
float32 cur_steer # 当前转向角度(°)

float32 left_wheel_speed    # 左轮速度
float32 right_wheel_speed   # 右轮速度

uint8 wheel_direction # 车轮转向方向：左或者右
uint8 shift_level # 当前档位
uint8 drive_mode # 驾驶模式,自动驾驶或遥控驾驶

bool is_autodrive   # 底盘自动驾驶按钮状态
bool p_status       # 底盘刹车状态