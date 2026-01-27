#include <MsgBase.h>

namespace USB2CAN
{

/* Common functions */
static inline void Quaternion2Euler(const geometry_msgs::msg::Quaternion quat_msg, double *roll, double *pitch, double *yaw)
{
    tf2::Quaternion quat;
    // tf2::quaternionMsgToTF(quat_msg, quat);
    tf2::fromMsg(quat_msg, quat);
    tf2::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);
    *roll = *roll * 180/M_PI;
    *pitch = *pitch * 180/M_PI;
    *yaw = *yaw * 180/M_PI;
    // printf("pitch=%.6f\n", *pitch);
}


/* For 发送车身姿态到vcu */
void SendPostureMsg::initImuMsg() {
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x124;
    canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
    canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
    canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
    canMsg.DataLen = 0x08;
}

void SendPostureMsg::setAngle(double angle) {
    int value = int(angle*10);
    writeInt(canMsg.Data, SCU_POSTURE_ANGLE_OFFSET, SCU_POSTURE_ANGLE_OFFSET, value);
}

void SendPostureMsg::setWeight(double weight) {
    int value = int(weight*10);
    writeInt(canMsg.Data, SCU_POSTURE_WEIGHT_OFFSET, SCU_POSTURE_WEIGHT_OFFSET, value);
}

void SendPostureMsg::rosMsg2canMsg() {
    double roll, pitch, yaw;
    Quaternion2Euler(this->imuMsg.orientation, &roll, &pitch, &yaw);
    this->setAngle(-pitch);  // IMU右手系, x前y左z上, 上坡时计算得到pitch为负数(TODO::换IMU验证). 对于vcu来讲, 上坡发送坡度为正值.
    this->setWeight(this->weight);
}

VCI_CAN_OBJ SendPostureMsg::getMessage() {
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}


// -*-*-*-*- Received canMsg: wheel status -*-*-*-*-

void WheelStatus::init() {
    this->torque = _torque();
    this->rpm = _rpm();
    this->dc = _dc();
}

int WheelStatus::_torque() {
    return readAsInt(this->canMsg.Data, CCU_TORQUE_MEASURED_OFFSET, CCU_TORQUE_MEASURED_LENGTH);
}

int WheelStatus::_rpm() {
    int ret = readAsInt(this->canMsg.Data, CCU_RPM_MEASURED_OFFSET, CCU_RPM_MEASURED_LENGTH);
    if (ret > 32767)
        ret -= 65536;
    return ret*this->lr;
}

int WheelStatus::_dc() {
    return readAsInt(this->canMsg.Data, CCU_DC_MEASURED_OFFSET, CCU_DC_MEASURED_LENGTH);
}

int WheelStatus::get_torque() {
    return this->torque;
}

int WheelStatus::get_rpm() {
    return this->rpm;
}

int WheelStatus::get_dc() {
    return this->dc;
}

void WheelSpeed_28x::init(){
    _parse_speed_rpm();
}

int WheelSpeed_28x::_adjust_rpm(int rpm){
    // 判断 RPM 值是否大于 32767
    if (rpm > 32767) {
        // 将 RPM 值转换为有符号整数
        rpm -= 65536;
    }
    // 返回处理后的 RPM 值
    return rpm;
}

void WheelSpeed_28x::_parse_speed_rpm() {
    int speed_rpm  = readAsInt(this->canMsg.Data, SPEED_RPM_28X,  SPEED_RPM_28X_LENGTH);
    speed_rpm = this->_adjust_rpm(speed_rpm);

    //rpm的精度是0.1，
    float factor = 10.0;
    this->speedRpm = speed_rpm / factor;
}

float WheelSpeed_28x::get_speed_rpm(){
    return this->speedRpm;
}

void WheelSpeed::init(){
    _get_speed_rpm();
}

int WheelSpeed::_adjust_rpm(int rpm){
    // 判断 RPM 值是否大于 32767
    if (rpm > 32767) {
        // 将 RPM 值转换为有符号整数
        rpm -= 65536;
    }
    // 返回处理后的 RPM 值
    return rpm;
}

void WheelSpeed::_get_speed_rpm() {
    int ret_front_left  = readAsInt(this->canMsg.Data, FRONT_LEFT_RPM,  FRONT_LEFT_RPM_LENGTH);
    int ret_front_right = readAsInt(this->canMsg.Data, FRONT_RIGHT_RPM, FRONT_RIGHT_RPM_LENGTH);
    int ret_rear_left   = readAsInt(this->canMsg.Data, REAR_LEFT_RPM,   REAR_LEFT_RPM_LENGTH);
    int ret_rear_right  = readAsInt(this->canMsg.Data, REAR_RIGHT_RPM,  REAR_RIGHT_RPM_LENGTH);
    ret_front_left = this->_adjust_rpm(ret_front_left);
    ret_front_right = this->_adjust_rpm(ret_front_right);
    ret_rear_left = this->_adjust_rpm(ret_rear_left);
    ret_rear_right = this->_adjust_rpm(ret_rear_right);

    //rpm的精度是0.1，
    float factor = 10.0;
    this->frontLeftRpm = ret_front_left / factor;
    this->frontRightRpm = ret_front_right / factor;
    this->rearLeftRpm = ret_rear_left / factor;
    this->rearRightRpm = ret_rear_right / factor;
}


float WheelSpeed::get_front_left_rpm(){
    return this->frontLeftRpm;
}

float WheelSpeed::get_front_right_rpm(){
    return this->frontRightRpm;
}

float WheelSpeed::get_rear_left_rpm(){
    return this->rearLeftRpm;
}

float WheelSpeed::get_rear_right_rpm(){
    return this->rearRightRpm;
}


} // end namespace USB2CAN
