#ifndef MSGNWD_H
#define MSGNWD_H

#include "MsgBase.h"

namespace USB2CAN
{

// send vehicle control msg protocol
#define NWD_SCU_SHIFT_LEVEL_OFFSET 0
#define NWD_SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define NWD_SCU_ACC_MODE_OFFSET 2
#define NWD_SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define NWD_SCU_BRAKE_MODE_OFFSET 4
#define NWD_SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define NWD_SCU_DRIVE_MODE_REQ_OFFSET 6
#define NWD_SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define NWD_SCU_STEERING_WHEEL_ANGLE_OFFSET 8
#define NWD_SCU_STEERING_WHEEL_ANGLE_LENGTH 16 // 请求方向盘转角信号
#define NWD_SCU_TARGET_SPEED_OFFSET 24
#define NWD_SCU_TARGET_SPEED_LENGTH 9 // 目标车速请求
#define NWD_SCU_EBRAKE_OFFSET 33
#define NWD_SCU_EBRAKE_LENGTH 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define NWD_SCU_GRADIENT_OFFSET 34
#define NWD_SCU_GRADIENT_LENGTH 6 // 道路倾斜坡度设置  -15度～+15度 单位 0.5度

// 空车扭矩使能和GRADIENT冲突
#define NWD_SCU_GearNTorque_En_OFFSET 34
#define NWD_SCU_GearNTorque_En_LENGTH 1

class MsgNWD: public SendMsgBase{
protected:
    void initCanMsg();
    void setDriveMode(DriveMode driveMode);
    void setBrakeMode(int v);
    void setAccMode(int v);
    void setShiftLevel(ShiftLevel shiftLevel);
    void setGradient(double v);
    void setGearNTorque_En(int v);
    void setSpeed(double v);
    void setWheelAngle(double angle);
    void setEBrake(bool need = false);
    void rosMsg2canMsg();
public:
    MsgNWD():SendMsgBase()
    {
        initCanMsg();
    }

    MsgNWD(VCI_CAN_OBJ in_canMsg):SendMsgBase(in_canMsg)
    {
    }

    MsgNWD(yunle_msgs::msg::Ecu ecuMsg, double pre_steer):SendMsgBase(ecuMsg, pre_steer)
    {
        initCanMsg();
    }

    ~MsgNWD(){}

    VCI_CAN_OBJ getMessage();
};

}

#endif