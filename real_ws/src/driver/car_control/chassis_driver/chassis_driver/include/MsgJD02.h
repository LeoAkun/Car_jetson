#ifndef MSGJD02_H
#define MSGJD02_H

#include "MsgBase.h"

namespace USB2CAN
{

// send vehicle control msg protocol
#define JD02_SCU_SHIFT_LEVEL_OFFSET 0
#define JD02_SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define JD02_SCU_ACC_MODE_OFFSET 2
#define JD02_SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define JD02_SCU_BRAKE_MODE_OFFSET 4
#define JD02_SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define JD02_SCU_DRIVE_MODE_REQ_OFFSET 6
#define JD02_SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define JD02_SCU_LMOTOR_SPD_RPM_OFFSET 8
#define JD02_SCU_LMOTOR_SPD_RPM_LENGTH 16 // Left wheel moter speed use to change direction
#define JD02_SCU_RMOTOR_SPD_RPM_OFFSET 24
#define JD02_SCU_RMOTOR_SPD_RPM_LENGTH 16 // Right wheel moter speed use to change direction
#define JD02_SCU_LEFT_TURN_LIGHT_REQ_OFFSET 40
#define JD02_SCU_LEFT_TURN_LIGHT_REQ_LENGTH 2
#define JD02_SCU_RIGHT_TURN_LIGHT_REQ_OFFSET 42
#define JD02_SCU_RIGHT_TURN_LIGHT_REQ_LENGTH 2
#define JD02_SCU_HAZARD_LIGHT_REQ_OFFSET 44
#define JD02_SCU_HAZARD_LIGHT_REQ_LENGTH 2
#define JD02_SCU_POSITION_LIGHT_REQ_OFFSET 46
#define JD02_SCU_POSITION_LIGHT_REQ_LENGTH 2
#define JD02_SCU_LOWBEAM_LIGHT_REQ_OFFSET 48
#define JD02_SCU_LOWBEAM_LIGHT_REQ_LENGTH 2
#define JD02_SCU_HIGHBEAM_LIGHT_REQ_OFFSET 50
#define JD02_SCU_HIGHBEAM_LIGHT_REQ_LENGTH 2
#define JD02_SCU_REARFOG_LIGHT_REQ_OFFSET 52
#define JD02_SCU_REARFOG_LIGHT_REQ_LENGTH 2
#define JD02_SCU_HORN_LIGHT_REQ_OFFSET 54
#define JD02_SCU_HORN_LIGHT_REQ_LENGTH 2
#define JD02_SCU_DOWN_ACCELERATED_SPEED_OFFSET 56
#define JD02_SCU_DOWN_ACCELERATED_SPEED_LENGTH 3 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define JD02_SCU_UP_ACCELERATED_SPEED_OFFSET 59
#define JD02_SCU_UP_ACCELERATED_SPEED_LENGTH 3 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define JD02_SCU_EBRAKE_OFFSET 63
#define JD02_SCU_EBRAKE_LENGTH 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动

class MsgJD02: public SendMsgBase{
protected:
    void initCanMsg();
    void setDriveMode(DriveMode driveMode);
    void setBrakeMode(int v);
    void setAccMode(int v);
    void setShiftLevel(ShiftLevel shiftLevel);
    void setEBrake(bool need = false);
    void rosMsg2canMsg();
    void setDifferentialDrive(double v, double angle);
public:
    MsgJD02():SendMsgBase()
    {
        initCanMsg();
    }

    MsgJD02(VCI_CAN_OBJ in_canMsg):SendMsgBase(in_canMsg){}

    MsgJD02(yunle_msgs::msg::Ecu ecuMsg, double pre_steer):SendMsgBase(ecuMsg, pre_steer)
    {
        initCanMsg();
    }

    ~MsgJD02(){}
    
    VCI_CAN_OBJ getMessage();
};


}

#endif