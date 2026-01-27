#ifndef CANLIB_MSG_H
#define CANLIB_MSG_H


#include "utils.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <stdint-gcc.h>
#include <string.h>
#include <string>
#include "controlcan.h"
#include <yunle_msgs/msg/battery.hpp>
#include <yunle_msgs/msg/vehicle_status.hpp>
#include <yunle_msgs/msg/ecu.hpp>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

#define CAR_TYPE 1


// reveive vehicle status msg protocol
#define CCU_SHIFT_LEVEL_OFFSET 0
#define CCU_SHIFT_LEVEL_LENGTH 2     // 档位  1：D  2：N  3：R
#define CCU_P_STATUS_OFFSET 2
#define CCU_P_STATUS_LENGTH 1
#define CCU_DRIVE_MODE_SHIFT_OFFSET 5
#define CCU_DRIVE_MODE_SHIFT_LENGTH 1

#define CCU_WHEEL_DIRECTION_OFFSET 7 // 实际方向盘角度方向 0左边 1右边
#define CCU_WHEEL_ANGLE_OFFSET 8
#define CCU_WHEEL_ANGLE_LEGNTH 12 // 实际方向盘角度 单位 0.1度  最大值 1200 代表 120度
#define CCU_SPEED_OFFSET 20
#define CCU_SPEED_LENGTH 9 // 当前车速 单位 0.1m/s 最大值 510 代表 51m/s
#define CCU_DRIVE_MODE_OFFSET 29
#define CCU_DRIVE_MODE_LENGTH 3 // 当前驾驶模式 0不影响  1自动驾驶模式  2驾驶员PAD模式  3驾驶员方向盘模式\
                                   自动驾驶模式的优先级低于 方向盘模式 也就是 在自动驾驶模式下，调整控制版，车辆\
                                   优先采取控制版的命令
#define CCU_ACCELERATE_LEVEL_OFFSET 32
#define CCU_ACCELERATE_LEVEL_LENGTH 2 // 当前加速档位
#define CCU_BRAKE_LEVEL_OFFSET 34
#define CCU_BRAKE_LEVEL_LENGTH 2 // 当前减速档位
#define CCU_TOTAL_ODOMETER_OFFSET 36
#define CCU_TOTAL_ODOMETER_LENGTH 20 // 车辆累计里程数 单位：km

// reveive vehicle battery msg protocol
#define BMU_VOLTAGE_OFFSET 0
#define BMU_VOLTAGE_LENGTH 16 // 动力电池 电压 单位：V Physical voltage= BSU_Battery_Out_Voltage * 0.1
#define BMU_AMPERE_OFFSET 16
#define BMU_AMPERE_LENGTH 16 // 动力电池 电流 单位：A Physical current= (BSU_Battery_Out_Current - 4000) * 0.1
#define BMU_CAPACITY_OFFSET 32
#define BMU_CAPACITY_LENGTH 8 // 动力电池 电量 单位：百分比 Physical current= BSU_SysSOC  *  0.4%
#define BMU_SYS_STATUS_OFFSET 40
#define BMU_SYS_STATUS_LENGTH 2 // BSU 系统状态：  0 正常 \
                                                 1 不影响车辆正常行驶的故障\
                                                 2 影响车辆正常行使，需要驾驶员限制驾驶的故障\
                                                 3 驾驶员需要立即停车，并请求救援的故障

#define BMU_CHARGE_STATUS_OFFSET 42 // BSU 充电状态： 0未充电  1正在充电

#define BMU_NEW_VOLTAGE_OFFSET 0
#define BMU_NEW_VOLTAGE_LENGTH 16  // 动力电池 电压V = value * 0.1
#define BMU_NEW_DASHBOARD_OFFSET 16
#define BMU_NEW_DASHBOARD_LENGTH 16 // SOC仪表显示
#define BMU_NEW_AMPERE_OFFSET 32
#define BMU_NEW_AMPERE_LENGTH 16  // 动力电池 电流A = value * 0.1
#define BMU_NEW_SOC_OFFSET 48
#define BMU_NEW_SOC_LENGTH 16  // 动力电池 荷电状态

#if (CAR_TYPE == 1)

// send vehicle control msg protocol
#define SCU_SHIFT_LEVEL_OFFSET 0
#define SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define SCU_ACC_MODE_OFFSET 2
#define SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define SCU_BRAKE_MODE_OFFSET 4
#define SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define SCU_DRIVE_MODE_REQ_OFFSET 6
#define SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define SCU_LMOTOR_SPD_RPM_OFFSET 8
#define SCU_LMOTOR_SPD_RPM_LENGTH 16 // Left wheel moter speed use to change direction
#define SCU_RMOTOR_SPD_RPM_OFFSET 24
#define SCU_RMOTOR_SPD_RPM_LENGTH 16 // Right wheel moter speed use to change direction
#define SCU_LEFT_TURN_LIGHT_REQ_OFFSET 40
#define SCU_LEFT_TURN_LIGHT_REQ_LENGTH 2
#define SCU_RIGHT_TURN_LIGHT_REQ_OFFSET 42
#define SCU_RIGHT_TURN_LIGHT_REQ_LENGTH 2
#define SCU_HAZARD_LIGHT_REQ_OFFSET 44
#define SCU_HAZARD_LIGHT_REQ_LENGTH 2
#define SCU_POSITION_LIGHT_REQ_OFFSET 46
#define SCU_POSITION_LIGHT_REQ_LENGTH 2
#define SCU_LOWBEAM_LIGHT_REQ_OFFSET 48
#define SCU_LOWBEAM_LIGHT_REQ_LENGTH 2
#define SCU_HIGHBEAM_LIGHT_REQ_OFFSET 50
#define SCU_HIGHBEAM_LIGHT_REQ_LENGTH 2
#define SCU_REARFOG_LIGHT_REQ_OFFSET 52
#define SCU_REARFOG_LIGHT_REQ_LENGTH 2
#define SCU_HORN_LIGHT_REQ_OFFSET 54
#define SCU_HORN_LIGHT_REQ_LENGTH 2
#define SCU_DOWN_ACCELERATED_SPEED_OFFSET 56
#define SCU_DOWN_ACCELERATED_SPEED_LENGTH 3 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define SCU_UP_ACCELERATED_SPEED_OFFSET 59
#define SCU_UP_ACCELERATED_SPEED_LENGTH 3 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define SCU_EBRAKE_OFFSET 63
#define SCU_EBRAKE_LENGTH 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动


#elif (CAR_TYPE == 0)

// send vehicle control msg protocol
#define SCU_SHIFT_LEVEL_OFFSET 0
#define SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define SCU_ACC_MODE_OFFSET 2
#define SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define SCU_BRAKE_MODE_OFFSET 4
#define SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define SCU_DRIVE_MODE_REQ_OFFSET 6
#define SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define SCU_STEERING_WHEEL_ANGLE_OFFSET 8
#define SCU_STEERING_WHEEL_ANGLE_LENGTH 16 // 请求方向盘转角信号
#define SCU_TARGET_SPEED_OFFSET 24
#define SCU_TARGET_SPEED_LENGTH 9 // 目标车速请求
#define SCU_EBRAKE_OFFSET 33
#define SCU_EBRAKE_LENGTH 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define SCU_GRADIENT_OFFSET 34
#define SCU_GRADIENT_LENGTH 6 // 道路倾斜坡度设置  -15度～+15度 单位 0.5度

#endif

#define PAYLOAD_LENGTH 8

#define BUFFER_SIZE 8

namespace USB2CAN
{
enum ShiftLevel
{
    NOT_DETECTED = 0,
    D = 1, // 前进
    N = 2, // 停车
    R = 3  // 倒车
};

enum DriveMode
{
    AUTO_MODE = 1,
    PAD_MODE = 2,
    WHEEL_MODE = 3
};

enum SteerDirection
{
    left = 0,
    right = 1
};

class SendMsgBase{
protected:
    // yunle_msgs::msg::ecu ecuMsg;
    yunle_msgs::msg::Ecu ecuMsg;
    VCI_CAN_OBJ canMsg;
    double pre_steer;
    virtual void initCanMsg() {}
    virtual void setDriveMode(DriveMode driveMode) {}
    virtual void setBrakeMode(int v) {}
    virtual void setAccMode(int v) {}
    virtual void setShiftLevel(ShiftLevel shiftLevel) {}
    virtual void setEBrake(bool need = false) {}
    virtual void rosMsg2canMsg() {}
public:
    SendMsgBase() {}

    SendMsgBase(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    SendMsgBase(yunle_msgs::msg::Ecu ecuMsg, double pre_steer)
    {
        this->ecuMsg = ecuMsg;
        this->pre_steer = pre_steer;
    }

    virtual ~SendMsgBase() {}

    virtual VCI_CAN_OBJ getMessage(){VCI_CAN_OBJ obj; return obj;}

    void print();

};

class SendMsg
{
private:
    // uint8_t packet[BUFFER_SIZE];
    yunle_msgs::msg::Ecu ecuMsg;
    VCI_CAN_OBJ canMsg;

    double pre_steer;

    void initCanMsg();

    void setDriveMode(DriveMode driveMode);

    void setBrakeMode(int v);

    void setAccMode(int v);

    void setShiftLevel(ShiftLevel shiftLevel);

    void setSpeed(double v);

    void setWheelAngle(double angle);

    void setDifferentialDrive(double v, double angle);

    void setGradient(double v);

    void setEBrake(bool need = false);

    void rosMsg2canMsg();

public:
    SendMsg()
    {
        initCanMsg();
    }

    SendMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    SendMsg(const yunle_msgs::msg::Ecu& ecuMsg, double pre_steer)
    {
        initCanMsg();

        this->ecuMsg = ecuMsg;
        this->pre_steer = pre_steer;
    }

    void print();

    VCI_CAN_OBJ getMessage();
};

class BatteryMsg
{
private:
    VCI_CAN_OBJ canMsg;
    bool protocol_new;

    yunle_msgs::msg::Battery rosMsg_battery_status;

    double getVoltage();

    double getAmpere();

    double getBatteryCapacity();

    int getBsuSysStatus();

    int getChargeStatus();

    void createMessage();

public:
    BatteryMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
        this->protocol_new = false;
    }

    BatteryMsg(VCI_CAN_OBJ in_canMsg, bool protocol_new)
    {
        this->canMsg = in_canMsg;
        this->protocol_new = protocol_new;
    }

    void print();

    yunle_msgs::msg::Battery getMessage();
};

class VehicleStatusMsg
{
private:
    // uint8_t packet[BUFFER_SIZE];
    VCI_CAN_OBJ canMsg;

    yunle_msgs::msg::VehicleStatus rosMsg_vehicle_status;

    int shiftLevel();

    double speed();

    double wheelAngle();

    SteerDirection wheelDirection();

    bool autodriveFlag();

    bool pStatus();

    int driveMode();

    void createMessage();

public:
    VehicleStatusMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    void print();

    std::string toString();

    yunle_msgs::msg::VehicleStatus getMessage();
};

} // namespace USB2CAN
#endif // !CANLIB_MSG_H
