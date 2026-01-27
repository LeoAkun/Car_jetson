#include "MsgNWD.h"


namespace USB2CAN
{
    void MsgNWD::initCanMsg(){
        bzero(canMsg.Data, BUFFER_SIZE);
        canMsg.ID = 0x120;
        canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
        canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
        canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
        canMsg.DataLen = 0x08;

        this->setGradient(0);          // 设置路面坡度
        this->setGearNTorque_En(1);    // workaround使能空档扭矩: 注意，和setGradient冲突, Gradient应该是已经废弃不用了
        this->setAccMode(1);           // 设置加速档位
        this->setDriveMode(AUTO_MODE); // 设置驾驶模式 ： 自动驾驶
        this->setBrakeMode(1);         // 设置减速档位
    }

    void MsgNWD::setDriveMode(DriveMode driveMode){
        writeInt(canMsg.Data, NWD_SCU_DRIVE_MODE_REQ_OFFSET, NWD_SCU_DRIVE_MODE_REQ_LENGTH, driveMode);
    }

    void MsgNWD::setBrakeMode(int v){
        writeInt(canMsg.Data, NWD_SCU_BRAKE_MODE_OFFSET, NWD_SCU_BRAKE_MODE_LENGTH, v);
    }

    void MsgNWD::setAccMode(int v){
        writeInt(canMsg.Data, NWD_SCU_ACC_MODE_OFFSET, NWD_SCU_ACC_MODE_LENGTH, v);
    }

    void MsgNWD::setShiftLevel(ShiftLevel shiftLevel){
        writeInt(canMsg.Data, NWD_SCU_SHIFT_LEVEL_OFFSET, NWD_SCU_SHIFT_LEVEL_LENGTH, shiftLevel);
    }

    void MsgNWD::setGearNTorque_En(int enable){
        writeInt(canMsg.Data, NWD_SCU_GearNTorque_En_LENGTH, NWD_SCU_GearNTorque_En_LENGTH, enable);
    }

    void MsgNWD::setEBrake(bool need){
        int v = 0;
        if (need)
        {
            v = 1;
        }

        writeInt(canMsg.Data, NWD_SCU_EBRAKE_OFFSET, NWD_SCU_EBRAKE_LENGTH, v);
    }

    void MsgNWD::rosMsg2canMsg(){
        this->setBrakeMode(ecuMsg.brake);
        this->setShiftLevel(ShiftLevel(ecuMsg.shift));
        this->setSpeed(ecuMsg.motor);
        this->setWheelAngle((ecuMsg.steer + this->pre_steer) / 2);
        this->setEBrake(ecuMsg.brake);
    }

    void MsgNWD::setGradient(double v){
        int iv = v * 2; // 因为can里面gradient的单位是0.5度
        writeInt(canMsg.Data, NWD_SCU_GRADIENT_OFFSET, NWD_SCU_GRADIENT_LENGTH, iv);
    }

    void MsgNWD::setSpeed(double v){
        int speed = (int)lround(v * 10);
        writeInt(canMsg.Data, NWD_SCU_TARGET_SPEED_OFFSET, NWD_SCU_TARGET_SPEED_LENGTH, speed);
    }

    void MsgNWD::setWheelAngle(double angle){
        // 120对应27°前轮转角, 即实际ratio = 120 / 27 = 4.444
        double _v = angle * 4.4444444;
        if (_v > 120) _v = 120.0;
        if (_v < -120) _v = -120.0;
        // short v = angle * 10; // 分辨率0.1,转换成整数
        short v = _v * 10; // 分辨率1,转换成整数
        writeInt(canMsg.Data, NWD_SCU_STEERING_WHEEL_ANGLE_OFFSET, NWD_SCU_STEERING_WHEEL_ANGLE_LENGTH, v);
    }

    VCI_CAN_OBJ MsgNWD::getMessage(){
        this->rosMsg2canMsg();
        VCI_CAN_OBJ return_msg = this->canMsg;
        return return_msg;
    }

}
