#ifndef DYNAMIXELLIB_H
#define DYNAMIXELLIB_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"

using namespace std;
typedef unsigned int uint;

namespace FAR{

// Control table address
const uint8_t ADDR_TORQUE_ENABLE = 64;
const uint8_t ADDR_GOAL_POSITION = 116;
const uint8_t ADDR_GOAL_VELOCITY = 104;
const uint8_t ADDR_GOAL_CURRENT = 102;
const uint8_t ADDR_PRESENT_POSITION = 132;
const uint8_t ADDR_PRESENT_VELOCITY = 128;
const uint8_t ADDR_PRESENT_CURRENT = 126;
const uint8_t ADDR_PRESENT_INPUT_VOLTAGE = 144;
const uint8_t ADDR_OPERATING_MODE = 11;
const uint8_t ADDR_LED = 65;
const uint8_t ADDR_REALTIME_TICK = 120;
const uint8_t ADDR_BAUD_RATE = 8;
const uint8_t ADDR_VELOCITY_LIMIT =	44;
const uint8_t ADDR_CURRRENT_LIMIT =	38;
const uint8_t ADDR_PROFILE_VELOCITY = 112;
const uint8_t ADDR_PROFILE_ACCELERATION = 108;
const uint8_t ADDR_POSITION_P_GAIN = 84;
const uint8_t ADDR_POSITION_I_GAIN = 82;
const uint8_t ADDR_POSITION_D_GAIN = 80;
const uint8_t ADDR_RETURN_DELAY_TIME = 9;
const uint8_t ADDR_DRIVE_MODE = 10;
const uint8_t ADDR_MOVING = 122;
const uint8_t ADDR_MOVING_STATUS = 123;
const uint16_t ADDR_INDIRECTADDRESS_FOR_READ = 168;
const uint16_t ADDR_INDIRECTADDRESS_FOR_WRITE = 578;
const uint16_t ADDR_INDIRECTDATA_FOR_READ = 224;
const uint16_t ADDR_INDIRECTDATA_FOR_WRITE = 634;

// Lenght address
const uint8_t LEN_PRESENT_POSITION = 4;
const uint8_t LEN_PRESENT_VELOCITY = 4;
const uint8_t LEN_PRESENT_CURRENT = 2;
const uint8_t LEN_MOVING = 1;
const uint8_t LEN_MOVING_STATUS = 1;
const uint8_t LEN_TORQUE_ENABLE = 1;
const uint8_t LEN_INDIRECTADDRESS_FOR_READ = LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT + LEN_TORQUE_ENABLE;
const uint8_t LEN_OPERATING_MODE = 1;
const uint8_t LEN_GOAL_POSITION = 4;
const uint8_t LEN_GOAL_CURRENT = 2;
const uint8_t LEN_PROFILE_ACCELERATION = 4;
const uint8_t LEN_PROFILE_VELOCTIY = 4;
const uint8_t LEN_VELOCITY_LIMIT = 4;
const uint8_t LEN_POSITION_P_GAIN = 2;
const uint8_t LEN_INDIRECTADDRESS_FOR_WRITE = LEN_TORQUE_ENABLE + LEN_GOAL_POSITION;

// Protocol version
#define PROTOCOL_VERSION 2.0

//const char DEVICENAME[] = "/dev/ttyUSB0";
//const int32_t BAUDRATE = 4000000;

const uint8_t BR_DF = 1;
const uint8_t BR_1M = 3;
const uint8_t BR_2M	= 4;
const uint8_t BR_4M = 6;
const uint8_t BR_45M = 7;

const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;

const uint8_t DRIVE_MODE_DEFAULT = 0;
const uint8_t DRIVE_MODE_TIME_BASE_PROFILE = 4;

class DxlControl
{
public:
    DxlControl();
    ~DxlControl();

    int dxl_comm_result;// = COMM_TX_FAIL;
    uint8_t dxl_error;// = 0;	// Dynamixel error
    uint8_t single_id;

    bool init(string DEVICENAME, int BAUDRATE);
    int dxl_init(uint8_t ID, uint8_t operating_mode);
    void dxl_deinit(uint8_t ID, int32_t home_pos);
    void dxl_deinit(uint8_t ID);
    void dxl_searching();

    void setLED(uint8_t ID, uint8_t on_off);
    void setGoalPosition(uint8_t ID, int32_t goal_position);
    void setGoalVelocity(uint8_t ID, int32_t goal_velocity);
    void setOperateMode(uint8_t ID, uint8_t mode);
    int setTorqueEnable(uint8_t ID, uint8_t enable);
    void setProfileVelocity(uint8_t ID, uint32_t profile_velocity);
    void setProfileAcceleration(uint8_t ID, uint32_t profile_velocity);

    void getPresentPosition(uint8_t ID, int32_t *present_position_ptr);
    void getPresentVelocity(uint8_t ID, int32_t *present_velocity_ptr);
    void getPresentCurrent(uint8_t ID, int16_t *present_current_ptr);

    void initGroupSyncReadIndirectAddress(uint8_t ID);
    void getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, int8_t *moving, int8_t *moving_status, uint8_t num_joint);
    void getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t num_joint, uint8_t ID);
    void getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t* torque_enabled, uint8_t num_joint);

    void initGroupSyncWriteIndirectAddress(uint8_t ID);
    void setGroupSyncWriteIndirectAddress(const uint32_t *profile_acc, const uint32_t* profile_vel, const uint16_t* pos_p_gain, uint8_t num_joint);
    void setGroupSyncWriteIndirectAddress(uint32_t *profile_acc, uint32_t* profile_vel, const uint16_t* pos_p_gain, uint8_t num_joint);
    void setGroupSyncWriteIndirectAddress(uint8_t *torque_enable, int32_t *goal_position, uint8_t num_joint);
//    void initGroupSyncWirteIndirectAddress6();
//    void setGroupSyncWriteIndirectAddress6(const uint32_t profile_acc, const uint32_t profile_vel, const uint32_t goal_position);

    void setGroupSyncWriteTorqueEnable(uint8_t enable, uint8_t num_joint);
    void setGroupSyncWriteOperatingMode(uint8_t enable, uint8_t num_joint);
    void setGroupSyncWriteGoalPosition(int32_t *goalPosition, uint8_t num_joint);
    void setGroupSyncWriteGoalCurrent(int16_t *goalCurrent, uint8_t num_joint);

    void getGroupSyncReadPresentPosition(int32_t *present_position, uint8_t num_joint);

    uint32_t feeding_profile_acc[6], feeding_profile_vel[6];
    uint16_t feeding_pos_p_gain[6];
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    bool init_flag;
};

const double POSITION_UNIT = 0.088;	// [deg]
const double VELOCITY_UNIT = 0.229;	// [RPM]
const double CURRENT_UNIT = 2.69; // [mA]

enum JointOpMode{ current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };

const double TORQUE_CONSTANT_W150 = 1.65;
const double TORQUE_CONSTANT_V270 = 8.0;
const double TORQUE_CONSTANT_W270 = 3.6;
const double TORQUE_CONSTANT_V350 = 4.7;
const double TORQUE_CONSTANT_W350 = 4.2;

const uint32_t default_vel_limit[6] = {128, 128, 128, 130, 130, 130};
const uint32_t default_profile_acc[6] = {0, 0, 0, 0, 0, 0};
const uint32_t default_profile_vel[6] = {0, 0, 0, 0, 0, 0};
//const uint16_t default_pos_p_gain[6] = {900, 900, 900, 1200, 1200, 1200};
const uint16_t default_pos_p_gain[6] = {1500, 1500, 1500, 3000, 3000, 3000};

const uint32_t init_vel_limit[6] = {128, 128, 128, 130, 130, 130};
const uint32_t init_profile_acc[6] = {0, 0, 0, 0, 0, 0};
const uint32_t init_profile_vel[6] = {0, 0, 0, 0, 0, 0};
const uint16_t init_pos_p_gain[6] = {800, 800, 800, 800, 800, 800};

}

using namespace FAR;

#endif // DYNAMIXELLIB_H
