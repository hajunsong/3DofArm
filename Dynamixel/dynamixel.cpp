#include "dynamixel.h"

DxlControl::DxlControl()
{
    dxl_comm_result = COMM_TX_FAIL;
    dxl_error = 0;
    init_flag = false;
    single_id = 99;
}

DxlControl::~DxlControl()
{
    if(init_flag){
//        groupSyncRead->clearParam();
//        printf("groupSyncRead cleared param\n");
//        delete groupSyncRead;
//        printf("Deleted groupSyncRead\n");

//        groupSyncWrite->clearParam();
//        printf("groupSyncWrite cleared param\n");
//        delete groupSyncWrite;
//        printf("Deleted groupSyncWrite\n");

//        groupSyncWriteTorqueEnable->clearParam();
//        printf("groupSyncWriteTorqueEnable cleared param\n");
//        delete groupSyncWriteTorqueEnable;
//        printf("Deleted groupSyncWriteTorqueEnable\n");

//        groupSyncWritePresentPosition->clearParam();
//        printf("groupSyncWritePresentPosition cleared param\n");
//        delete groupSyncWritePresentPosition;
//        printf("Deleted groupSyncWritePresentPosition\n");

//        groupSyncWritePresentCurrent->clearParam();
//        printf("groupSyncWritePresentCurrent cleared param\n");
//        delete groupSyncWritePresentCurrent;
//        printf("Deleted groupSyncWritePresentCurrent\n");

        // Close port
        portHandler->closePort();
        printf("Closed port\n");
    }
}

bool DxlControl::init(string DEVICENAME, int BAUDRATE) {

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        return false;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        return false;
    }

    // Initialize Groupsyncread instance
//    groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTADDRESS_FOR_READ);

    // Initialize GroupSyncWrite instance
//    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_WRITE, LEN_INDIRECTADDRESS_FOR_WRITE);
//    groupSyncWriteTorqueEnable = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
//    groupSyncWritePresentPosition = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
//    groupSyncWritePresentCurrent = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    init_flag = true;
    return true;
}

int DxlControl::dxl_init(uint8_t ID, uint8_t operating_mode)
{
    // Check Dynamixel Torque on or off
    uint8_t torque = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, &torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("[Check torque] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("[Check torque] %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }

    printf("torque enable : %d\n", torque);
    if (torque == TORQUE_ENABLE)
    {
        // Disable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("[Torque Disable] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error != 0) {
            printf("[Torque Disable] %s\n", packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
    }

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("[LED on] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("[LED on] %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }

//    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_DRIVE_MODE, DRIVE_MODE_TIME_BASE_PROFILE, &dxl_error);
//    if (dxl_comm_result != COMM_SUCCESS) {
//        printf("[LED on] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        return 0;
//    }
//    else if (dxl_error != 0) {
//        printf("[LED on] %s\n", packetHandler->getRxPacketError(dxl_error));
//        return 0;
//    }

    // Write Dynamixel Operating mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, operating_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("[Operating mode] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("[Operating mode] %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    printf("[ID:%03d] operate to ", ID);
    switch(operating_mode){
        case JointOpMode::current_mode:
            printf("current mode\n");
            break;
        case JointOpMode::position_mode:
            printf("position mode\n");
            break;
        case JointOpMode::velocity_mode:
            printf("velocity mode\n");
            break;
        case JointOpMode::extended_position_mode:
            printf("extended position mode\n");
            break;
        case JointOpMode::current_based_position_mode:
            printf("current based position mode\n");
            break;
        default:
            break;
    }

//    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_VELOCITY_LIMIT, init_vel_limit[ID], &dxl_error); // df : 300

//    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_ACCELERATION, 1, &dxl_error); // df : 0
//    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_VELOCITY, 3, &dxl_error); // df : 0
//    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_P_GAIN, init_pos_p_gain[ID], &dxl_error); // df : 800

//    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_CURRRENT_LIMIT, default_cur_limit[ID], &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_RETURN_DELAY_TIME, 20, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("[Return delay time] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("[Return delay time] %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }

    // Enable Dynamixel Torque
    //	packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

//    bool dxl_addparam_result = false;	// addParam result

    // Add parameter storage
//    dxl_addparam_result = groupSyncRead->addParam(ID);
//    if (dxl_addparam_result != true)
//    {
////        fprintf(stderr, "[ID:%03d] groupSyncRead add param failed", ID);
//        printf("[ID:%03d] groupSyncRead add param failed", ID);
//    }

    printf("Dynamixel(ID : %d) has been successfully connected\n", ID);

    return 1;
}

void DxlControl::dxl_deinit(uint8_t ID, int32_t home_pos)
{
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 0, &dxl_error);
    // Reset home position
    int32_t pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(home_pos), &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Goal pos : %d, Current pos : %d\n", home_pos, pos);
    } while (abs(home_pos - pos) > 200);
    // Disable Dynamixel Torque
//    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::dxl_deinit(uint8_t ID){
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, 0, &dxl_error);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::dxl_searching()
{
    for(uint8_t i = 0; i < 10; i++){
        // Check Dynamixel Torque on or off
        uint8_t torque = 0;
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, &torque, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("[Check torque] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("[Check torque] %s\n", packetHandler->getRxPacketError(dxl_error));
        }

        printf("dxl_comm_result : %d\n", dxl_comm_result);
        if (dxl_comm_result == COMM_SUCCESS)
        {
            single_id = i;
            printf("Dynamixel search complete...[ID:%03d]\n", single_id);
            break;
        }
    }
}

void DxlControl::setLED(uint8_t ID, uint8_t on_off)
{
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED, on_off, &dxl_error);
}

void DxlControl::setGoalPosition(uint8_t ID, int32_t goal_position)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void DxlControl::setGoalVelocity(uint8_t ID, int32_t goal_velocity)
{
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_VELOCITY, static_cast<uint32_t>(goal_velocity), &dxl_error);
}

void DxlControl::setProfileVelocity(uint8_t ID, uint32_t profile_velocity){
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_VELOCITY, profile_velocity, &dxl_error);
}

void DxlControl::setProfileAcceleration(uint8_t ID, uint32_t profile_acceleration){
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_ACCELERATION, profile_acceleration, &dxl_error);
}

void DxlControl::setOperateMode(uint8_t mode, uint8_t ID)
{
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, mode, &dxl_error);

    // Reset home position
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

int DxlControl::setTorqueEnable(uint8_t ID, uint8_t enable)
{
    // Change State Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, enable, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 1;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return 1;
    }
    return 0;
}

void DxlControl::getPresentPosition(uint8_t ID, int32_t* present_position_ptr)
{
    int32_t present_position = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    // return present_position/* * 360.0 / RESOLUTION * M_PI/180.0*/;
    *present_position_ptr = present_position;
}

void DxlControl::getPresentVelocity(uint8_t ID, int32_t* present_velocity_ptr)
{
    int32_t present_velocity = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_VELOCITY, reinterpret_cast<uint32_t*>(&present_velocity), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    //    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
    *present_velocity_ptr = present_velocity;
}

void DxlControl::getPresentCurrent(uint8_t ID, int16_t*  present_current_ptr)
{
    int16_t present_current = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_CURRENT, reinterpret_cast<uint16_t*>(&present_current), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    *present_current_ptr = present_current;
}

void DxlControl::setGroupSyncWriteTorqueEnable(uint8_t enable, uint8_t num_joint)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE);
    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        // Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(num_joint == 1 ? single_id : i, &enable);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", num_joint == 1 ? single_id : i);
            return;
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::setGroupSyncWriteOperatingMode(uint8_t mode, uint8_t num_joint)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_OPERATING_MODE, LEN_OPERATING_MODE);
    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        // Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(num_joint == 1 ? single_id : i, &mode);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", num_joint == 1 ? single_id : i);
            return;
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::setGroupSyncWriteGoalPosition(int32_t *goalPosition, uint8_t num_joint)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        uint8_t param_goal_position[LEN_GOAL_POSITION];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPosition[i]));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPosition[i]));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPosition[i]));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPosition[i]));

        // Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(num_joint == 1 ? single_id : i, param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", num_joint == 1 ? single_id : i);
            return;
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::setGroupSyncWriteGoalCurrent(int16_t *goalCurrent, uint8_t num_joint)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);
    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        uint8_t param_goal_current[LEN_GOAL_CURRENT];
        param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD(goalCurrent[i]));
        param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD(goalCurrent[i]));

        // Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(num_joint == 1 ? single_id : i, param_goal_current);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
            return;
        }
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::getGroupSyncReadPresentPosition(int32_t *present_position, uint8_t num_joint)
{
    dynamixel::GroupSyncRead groupSyncReadPresentPosition(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    bool dxl_addparam_result = false;	// addParam result
    // Add parameter storage for Dynamixel#1 present position value

    for (uint8_t i = 0; i < num_joint; i++) {
        dxl_addparam_result = groupSyncReadPresentPosition.addParam(num_joint == 1 ? single_id : i);
        if (dxl_addparam_result != true)
        {
            printf("[ID:%03d] groupSyncRead add param failed\n", num_joint == 1 ? single_id : i);
            return;
        }
    }

    bool dxl_getdata_result = false;	// GetParam result

    // Syncread present position
    dxl_comm_result = groupSyncReadPresentPosition.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else {
        for (uint8_t i = 0; i < num_joint; i++) {
            if (groupSyncReadPresentPosition.getError(num_joint == 1 ? single_id : i, &dxl_error)) {
                printf("[ID:%03d] %s\n", num_joint == 1 ? single_id : i, packetHandler->getRxPacketError(dxl_error));
            }
        }
    }

    // Check if groupsyncread data of Dynamixel#n is available
    for (uint8_t i = 0; i < num_joint; i++) {
        dxl_getdata_result = groupSyncReadPresentPosition.isAvailable(num_joint == 1 ? single_id : i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            printf("[ID:%03d] groupSyncRead getdata failed\n", num_joint == 1 ? single_id : i);
            return;
        }
    }

    // Get Dynamixel#n present position value
    for (uint8_t i = 0; i < num_joint; i++) {
        present_position[i] = static_cast<int32_t>(groupSyncReadPresentPosition.getData(num_joint == 1 ? single_id : i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION));
        printf("[ID:%03d] PresPos:%03d\n", num_joint == 1 ? single_id : i, present_position[i]);
    }

    groupSyncReadPresentPosition.clearParam();
}


void DxlControl::initGroupSyncWriteIndirectAddress(uint8_t ID){
    // INDIRECTDATA parameter storages replace profile_acceleration, profile_velocity, velocity_limit, position_p_gain
    uint16_t indx = 0;
    for(uint8_t i = 0; i < LEN_PROFILE_ACCELERATION; i++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_PROFILE_ACCELERATION + i, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }

    for(uint8_t i = 0; i < LEN_PROFILE_VELOCTIY; i++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_PROFILE_VELOCITY + i, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }

//    for(uint8_t i = 0; i < LEN_VELOCITY_LIMIT; i++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_VELOCITY_LIMIT + i, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

    for(uint8_t i = 0; i < LEN_POSITION_P_GAIN; i++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_POSITION_P_GAIN + i, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }
}

void DxlControl::setGroupSyncWriteIndirectAddress(const uint32_t *profile_acc, const uint32_t* profile_vel, const uint16_t* pos_p_gain, uint8_t num_joint){

    // Syncwrite present data from indirectdata
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_WRITE, LEN_INDIRECTADDRESS_FOR_WRITE);

    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        // Allocate LED and goal position value into byte array
        uint8_t param_indirect_sync_write[LEN_INDIRECTADDRESS_FOR_WRITE];
        param_indirect_sync_write[0] = DXL_LOBYTE(DXL_LOWORD(profile_acc[i]));
        param_indirect_sync_write[1] = DXL_HIBYTE(DXL_LOWORD(profile_acc[i]));
        param_indirect_sync_write[2] = DXL_LOBYTE(DXL_HIWORD(profile_acc[i]));
        param_indirect_sync_write[3] = DXL_HIBYTE(DXL_HIWORD(profile_acc[i]));
        param_indirect_sync_write[4] = DXL_LOBYTE(DXL_LOWORD(profile_vel[i]));
        param_indirect_sync_write[5] = DXL_HIBYTE(DXL_LOWORD(profile_vel[i]));
        param_indirect_sync_write[6] = DXL_LOBYTE(DXL_HIWORD(profile_vel[i]));
        param_indirect_sync_write[7] = DXL_HIBYTE(DXL_HIWORD(profile_vel[i]));
        param_indirect_sync_write[8] = DXL_LOBYTE(DXL_LOWORD(pos_p_gain[i]));
        param_indirect_sync_write[9] = DXL_HIBYTE(DXL_LOWORD(pos_p_gain[i]));

        // Add values to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(i, param_indirect_sync_write);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
            return;
        }
    }

    // Syncwrite all
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::setGroupSyncWriteIndirectAddress(uint32_t *profile_acc, uint32_t* profile_vel, const uint16_t* pos_p_gain, uint8_t num_joint){

    // Syncwrite present data from indirectdata
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_WRITE, LEN_INDIRECTADDRESS_FOR_WRITE);

    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        // Allocate LED and goal position value into byte array
        uint8_t param_indirect_sync_write[LEN_INDIRECTADDRESS_FOR_WRITE];
        param_indirect_sync_write[0] = DXL_LOBYTE(DXL_LOWORD(profile_acc[i]));
        param_indirect_sync_write[1] = DXL_HIBYTE(DXL_LOWORD(profile_acc[i]));
        param_indirect_sync_write[2] = DXL_LOBYTE(DXL_HIWORD(profile_acc[i]));
        param_indirect_sync_write[3] = DXL_HIBYTE(DXL_HIWORD(profile_acc[i]));
        param_indirect_sync_write[4] = DXL_LOBYTE(DXL_LOWORD(profile_vel[i]));
        param_indirect_sync_write[5] = DXL_HIBYTE(DXL_LOWORD(profile_vel[i]));
        param_indirect_sync_write[6] = DXL_LOBYTE(DXL_HIWORD(profile_vel[i]));
        param_indirect_sync_write[7] = DXL_HIBYTE(DXL_HIWORD(profile_vel[i]));
        param_indirect_sync_write[8] = DXL_LOBYTE(DXL_LOWORD(pos_p_gain[i]));
        param_indirect_sync_write[9] = DXL_HIBYTE(DXL_LOWORD(pos_p_gain[i]));

        // Add values to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(i, param_indirect_sync_write);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
            return;
        }
    }

    // Syncwrite all
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

void DxlControl::setGroupSyncWriteIndirectAddress(uint8_t *torque_enable, int32_t *goal_position, uint8_t num_joint)
{
    // Syncwrite present data from indirectdata
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_WRITE, LEN_INDIRECTADDRESS_FOR_WRITE);

    bool dxl_addparam_result = false;	// addParam result

    for (uint8_t i = 0; i < num_joint; i++) {
        // Allocate LED and goal position value into byte array
        uint8_t param_indirect_sync_write[LEN_INDIRECTADDRESS_FOR_WRITE];
        param_indirect_sync_write[0] = torque_enable[i];
        param_indirect_sync_write[1] = DXL_LOBYTE(DXL_LOWORD(goal_position[i]));
        param_indirect_sync_write[2] = DXL_HIBYTE(DXL_LOWORD(goal_position[i]));
        param_indirect_sync_write[3] = DXL_LOBYTE(DXL_HIWORD(goal_position[i]));
        param_indirect_sync_write[4] = DXL_HIBYTE(DXL_HIWORD(goal_position[i]));

        // Add values to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(i, param_indirect_sync_write);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
            return;
        }
    }

    // Syncwrite all
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
}

//void DxlControl::initGroupSyncWirteIndirectAddress6()
//{
//    // INDIRECTDATA parameter storages replace profile_acceleration, profile_velocity, goal_position
//    uint8_t ID = 5;
//    uint16_t indx = 0;
//    for(uint8_t i = 0; i < LEN_PROFILE_ACCELERATION; i++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_PROFILE_ACCELERATION + i, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint8_t i = 0; i < LEN_PROFILE_VELOCTIY; i++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_PROFILE_VELOCITY + i, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint8_t i = 0; i < LEN_GOAL_POSITION; i++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_WRITE + indx, ADDR_GOAL_POSITION + i, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }
//}

//void DxlControl::setGroupSyncWriteIndirectAddress6(const uint32_t profile_acc, const uint32_t profile_vel, const uint32_t goal_position)
//{
//    // Syncwrite present data from indirectdata
//    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_WRITE, LEN_INDIRECTADDRESS_FOR_WRITE);

//    bool dxl_addparam_result = false;	// addParam result

//    for (uint8_t i = 0; i < num_joint; i++) {
//        // Allocate LED and goal position value into byte array
//        uint8_t param_indirect_sync_write[LEN_INDIRECTADDRESS_FOR_WRITE];
//        param_indirect_sync_write[0] = DXL_LOBYTE(DXL_LOWORD(profile_acc[i]));
//        param_indirect_sync_write[1] = DXL_HIBYTE(DXL_LOWORD(profile_acc[i]));
//        param_indirect_sync_write[2] = DXL_LOBYTE(DXL_HIWORD(profile_acc[i]));
//        param_indirect_sync_write[3] = DXL_HIBYTE(DXL_HIWORD(profile_acc[i]));
//        param_indirect_sync_write[4] = DXL_LOBYTE(DXL_LOWORD(profile_vel[i]));
//        param_indirect_sync_write[5] = DXL_HIBYTE(DXL_LOWORD(profile_vel[i]));
//        param_indirect_sync_write[6] = DXL_LOBYTE(DXL_HIWORD(profile_vel[i]));
//        param_indirect_sync_write[7] = DXL_HIBYTE(DXL_HIWORD(profile_vel[i]));
//        param_indirect_sync_write[8] = DXL_LOBYTE(DXL_LOWORD(vel_limit[i]));
//        param_indirect_sync_write[9] = DXL_HIBYTE(DXL_LOWORD(vel_limit[i]));
//        param_indirect_sync_write[10] = DXL_LOBYTE(DXL_HIWORD(vel_limit[i]));
//        param_indirect_sync_write[11] = DXL_HIBYTE(DXL_HIWORD(vel_limit[i]));
//        param_indirect_sync_write[12] = DXL_LOBYTE(DXL_LOWORD(pos_p_gain[i]));
//        param_indirect_sync_write[13] = DXL_HIBYTE(DXL_LOWORD(pos_p_gain[i]));

//        // Add values to the Syncwrite storage
//        dxl_addparam_result = groupSyncWrite.addParam(i, param_indirect_sync_write);
//        if (dxl_addparam_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncWrite add param failed", i);
//            return;
//        }
//    }

//    // Syncwrite all
//    dxl_comm_result = groupSyncWrite.txPacket();
//    if (dxl_comm_result != COMM_SUCCESS)
//    {
//        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//    }

//    // Clear syncwrite parameter storage
//    groupSyncWrite.clearParam();
//}

//void DxlControl::initGroupSyncReadIndirectAddress(uint8_t ID){
//    // INDIRECTDATA parameter storages replace present position, present velocity, present current, moving, moving status
//    uint16_t indx = 0;
//    for(uint16_t j = 0; j < LEN_PRESENT_POSITION; j++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_POSITION + j, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("[Indirect address (present position)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("[Indirect address (present position)] %s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint16_t j = 0; j < LEN_PRESENT_VELOCITY; j++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_VELOCITY + j, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("[Indirect address (present velocity)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("[Indirect address (present velocity)] %s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint16_t j = 0; j < LEN_PRESENT_CURRENT; j++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_CURRENT + j, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("[Indirect address (present current)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("[Indirect address (present current)] %s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint16_t j = 0; j < LEN_MOVING; j++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_MOVING, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("[Indirect address (moving)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("[Indirect address (moving)] %s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }

//    for(uint16_t j = 0; j < LEN_MOVING_STATUS; j++){
//        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_MOVING_STATUS + j, &dxl_error);
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("[Indirect address (moving status)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("[Indirect address (moving status)] %s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//        indx += 2;
//    }
//}

void DxlControl::initGroupSyncReadIndirectAddress(uint8_t ID){
    // INDIRECTDATA parameter storages replace present position, present velocity, present current, moving, moving status
    uint16_t indx = 0;
    for(uint16_t j = 0; j < LEN_PRESENT_POSITION; j++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_POSITION + j, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[Indirect address (present position)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("[Indirect address (present position)] %s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }

    for(uint16_t j = 0; j < LEN_PRESENT_VELOCITY; j++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_VELOCITY + j, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[Indirect address (present velocity)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("[Indirect address (present velocity)] %s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }

    for(uint16_t j = 0; j < LEN_PRESENT_CURRENT; j++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_PRESENT_CURRENT + j, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[Indirect address (present current)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("[Indirect address (present current)] %s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }

    for(uint16_t j = 0; j < LEN_TORQUE_ENABLE; j++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_INDIRECTADDRESS_FOR_READ + indx, ADDR_TORQUE_ENABLE + j, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("[Indirect address (present current)] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("[Indirect address (present current)] %s\n", packetHandler->getRxPacketError(dxl_error));
        }
        indx += 2;
    }
}


void DxlControl::getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t* torque_enable, uint8_t num_joint){
    // Syncread present data from indirectdata
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTADDRESS_FOR_READ);

    bool dxl_addparam_result = false;	// addParam result

    for(uint8_t ID = 0; ID < num_joint; ID++){
        // Add parameter storage
        dxl_addparam_result = groupSyncRead.addParam(ID);
        if (dxl_addparam_result != true)
        {
            printf("[ID:%03d] groupSyncRead add param failed", ID);
        }
    }

    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return;
    }

    for (uint8_t ID = 0; ID < num_joint; ID++){
        // Check if groupsyncread data of Dyanamixel is available
        int dxl_getdata_result = 0;
        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
        }

        // Check if groupsyncread data of Dyanamixel is available
        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
        }

        // Check if groupsyncread data of Dyanamixel is available
        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
        }

        // Check if groupsyncread data of Dyanamixel is available
        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT, LEN_TORQUE_ENABLE);
        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
        }
    }

    for (uint8_t ID = 0; ID < num_joint; ID++){
        // Get Dynamixel present position value
        present_position[ID] = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION));

        // Get Dynamixel present velocity value
        present_velocity[ID] = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY));

        // Get Dynamixel present current value
        present_current[ID] = static_cast<int16_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT));

        // Get Dynamixel present torque enable
        torque_enable[ID] = static_cast<uint8_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT, LEN_TORQUE_ENABLE));
    }

    groupSyncRead.clearParam();
}

//void DxlControl::getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, int8_t *moving, int8_t *moving_status, uint8_t num_joint){
//    // Syncread present data from indirectdata
//    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTADDRESS_FOR_READ);

//    bool dxl_addparam_result = false;	// addParam result

//    for(uint8_t ID = 0; ID < num_joint; ID++){
//        // Add parameter storage
//        dxl_addparam_result = groupSyncRead.addParam(ID);
//        if (dxl_addparam_result != true)
//        {
//            printf("[ID:%03d] groupSyncRead add param failed", ID);
//        }
//    }

//    dxl_comm_result = groupSyncRead.txRxPacket();
//    if (dxl_comm_result != COMM_SUCCESS) {
//        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        return;
//    }

//    for (uint8_t ID = 0; ID < num_joint; ID++){
//        // Check if groupsyncread data of Dyanamixel is available
//        int dxl_getdata_result = 0;
//        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION);
//        if (dxl_getdata_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
//        }

//        // Check if groupsyncread data of Dyanamixel is available
//        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY);
//        if (dxl_getdata_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
//        }

//        // Check if groupsyncread data of Dyanamixel is available
//        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT);
//        if (dxl_getdata_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
//        }

//        // Check if groupsyncread data of Dyanamixel is available
//        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT, LEN_MOVING);
//        if (dxl_getdata_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
//        }

//        // Check if groupsyncread data of Dyanamixel is available
//        dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT + LEN_MOVING, LEN_MOVING_STATUS);
//        if (dxl_getdata_result != true)
//        {
//            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", ID);
//        }
//    }

//    for (uint8_t ID = 0; ID < num_joint; ID++){
//        // Get Dynamixel present position value
//        present_position[ID] = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION));

//        // Get Dynamixel present velocity value
//        present_velocity[ID] = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY));

//        // Get Dynamixel present current value
//        present_current[ID] = static_cast<int16_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT));

//        // Get Dynamixel moving value
//        moving[ID] = static_cast<int8_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT, LEN_MOVING));

//        // Get Dynamixel moving status value
//        moving_status[ID] = static_cast<int8_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT + LEN_MOVING, LEN_MOVING_STATUS));
//    }

//    groupSyncRead.clearParam();
//}

void DxlControl::getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t num_joint, uint8_t ID){
	// Syncread present data from indirectdata
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_INDIRECTDATA_FOR_READ, LEN_INDIRECTADDRESS_FOR_READ);

	bool dxl_addparam_result = false;	// addParam result

//	for(uint8_t ID = 0; ID < num_joint; ID++){
		// Add parameter storage
		dxl_addparam_result = groupSyncRead.addParam(ID);
		if (dxl_addparam_result != true)
		{
			printf("[ID:%03d] groupSyncRead add param failed", ID);
		}
//	}

	// Syncread present data from indirectdata
	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		return;
	}

	// Check if groupsyncread data of Dyanamixel is available
	int dxl_getdata_result = 0;
	dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION);
	if (dxl_getdata_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ID);
	}

	// Check if groupsyncread data of Dyanamixel is available
	dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY);
	if (dxl_getdata_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ID);
	}

	// Check if groupsyncread data of Dyanamixel is available
	dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT);
	if (dxl_getdata_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ID);
	}

	// Get Dynamixel present position value
	*present_position = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ, LEN_PRESENT_POSITION));

	// Get Dynamixel present velocity value
	*present_velocity = static_cast<int32_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION, LEN_PRESENT_VELOCITY));

	// Get Dynamixel present current value
	*present_current = static_cast<int16_t>(groupSyncRead.getData(ID, ADDR_INDIRECTDATA_FOR_READ + LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY, LEN_PRESENT_CURRENT));

	groupSyncRead.clearParam();
}
