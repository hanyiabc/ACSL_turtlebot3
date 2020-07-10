
#include "turtlebot3_motor_torque_driver.h"

TurtleBot3MotorTorqueDriver::TurtleBot3MotorTorqueDriver()
    : baudrate_(BAUDRATE),
      protocol_version_(PROTOCOL_VERSION),
      left_wheel_id_(DXL_LEFT_ID),
      right_wheel_id_(DXL_RIGHT_ID)
{
    torque_ = false;
    // dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
    dynamixel_limit_max_current_ = WAFFLE_DXL_LIMIT_MAX_CURRENT;
}

TurtleBot3MotorTorqueDriver::~TurtleBot3MotorTorqueDriver()
{
    close();
}

bool TurtleBot3MotorTorqueDriver::init(String turtlebot3)
{
    DEBUG_SERIAL.begin(57600);
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler_->openPort() == false)
    {
        DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
        return false;
    }

    // Set port baudrate
    if (portHandler_->setBaudRate(baudrate_) == false)
    {
        DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
        return false;
    }

    // Enable Dynamixel Torque
    setTorque(true);

    groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
    groupSyncReadEncoder_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    groupSyncWriteTorque_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_CURRENT, LEN_X_GOAL_CURRENT);
    groupSyncReadTorque_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);

    // if (turtlebot3 == "Burger")
    //     dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
    // else if (turtlebot3 == "Waffle or Waffle Pi")
    //     dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
    // else
    //     dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

    DEBUG_SERIAL.println("Success to init Motor Driver");
    return true;
}

bool TurtleBot3MotorTorqueDriver::setTorque(bool onoff)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    torque_ = onoff;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

bool TurtleBot3MotorTorqueDriver::getTorque()
{
    return torque_;
}

void TurtleBot3MotorTorqueDriver::close(void)
{
    // Disable Dynamixel Torque
    setTorque(false);

    // Close port
    portHandler_->closePort();
    DEBUG_SERIAL.end();
}

bool TurtleBot3MotorTorqueDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadEncoder_->addParam(left_wheel_id_);
    if (dxl_addparam_result != true)
        return false;

    dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
    if (dxl_addparam_result != true)
        return false;

    // Syncread present position
    dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;

    dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;

    // Get data
    left_value = groupSyncReadEncoder_->getData(left_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

    groupSyncReadEncoder_->clearParam();
    return true;
}

bool TurtleBot3MotorTorqueDriver::readTorque(float &left_torque, float &right_torque)
{
    int16_t left_value, right_value;
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadTorque_->addParam(left_wheel_id_);
    if (dxl_addparam_result != true)
        return false;

    dxl_addparam_result = groupSyncReadTorque_->addParam(right_wheel_id_);
    if (dxl_addparam_result != true)
        return false;

    // Syncread present position
    dxl_comm_result = groupSyncReadTorque_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadTorque_->isAvailable(left_wheel_id_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);
    if (dxl_getdata_result != true)
        return false;

    dxl_getdata_result = groupSyncReadTorque_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);
    if (dxl_getdata_result != true)
        return false;

    // Get data
    left_value = groupSyncReadTorque_->getData(left_wheel_id_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);
    right_value = groupSyncReadTorque_->getData(right_wheel_id_, ADDR_X_PRESENT_CURRENT, LEN_X_PRESENT_CURRENT);
    // Convert register value to torque
    left_torque = CURRENT_TO_TORQUE(OUTPUT_TO_CURRENT(left_value));
    right_torque = CURRENT_TO_TORQUE(OUTPUT_TO_CURRENT(right_value));

    groupSyncReadTorque_->clearParam();
    return true;
}

bool TurtleBot3MotorTorqueDriver::writeVelocity(int64_t left_value, int64_t right_value)
{
    bool dxl_addparam_result;
    int8_t dxl_comm_result;

    uint8_t left_data_byte[4] = {
        0,
    };
    uint8_t right_data_byte[4] = {
        0,
    };

    left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
    left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
    left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
    left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

    dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_wheel_id_, (uint8_t *)&left_data_byte);
    if (dxl_addparam_result != true)
        return false;

    right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
    right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
    right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
    right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

    dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_wheel_id_, (uint8_t *)&right_data_byte);
    if (dxl_addparam_result != true)
        return false;

    dxl_comm_result = groupSyncWriteVelocity_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    groupSyncWriteVelocity_->clearParam();
    return true;
}

bool TurtleBot3MotorTorqueDriver::writeTorque(int16_t left_value, int16_t right_value)
{
    bool dxl_addparam_result;
    int8_t dxl_comm_result;

    dxl_addparam_result = groupSyncWriteTorque_->addParam(left_wheel_id_, (uint8_t *)&left_value);
    if (dxl_addparam_result != true)
        return false;

    dxl_addparam_result = groupSyncWriteTorque_->addParam(right_wheel_id_, (uint8_t *)&right_value);
    if (dxl_addparam_result != true)
        return false;

    dxl_comm_result = groupSyncWriteTorque_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    groupSyncWriteTorque_->clearParam();
    return true;
}



bool TurtleBot3MotorTorqueDriver::controlMotor(const float wheel_radius, const float wheel_separation, float *value)
{
    bool dxl_comm_result = false;

    float wheel_velocity_cmd[2];

    float lin_vel = value[LEFT];
    float ang_vel = value[RIGHT];

    wheel_velocity_cmd[LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_velocity_cmd[LEFT] = constrain(wheel_velocity_cmd[LEFT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

    dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
    if (dxl_comm_result == false)
        return false;

    return true;
}

bool TurtleBot3MotorTorqueDriver::controlMotor(float *torque)
{
        bool dxl_comm_result = false;

        uint16_t wheel_current_cmd[2];
        wheel_current_cmd[LEFT] = CURRENT_TO_OUTPUT(TORQUE_TO_CURRENT(torque[LEFT]));
        wheel_current_cmd[RIGHT] = CURRENT_TO_OUTPUT(TORQUE_TO_CURRENT(torque[RIGHT]));

        wheel_current_cmd[LEFT] = constrain(wheel_current_cmd[LEFT], -dynamixel_limit_max_current_, dynamixel_limit_max_current_);
        wheel_current_cmd[RIGHT] = constrain(wheel_current_cmd[RIGHT], -dynamixel_limit_max_current_, dynamixel_limit_max_current_);

        dxl_comm_result = writeTorque((int16_t)wheel_current_cmd[LEFT], (int16_t)wheel_current_cmd[RIGHT]);
        if (dxl_comm_result == false)
            return false;

        return true;
}
