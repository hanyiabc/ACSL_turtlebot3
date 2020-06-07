/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#pragma once

#include <TurtleBot3.h>
#include <stdint.h>
#define WAFFLE_DXL_LIMIT_MAX_EFFORT 3

class TurtleBot3MotorEffortDriver
{
public:
    TurtleBot3MotorEffortDriver();
    ~TurtleBot3MotorEffortDriver();
    bool init(String turtlebot3);
    void close(void);
    bool setTorque(bool onoff);
    bool getTorque();
    bool readEncoder(int32_t &left_value, int32_t &right_value);
    bool writeVelocity(int64_t left_value, int64_t right_value);
    bool controlMotor(const float wheel_radius, const float wheel_separation, float *value);

private:
    uint32_t baudrate_;
    float protocol_version_;
    uint8_t left_wheel_id_;
    uint8_t right_wheel_id_;
    bool torque_;

    uint16_t dynamixel_limit_max_velocity_;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
    dynamixel::GroupSyncRead *groupSyncReadEncoder_;



    double dynamixel_limit_max_effort_;
    dynamixel::GroupSyncWrite *groupSyncWriteEffort_;
};

