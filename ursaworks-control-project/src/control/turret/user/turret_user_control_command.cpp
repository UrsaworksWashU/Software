/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "turret_user_control_command.hpp"

#include "tap/drivers.hpp"

#include "../turret_subsystem.hpp"
#include "src/robot/control_operator_interface.hpp"

#include "src/main.cpp" // for readUartData

namespace xcysrc::control::turret::user
{
TurretUserControlCommand::TurretUserControlCommand(
    tap::Drivers *drivers,
    ControlOperatorInterface &controlOperatorInterface,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    float userYawInputScalar,
    float userPitchInputScalar,
    uint8_t turretID)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar),
      turretID(turretID)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretUserControlCommand::isReady() { return !isFinished(); }

void TurretUserControlCommand::initialize()
{
    yawController->initialize();
    pitchController->initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretUserControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    float pitchInput;
    float yawInput;

    // Check if auto-aim is active and user is not aiming manually
    if (controlOperatorInterface.isAutoAimSwitchActive() && !controlOperatorInterface.isUserAiming(turretID))
    {
        std::vector<uint8_t> messageBuffer;
        messageBuffer.reserve(MESSAGE_LENGTH);
        bool messageReceived = readUartData(messageBuffer);
        if (messageReceived)
        {
            std::string message(reinterpret_cast<const char*>(messageBuffer.data()), messageBuffer.size());
            size_t y_pos = message.find('Y');
            if (message[0] == 'P' && y_pos != std::string::npos) {
                std::string pitch_str = message.substr(1, y_pos - 1);
                std::string yaw_str = message.substr(y_pos + 1);
                pitchInput = std::stof(pitch_str);
                yawInput = std::stof(yaw_str);
            }
            else
            {
                // Invalid message format, default to zero input
                pitchInput = 0.0f;
                yawInput = 0.0f;
            }
        }
        else
        {
            // No valid message received, default to zero input
            pitchInput = 0.0f;
            yawInput = 0.0f;
        }
    }
    // Otherwise, use manual user input
    else
    {
        pitchInput = userPitchInputScalar * controlOperatorInterface.getTurretPitchInput(turretID);
        yawInput = userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID);
    }

    const float pitchSetpoint =
        pitchController->getSetpoint() +
        pitchInput;
    pitchController->runController(dt, pitchSetpoint);

    // Get current world frame yaw angle from turret imu
    float currYaw = drivers->bmi088.getYaw();

    bool spinOn = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN);

    bool isMoving = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D);

    // ======== 1st standard ========
    // if (!spinOn)
    // {
    //     const float yawSetpoint =
    //         yawController->getSetpoint() +
    //         userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID)
    //         - (drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) / 86.0f);


    //     yawController->runController(dt, yawSetpoint);
    // }
    // else
    // {
    //     if (isMoving)
    //     {
    //         const float yawSetpoint =
    //             yawController->getSetpoint() +
    //             userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID)
    //             -(0.5f / 78.0f);

    //         yawController->runController(dt, yawSetpoint);
    //     }
    //     else
    //     {
    //         const float yawSetpoint =
    //             yawController->getSetpoint() +
    //             userYawInputScalar * controlOperatorInterface.getTurretYawInput(turretID)
    //             -(0.5f / 73.0f);

    //         yawController->runController(dt, yawSetpoint);
    //     }
    // }

    // ======== 2nd standard ========
    if (!spinOn)
    {
        const float yawSetpoint =
            yawController->getSetpoint() +
            yawInput
            - (drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) / 86.0f);


        yawController->runController(dt, yawSetpoint);
    }
    else
    {
        if (isMoving)
        {
            const float yawSetpoint =
                yawController->getSetpoint() +
                yawInput
                -(0.85f / 78.0f);

            yawController->runController(dt, yawSetpoint);
        }
        else
        {
            const float yawSetpoint =
                yawController->getSetpoint() +
                yawInput
                -(0.85f / 73.0f);

            yawController->runController(dt, yawSetpoint);
        }
    }
}

bool TurretUserControlCommand::isFinished() const
{
    return !pitchController->isOnline() && !yawController->isOnline();
}

void TurretUserControlCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}


}  // namespace xcysrc::control::turret::user
