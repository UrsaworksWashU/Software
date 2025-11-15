/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "chassis_autorotate_command.hpp"

#include <algorithm>
#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"
#include "modm/math/geometry/angle.hpp"

#include "src/control/turret/turret_subsystem.hpp"

#include "chassis_rel_drive.hpp"
#include "chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace xcysrc::control::turret;

namespace xcysrc::chassis
{
ChassisAutorotateCommand::ChassisAutorotateCommand(
    tap::Drivers* drivers,
    xcysrc::control::ControlOperatorInterface* operatorInterface,
    MecanumChassisSubsystem* chassis,
    const xcysrc::control::turret::TurretMotor* yawMotor,
    ChassisSymmetry chassisSymmetry)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis),
      yawMotor(yawMotor),
      chassisSymmetry(chassisSymmetry),
      chassisAutorotating(true)
{
    addSubsystemRequirement(chassis);
}

void ChassisAutorotateCommand::initialize()
{
    desiredRotationAverage = chassis->getDesiredRotation();
}

void ChassisAutorotateCommand::updateAutorotateState()
{
    float turretYawActualSetpointDiff = std::abs(yawMotor->getValidChassisMeasurementError());

    

    if (chassisAutorotating && chassisSymmetry != ChassisSymmetry::SYMMETRICAL_NONE &&
        !yawMotor->getConfig().limitMotorAngles &&
        turretYawActualSetpointDiff > (M_PI - TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION))
    {
        // If turret setpoint all of a sudden turns around, don't autorotate
        chassisAutorotating = false;
    }
    else if (
        !chassisAutorotating &&
        turretYawActualSetpointDiff < TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION)
    {
        // Once the turret setpoint/target have reached each other, start turning again
        chassisAutorotating = true;
    }
}

float ChassisAutorotateCommand::computeAngleFromCenterForAutorotation(
    float turretAngleFromCenter,
    float maxAngleFromCenter)
{
    // Use limitVal instead of WrappedFloat to clamp the angle instead of wrapping it
    // This prevents the angle from suddenly flipping when it exceeds maxAngleFromCenter
    // Original code used ContiguousFloat which clamps values, not wraps them
    return limitVal(turretAngleFromCenter, -maxAngleFromCenter, maxAngleFromCenter);
}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (yawMotor->isOnline())
    {
        updateAutorotateState();

        float turretAngleFromCenter = yawMotor->getAngleFromCenter();

        // Original code had: if (turretAngleFromCenter < 500) { chassisAutorotating = false; }
        // This condition was always true (since angle is in radians, range -π to π, so < 500 is always true)
        // So it always disabled autorotation. We replace it with checking spinOn state.
        // Disable autorotation when spinOn is false (prevents autorotation from starting automatically on boot)
        bool spinOn = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN);
        if (!spinOn)
        {
            chassisAutorotating = false;
        }

        bool isMoving = drivers->remote.keyPressed(tap::communication::serial::Remote::Key::W) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::A) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::S) || drivers->remote.keyPressed(tap::communication::serial::Remote::Key::D);

        if (!spinOn){
            desiredRotationAverage = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL) * 6000;
        }
        else{
            if (isMoving){
                // ======== 1st standard ========
                // desiredRotationAverage = 0.5 * 6000;

                // ======== 2nd standard ========
                desiredRotationAverage = 0.5 * 6000;
            }
            else{
                // ======== 1st standard ========
                // desiredRotationAverage = 0.5 * 6000;

                // ======== 2nd standard ========
                desiredRotationAverage = 0.5 * 6000;
            }
            
        }

        

        if (chassisAutorotating)
        {
            float maxAngleFromCenter = M_PI;

            if (!yawMotor->getConfig().limitMotorAngles)
            {
                switch (chassisSymmetry)
                {
                    case ChassisSymmetry::SYMMETRICAL_180:
                        maxAngleFromCenter = M_PI_2;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_90:
                        maxAngleFromCenter = M_PI_4;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_NONE:
                    default:
                        break;
                }
            }
            float angleFromCenterForChassisAutorotate =
                computeAngleFromCenterForAutorotation(turretAngleFromCenter, maxAngleFromCenter);
            // PD controller to find desired rotational component of the chassis control
            float desiredRotation = chassis->chassisSpeedRotationPID(
                angleFromCenterForChassisAutorotate,
                yawMotor->getChassisFrameVelocity());

            // find an alpha value to be used for the low pass filter, some value >
            // AUTOROTATION_MIN_SMOOTHING_ALPHA, inversely proportional to
            // angleFromCenterForChassisAutorotate, so when autorotate angle error is large, low
            // pass filter alpha is small and more averaging will be applied to the desired
            // autorotation
            float autorotateSmoothingAlpha = std::max(
                1.0f - std::abs(angleFromCenterForChassisAutorotate) / maxAngleFromCenter,
                AUTOROTATION_MIN_SMOOTHING_ALPHA);

            // low pass filter the desiredRotation to avoid radical changes in the desired
            // rotation when far away from where we are centering the chassis around
            desiredRotationAverage =
                lowPassFilter(desiredRotationAverage, desiredRotation, autorotateSmoothingAlpha);
        }

        const float maxWheelSpeed = MecanumChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // the x/y translational speed is limited to this value, this means when rotation is
        // large, the translational speed will be clamped to a smaller value to compensate
        float rotationLimitedMaxTranslationalSpeed =
            maxWheelSpeed * chassis->calculateRotationTranslationalGain(desiredRotationAverage);

        float chassisXDesiredWheelspeed = limitVal(
            operatorInterface->getChassisXInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        float chassisYDesiredWheelspeed = limitVal(
            operatorInterface->getChassisYInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        // Rotate X and Y depending on turret angle
        rotateVector(&chassisXDesiredWheelspeed, &chassisYDesiredWheelspeed, turretAngleFromCenter);

        chassis->setDesiredOutput(
            chassisXDesiredWheelspeed,
            chassisYDesiredWheelspeed,
            desiredRotationAverage);
    }
    else
    {
        ChassisRelDrive::onExecute(operatorInterface, drivers, chassis);
    }
}

void ChassisAutorotateCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisAutorotateCommand::isFinished() const { return false; }

}  // namespace xcysrc::chassis
