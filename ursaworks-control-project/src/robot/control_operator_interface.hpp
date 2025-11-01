#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

namespace xcysrc
{
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 */
class ControlOperatorInterface
{
public:
    static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
    static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);
    static constexpr float SPEED_REDUCTION_SCALAR = (1.0f / 3.0f);
    static constexpr float USER_STICK_SENTRY_DRIVE_SCALAR = 5000.0f;

    /**
     * Max acceleration in rpm/s^2 of the chassis in the x direction
     */
    static constexpr float MAX_ACCELERATION_X = 10'000.0f;
    static constexpr float MAX_DECELERATION_X = 20'000.0f;

    /**
     * Max acceleration in rpm/s^2 of the chassis in the y direction
     */
    static constexpr float MAX_ACCELERATION_Y = 9'000.0f;
    static constexpr float MAX_DECELERATION_Y = 20'000.0f;
    /**
     * Max acceleration in rpm/s^2 of the chassis in the r direction
     */
    static constexpr float MAX_ACCELERATION_R = 40'000.0f;
    static constexpr float MAX_DECELERATION_R = 50'000.0f;

    ControlOperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(ControlOperatorInterface)
    mockable ~ControlOperatorInterface() = default;

    /**
     * @return The value used for chassis movement forward and backward, between
     * `[-getMaxUserWheelSpeed, getMaxUserWheelSpeed]`. Acceleration is applied to this value
     * controlled by `MAX_ACCELERATION_X` and `MAX_DECELERATION_X`. A linear combination of keyboard
     * and remote joystick information.
     */
    mockable float getChassisXInput();

    /**
     * @return The value used for chassis movement side to side, between `[-getMaxUserWheelSpeed,
     * getMaxUserWheelSpeed]`. Acceleration is applied to this value controlled by
     * `MAX_ACCELERATION_Y` and `MAX_DECELERATION_Y`. A linear combination of keyboard and remote
     * joystick information.
     */
    mockable float getChassisYInput();

    /**
     * @return The value used for chassis rotation, between `[-getMaxUserWheelSpeed,
     * getMaxUserWheelSpeed]`. Acceleration is applied to this value controlled by
     * `MAX_ACCELERATION_R` and `MAX_DECELERATION_R`. A linear combination of keyboard and remote
     * joystick information.
     */
    mockable float getChassisRInput();

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretYawInput(uint8_t turretID);

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     */
    mockable float getTurretPitchInput(uint8_t turretID);

    // /**
    //  * @returns the value used for sentiel drive speed, between
    //  *      [-USER_STICK_SENTRY_DRIVE_SCALAR, USER_STICK_SENTRY_DRIVE_SCALAR].
    //  */
    // mockable float getSentrySpeedInput();

    /**
     * @returns whether or not the key to disable diagonal drive is pressed.
     * The key is shared with the speed scaling key.
     */
    bool isSlowMode();


    // subsequent code is for barebones vision integration
    static constexpr float USER_AIM_DEADBAND = 0.05f; // Deadband for joystick/mouse aiming

    /**
     * @returns true if the remote switch for auto-aim is active.
     */
    mockable bool isAutoAimSwitchActive();

    /**
     * @returns true if the user is actively providing manual aim input
     * (joystick or mouse) outside of the deadband.
     */
    mockable bool isUserAiming(uint8_t turretID);


private:
    tap::Drivers *drivers;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterR = 0;

    tap::algorithms::LinearInterpolationPredictor chassisXInput;
    tap::algorithms::LinearInterpolationPredictor chassisYInput;
    tap::algorithms::LinearInterpolationPredictor chassisRInput;

    tap::algorithms::Ramp chassisXInputRamp;
    tap::algorithms::Ramp chassisYInputRamp;
    tap::algorithms::Ramp chassisRInputRamp;

    uint32_t prevChassisXInputCalledTime = 0;
    uint32_t prevChassisYInputCalledTime = 0;
    uint32_t prevChassisRInputCalledTime = 0;

    /**
     * Scales `value` when ctrl/shift are pressed and returns the scaled value.
     */
    float applyChassisSpeedScaling(float value);
};  // class ControlOperatorInterface

}  // namespace control

}  // namespace xcysrc

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_
