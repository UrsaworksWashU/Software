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


#include "tap/board/board.hpp"
#include "modm/platform/timer/timer_1.hpp"

#include "modm/architecture/interface/delay.hpp"
#include "modm/architecture/interface/interrupt.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "src/drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

#include "src/robot/robot_control.hpp"

#include "tap/communication/gpio/pwm.hpp"

#include "taproot/src/tap/algorithms/smooth_pid.hpp"

#include "taproot/src/tap/communication/serial/remote_serial_constants.hpp"

#include "taproot/src/tap/communication/serial/remote.hpp"

#include "tap/communication/serial/uart.hpp"
using namespace::modm::literals;


static constexpr float MAIN_LOOP_FREQUENCY = 500.0f;
static constexpr float MAHONY_KP = 0.1f;

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(tap::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(tap::Drivers *drivers);

using namespace xcysrc::standard;

static constexpr tap::motor::MotorId agitatorID = tap::motor::MOTOR7;
static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM =1000;
tap::motor::DjiMotor agimotor(::DoNotUse_getDrivers(),agitatorID,CAN_BUS2,false,"cool motor");

tap::motor::DjiMotor l1(::DoNotUse_getDrivers(),tap::motor::MOTOR1,CAN_BUS2,false,"cool motor");


static void initializePWM(tap::Drivers *drivers)
{
    drivers->leds.set(tap::gpio::Leds::Blue, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Blue, false);
    tap::gpio::Pwm::Pin pwmPin1= tap::gpio::Pwm::Pin::C1;
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 100);
    tap::gpio::Pwm::Pin pwmPin2= tap::gpio::Pwm::Pin::C2;
    drivers->pwm.write(.2,pwmPin1);
    drivers->pwm.write(.2,pwmPin2);
    modm::delay_ms(2000);
    drivers->pwm.write(0.06,pwmPin1);
    drivers->pwm.write(0.06,pwmPin2);
    modm::delay_ms(2000);
    drivers->pwm.write(0.13,pwmPin1);
    drivers->pwm.write(0.13,pwmPin2);
    modm::delay_ms(500);
    
// every time robot got killed or power off, initialize the gpio again
// better to make it to the button
}

static void flyingWheel(tap::Drivers *drivers)
{
    float rotateSpeed = 0.5f;

    tap::communication::serial::Remote::SwitchState switchState = drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH);
    if(switchState == tap::communication::serial::Remote::SwitchState::UP){
        drivers->pwm.write(0.14,tap::gpio::Pwm::Pin::C1);
        drivers->pwm.write(0.14,tap::gpio::Pwm::Pin::C2);
    }
    else if (switchState == tap::communication::serial::Remote::SwitchState::DOWN){
        drivers->pwm.write(0.14,tap::gpio::Pwm::Pin::C1);
        drivers->pwm.write(0.14,tap::gpio::Pwm::Pin::C2);
    }
    else {
        drivers->pwm.write(0.06,tap::gpio::Pwm::Pin::C1);
        drivers->pwm.write(0.06,tap::gpio::Pwm::Pin::C2);

        // rotate chassis with rotateSpeed
        // drivers->pwm.write(rotateSpeed,tap::gpio::Pwm::Pin::C1);
    }
}

static void spin(tap::Drivers *drivers)
{
    // bool spinOn = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN);
    // if(spinOn){
    //     drivers->pwm.write(0.13,tap::gpio::Pwm::Pin::C1);
    //     drivers->pwm.write(0.13,tap::gpio::Pwm::Pin::C2);
    // }
    // else{
    //     drivers->pwm.write(0.06,tap::gpio::Pwm::Pin::C1);
    //     drivers->pwm.write(0.06,tap::gpio::Pwm::Pin::C2);
    // }
}


static void agitatorSpin(tap::Drivers *drivers)
{
    /*tap::motor::DjiMotor agitatorMotor(::DoNotUse_getDrivers(), agitatorID, CAN_BUS,false,"cool motor"); */
    bool spin = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::UP) || drivers->remote.getMouseL();
    bool inv = (drivers->remote.getSwitch(tap::communication::serial::Remote::Switch::LEFT_SWITCH) == tap::communication::serial::Remote::SwitchState::DOWN) || drivers->remote.getMouseR();
    if(spin){
        agimotor.setDesiredOutput(static_cast<int32_t>(1200));  
    }
    else if(inv){
        agimotor.setDesiredOutput(static_cast<int32_t>(-3000));  
    }
    else{
        agimotor.setDesiredOutput(static_cast<int32_t>(0));  
    }
    drivers->djiMotorTxHandler.processCanSendData();
}

static void rotate(tap::Drivers *drivers) {
    float wheelInput = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL);
    drivers->leds.set(tap::gpio::Leds::Blue, wheelInput > 0.8F);
    drivers->leds.set(tap::gpio::Leds::Red, wheelInput < -0.8F);
    l1.setDesiredOutput(static_cast<int16_t>(5000));
    drivers->djiMotorTxHandler.processCanSendData();
}


static void IMUData(tap::Drivers *drivers)
{
    float yaw = drivers->bmi088.getYaw();
}



constexpr modm::baudrate_t BAUDRATE = 115.2_kBd;
tap::communication::serial::Uart uart;
const size_t MESSAGE_LENGTH = 4; // Define according to your protocol
int messageIndex = 0;

// Define your message protocol
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE = 0xBB;

// States for our receiver
enum class ReceiverState
{
    WAITING_FOR_START,
    RECEIVING_MESSAGE
};

// Below is a simple UART message receiver implementation
// --- Global variables to manage state ---
ReceiverState currentState = ReceiverState::WAITING_FOR_START;

bool readUartData(std::vector<uint8_t> &messageBuffer)
{
    uint8_t incomingByte;
    messageBuffer.clear();

    // Process all available bytes in the UART buffer
    while (uart.read(tap::communication::serial::Uart::UartPort::Uart1, &incomingByte))
    {
        switch (currentState)
        {
            case ReceiverState::WAITING_FOR_START:
                if (incomingByte == START_BYTE)
                {
                    currentState = ReceiverState::RECEIVING_MESSAGE;
                }
                // Ignore any other bytes
                break;

            case ReceiverState::RECEIVING_MESSAGE:
                if (incomingByte == END_BYTE)
                {
                    // Reset to wait for the next message
                    currentState = ReceiverState::WAITING_FOR_START;
                    messageIndex = 0;

                    // Complete message received
                    if (messageIndex == MESSAGE_LENGTH - 1)
                    {
                        return true;
                    }
                    // Incomplete message, discard
                    else
                    {
                        messageBuffer.clear();
                        return false;
                    }
                    

                }
                else
                {
                    // Add the byte to our buffer, with a check for overflow
                    if (messageIndex < MESSAGE_LENGTH)
                    {
                        messageBuffer[messageIndex++] = incomingByte;
                    }
                    else
                    {
                        // Buffer overflow! Discard the message and reset.
                        currentState = ReceiverState::WAITING_FOR_START;
                        messageIndex = 0;
                        messageBuffer.clear();
                        return false;
                    }
                }
                break;
        }
    }

    // if we are here, then there is nothing over UART to read
    return false;
}


bool sendAcknowledgment(tap::communication::serial::Uart &uart, tap::Drivers *drivers)
{
    tap::communication::serial::RefSerialData::RobotId myId = drivers->refSerial.getRobotData().robotId;
    std::string teamColor;
    if (tap::communication::serial::RefSerialData::isBlueTeam(myId))
    {
        teamColor = "BLUE";
    }
    else
    {
        teamColor = "RED";
    }
    
    return uart.write(
    tap::communication::serial::Uart::UartPort::Uart1,
    (const uint8_t*)teamColor.c_str(), // Cast the pointer to const uint8_t*
    teamColor.length()                  // Send the exact length of the string
    );
}



int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers *drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    drivers->leds.set(tap::gpio::Leds::Red, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Red, false);
    initSubsystemCommands(drivers);
    drivers->leds.set(tap::gpio::Leds::Green, true);
    modm::delay_ms(1000);
    drivers->leds.set(tap::gpio::Leds::Green, false);   
    agimotor.initialize();
    initializePWM(drivers); 

    uart.init<tap::communication::serial::Uart::UartPort::Uart1, BAUDRATE>();
    sendAcknowledgment(uart, drivers); // send acknowledgement to jetson on startup

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->bmi088.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.processCanSendData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());


            agitatorSpin(drivers);
            //rotate(drivers);
            flyingWheel(drivers);







            spin(drivers);
            IMUData(drivers);

        }
        modm::delay_us(10);
    }
    return 0;
}


static void initializeIo(tap::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    drivers->bmi088.initialize(MAIN_LOOP_FREQUENCY, MAHONY_KP, 0.0f);
}

static void updateIo(tap::Drivers *drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
//    drivers->bmi088.update();
}
