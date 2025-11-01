#include "src/control/vision.hpp"

namespace control {
namespace vision {

constexpr modm::baudrate_t BAUDRATE = 115.2_kBd;
tap::communication::serial::Uart uart;
const size_t MESSAGE_LENGTH = 14; // Define according to your protocol


// Define your message protocol
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE = 0xBB;

// States for our receiver
enum class ReceiverState
{
    WAITING_FOR_START,
    RECEIVING_MESSAGE
};

ReceiverState currentState;

void initialize(tap::Drivers *drivers)
{
    uart.init<tap::communication::serial::Uart::UartPort::Uart1, BAUDRATE>();
    sendAcknowledgment(uart, drivers); // send acknowledgement to jetson on startup
    currentState = ReceiverState::WAITING_FOR_START;
}

bool readUartData(std::vector<uint8_t> &messageBuffer)
{
    
    uint8_t incomingByte;
    messageBuffer.clear();
    size_t messageIndex = 0;

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

std::vector<float> getVisionPitchYaw()
{
    float pitch;
    float yaw;

    std::vector<uint8_t> messageBuffer;
    messageBuffer.reserve(control::vision::MESSAGE_LENGTH);
    bool messageReceived = control::vision::readUartData(messageBuffer);
    if (messageReceived)
    {
        std::string message(reinterpret_cast<const char*>(messageBuffer.data()), messageBuffer.size());
        size_t y_pos = message.find('Y');
        if (message[0] == 'P' && y_pos != std::string::npos) {
            std::string pitch_str = message.substr(1, y_pos - 1);
            std::string yaw_str = message.substr(y_pos + 1);
            pitch = std::stof(pitch_str);
            yaw = std::stof(yaw_str);
        }
        else
        {
            // Invalid message format, default to zero input
            pitch = 0.0f;
            yaw = 0.0f;
        }
    }
    else
    {
        // No valid message received, default to zero input
        pitch = 0.0f;
        yaw = 0.0f;
    }
    return {pitch, yaw};
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
} // namespace vision
} // namespace control