#include "tap/communication/serial/uart.hpp"
using namespace::modm::literals;
#include "src/drivers_singleton.hpp"

namespace control {
namespace vision {

    /**
     * initializes uart, currentState, and sends team color to jetson
     */
    void initialize(tap::Drivers *drivers);

    /**
     * Reads a message from uart
     * 
     * @param messageBuffer the vector to store the message in
     * 
     * @return true if message read successfully
     *          false otherwise
     */
    bool readUartData(std::vector<uint8_t>& messageBuffer);

    /** 
     * Parses the received message buffer into meaningful data.
     * @return a vector where the first value is pitch and the second value is yaw
     */
    std::vector<float> getVisionPitchYaw();

    /**
     * Sends team color to the jetson
     * 
     * @param uart the uart object
     * @param drivers drivers singleton
     * 
     * @return true if message successfully sent
     *          false otherwise
     */
    bool sendAcknowledgment(tap::communication::serial::Uart &uart, tap::Drivers *drivers);

} // namespace vision
} // namespace control