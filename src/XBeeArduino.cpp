#include "XBeeArduino.h"
#include "xbee.h"
#include "xbee_api_frames.h"
#include "port.h"

/**
 * @file XBeeArduino.cpp
 * @brief Implementation of the XBeeArduino class for Arduino-compatible XBee modules.
 * 
 * This file contains the implementation of the XBeeArduino class, which provides an interface 
 * for interacting with XBee modules on an Arduino platform.
 * 
 * @version 1.0
 * @date 2024-08-17
 * author Felix Galindo
 * 
 * @license MIT
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

XBeeArduino* XBeeArduino::instance_ = nullptr;

/**
 * @brief Constructor for XBeeArduino class.
 * 
 * Initializes the XBee module based on the type and sets up callbacks and UART communication.
 * 
 * @param serialPort A pointer to a Stream object (HardwareSerial or SoftwareSerial).
 * @param baudrate The baud rate for UART communication.
 * @param moduleType The type of XBee module (standard or LoRa).
 * @param onReceiveCallback A callback function to handle received data.
 * @param onSendCallback A callback function to handle post-send events.
 */
XBeeArduino::XBeeArduino(Stream* serialPort, uint32_t baudrate, XBeeModuleType moduleType,
                         void (*onReceiveCallback)(void*),
                         void (*onSendCallback)(void*))
    : serialPort_(serialPort), moduleType_(moduleType), xbee_(nullptr), baudRate_(baudrate),
      onReceiveCallback_(onReceiveCallback), onSendCallback_(onSendCallback) {

    if (moduleType_ == XBEE_STANDARD) {
        // Initialization for standard XBee modules
    } else if (moduleType_ == XBEE_LORA) {
        instance_ = this;
        ctable_.OnReceiveCallback = onReceiveWrapper, 
        ctable_.OnSendCallback = onSendWrapper, 
        htable_.PortUartRead = portUartRead,
        htable_.PortUartWrite = portUartWrite,
        htable_.PortMillis = portMillis,
        htable_.PortFlushRx = portFlushRx,
        htable_.PortUartInit = portUartInit,
        htable_.PortDelay = portDelay,

        xbee_ = (XBee*)XBeeLRCreate(&ctable_, &htable_);
    }
}

/**
 * @brief Destructor for XBeeArduino class.
 * 
 * Ensures proper disconnection and cleanup of the XBee module.
 */
XBeeArduino::~XBeeArduino() {
    if (xbee_ != nullptr) {
        XBeeDisconnect(xbee_);
        delete xbee_;  // Free the memory allocated for the XBee structure
    }
}

/**
 * @brief Initializes the XBee module.
 * @return True if initialization is successful, otherwise false.
 */
bool XBeeArduino::begin() {
    return portUartInit(baudRate_, serialPort_) == 0;  // Corrected baudRate_ reference
}

/**
 * @brief Connects the XBee module to the network.
 * @return True if the connection is successful, otherwise false.
 */
bool XBeeArduino::connect() {
    if (xbee_ != nullptr) {
        return XBeeConnect(xbee_);
    }
    return false;
}

/**
 * @brief Lets the XBee class process 
 * @return void
 */
void XBeeArduino::process() {
    if (xbee_ != nullptr) {
        XBeeProcess(xbee_);
    }
}

/**
 * @brief Disconnects the XBee module from the network.
 * @return True if disconnection is successful, otherwise false.
 */
bool XBeeArduino::disconnect() {
    if (xbee_ != nullptr) {
        return XBeeDisconnect(xbee_);
    }
    return false;  // Added a return statement here
}

/**
 * @brief Sends data through the XBee module.
 * @param data The data to be sent.
 * @return True if the data is sent successfully, otherwise false.
 */
template <typename T>
bool XBeeArduino::sendData(const T& data) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeSendData(xbee_, &data) == API_SEND_SUCCESS;
    }
    return false;
}

// Explicit template instantiation for XBeeLRPacket_s
template bool XBeeArduino::sendData<XBeeLRPacket_s>(const XBeeLRPacket_s&);

/**
 * @brief Checks if the XBee module is connected to the network.
 * @return True if the module is connected, otherwise false.
 */
bool XBeeArduino::isConnected() {
    if (xbee_ != nullptr) {
        return XBeeConnected(xbee_);
    }
    return false;
}

/**
 * @brief Resets the XBee module.
 */
void XBeeArduino::reset() {
    if (xbee_ != nullptr) {
        (void)XBeeHardReset(xbee_);
    }
}

/**
 * @brief Sets the API options for XBee.
 * @param options The API options to set.
 * @return True if the options are set successfully, otherwise false.
 */
bool XBeeArduino::setApiOptions(const uint8_t options) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeSetAPIOptions(xbee_, options);
    }
    return false;
}

void XBeeArduino::onReceiveWrapper(XBee* xbee, void* data) {
    if (instance_ && instance_->onReceiveCallback_) {
        instance_->onReceiveCallback_(data);  // Forward the call to the original callback
    }
}

void XBeeArduino::onSendWrapper(XBee* xbee, void* data) {
    if (instance_ && instance_->onSendCallback_) {
        instance_->onSendCallback_(data);  // Forward the call to the original callback
    }
}

/**
 * @brief Applys config changes on XBee
 * @return True if the changes are applied successfully, otherwise false.
 */
bool XBeeArduino::applyChanges(void) {

    return XBeeApplyChanges(xbee_);
}

/**
 * @brief Write config on XBee
 * @return True if the write config is successfull, otherwise false.
 */
bool XBeeArduino::writeConfig(void) {

    return XBeeWriteConfig(xbee_);
}

/**
 * @brief Sets the App EUI of the LoRaWAN XBee module.
 * @param value A pointer to a buffer where the App EUI will be stored.
 * @return True if the AppEUI is set successfully, otherwise false.
 */
bool XBeeArduino::setLoRaWANAppEUI(const char* value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetAppEUI(xbee_, value);
    }
    return false;
}

/**
 * @brief Sets the App Key of the LoRaWAN XBee module.
 * @param value A pointer to a buffer where the App Key will be stored.
 * @return True if the App Key is set successfully, otherwise false.
 */
bool XBeeArduino::setLoRaWANAppKey(const char* value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetAppKey(xbee_, value);
    }
    return false;
}

/**
 * @brief Sets the App Key of the LoRaWAN XBee module.
 * @param value A pointer to a buffer where the App Key will be stored.
 * @return True if the App Key is set successfully, otherwise false.
 */
bool XBeeArduino::setLoRaWANNwkKey(const char* value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetNwkKey(xbee_, value);
    }
    return false;
}

/**
 * @brief Retrieves the DevEUI of the LoRaWAN XBee module.
 * @param devEUI A pointer to a buffer where the DevEUI will be stored.
 * @return True if the DevEUI is retrieved successfully, otherwise false.
 */
bool XBeeArduino::getLoRaWANDevEUI(uint8_t* devEUI, uint8_t length) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRGetDevEUI(xbee_, devEUI, length);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Class on the XBee LR module.
 * 
 * This function sends the AT_LC command to set the LoRaWAN Class 
 * (e.g., 'A', 'B', 'C') on the XBee LR module.
 * 
 * @param[in] value The LoRaWAN Class to be set ('A', 'B', or 'C').
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANClass(const char value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetClass(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Activation Mode on the XBee LR module.
 * 
 * This function sends the AT_AM command to set the Activation Mode 
 * (e.g., OTAA or ABP) on the XBee LR module.
 * 
 * @param[in] value The Activation Mode to be set.
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANActivationMode(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetActivationMode(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Adaptive Data Rate (ADR) on the XBee LR module.
 * 
 * This function sends the AT_AD command to enable or disable the 
 * Adaptive Data Rate (ADR) feature on the XBee LR module.
 * 
 * @param[in] value The ADR setting to be set (e.g., 0 for disable, 1 for enable).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANADR(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetADR(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Data Rate on the XBee LR module.
 * 
 * This function sends the AT_DR command to set the Data Rate 
 * on the XBee LR module.
 * 
 * @param[in] value The Data Rate to be set.
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANDataRate(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetDataRate(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Region on the XBee LR module.
 * 
 * This function sends the AT_LR command to set the operational region 
 * (e.g., US915, EU868) on the XBee LR module.
 * 
 * @param[in] value The Region to be set.
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANRegion(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetRegion(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Duty Cycle on the XBee LR module.
 * 
 * This function sends the AT_DC command to configure the duty cycle 
 * settings on the XBee LR module.
 * 
 * @param[in] value The Duty Cycle to be set.
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANDutyCycle(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetDutyCycle(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to get the LoRaWAN Specification Version from the XBee LR module.
 * 
 * This function sends the AT_LV command to retrieve the LoRaWAN 
 * Specification Version currently in use by the XBee LR module.
 * 
 * @param[out] responseBuffer Buffer to store the retrieved version string.
 * @param[in] buffer_size The size of the response buffer.
 * 
 * @return bool Returns true if the command was successfully sent and a valid response was received, otherwise false.
 */
bool XBeeArduino::getLoRaWANSpecVersion(char* responseBuffer, uint8_t buffer_size) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRGetSpecVersion(xbee_, responseBuffer, buffer_size);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Join RX1 Delay on the XBee LR module.
 * 
 * This function sends the AT_J1 command to set the RX1 delay 
 * (in milliseconds) for the Join Accept message on the XBee LR module.
 * 
 * @param[in] value The RX1 Delay value to be set (in milliseconds).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANJoinRX1Delay(const uint32_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetJoinRX1Delay(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Join RX2 Delay on the XBee LR module.
 * 
 * This function sends the AT_J2 command to set the RX2 delay 
 * (in milliseconds) for the Join Accept message on the XBee LR module.
 * 
 * @param[in] value The RX2 Delay value to be set (in milliseconds).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANJoinRX2Delay(const uint32_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetJoinRX2Delay(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN RX1 Delay on the XBee LR module.
 * 
 * This function sends the AT_D1 command to set the RX1 delay 
 * (in milliseconds) on the XBee LR module.
 * 
 * @param[in] value The RX1 Delay value to be set (in milliseconds).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANRX1Delay(const uint32_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetRX1Delay(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN RX2 Delay on the XBee LR module.
 * 
 * This function sends the AT_D2 command to set the RX2 delay 
 * (in milliseconds) on the XBee LR module.
 * 
 * @param[in] value The RX2 Delay value to be set (in milliseconds).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANRX2Delay(const uint32_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetRX2Delay(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN RX2 Data Rate on the XBee LR module.
 * 
 * This function sends the AT_XD command to set the RX2 Data Rate 
 * on the XBee LR module.
 * 
 * @param[in] value The RX2 Data Rate to be set.
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANRX2DataRate(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetRX2DataRate(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN RX2 Frequency on the XBee LR module.
 * 
 * This function sends the AT_XF command to set the RX2 Frequency 
 * on the XBee LR module.
 * 
 * @param[in] value The RX2 Frequency to be set (in Hz).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANRX2Frequency(const uint32_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetRX2Frequency(xbee_, value);
    }
    return false;
}

/**
 * @brief Wrapper function to set the LoRaWAN Transmit Power on the XBee LR module.
 * 
 * This function sends the AT_PO command to set the transmit power 
 * level on the XBee LR module.
 * 
 * @param[in] value The Transmit Power value to be set (in dBm).
 * 
 * @return bool Returns true if the command was successfully sent and the response was positive, otherwise false.
 */
bool XBeeArduino::setLoRaWANTransmitPower(const uint8_t value) {
    if (moduleType_ == XBEE_LORA) {
        return XBeeLRSetTransmitPower(xbee_, value);
    }
    return false;
}