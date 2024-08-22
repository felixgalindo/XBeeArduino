/**
 * @file xbee_lr.h
 * @brief Header file for the XBee LR (LoRaWAN) subclass.
 * 
 * This file defines the interface for the XBee LR subclass, which extends the
 * functionality of the base XBee class to support LoRaWAN
 * communication features specific to XBee LR modules. It includes prototypes
 * for initialization, connection handling, and other LR-specific operations.
 * 
 * @version 1.0
 * @date 2024-08-08
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
 * 
 * @author Felix Galindo
 * @contact felix.galindo@digi.com
 */


#ifndef XBEELR_H
#define XBEELR_H

#include "xbee.h"

#define CONNECTION_TIMEOUT_MS 6000
#define SEND_DATA_TIMEOUT_MS 10000

// Structure for XBee LR LoRaWAN packet
typedef struct XBeeLRPacket_s{
    uint8_t port;
    uint8_t payloadSize;
    uint8_t *payload;
    uint8_t ack;
    uint8_t status;
    uint8_t frameId;
    //For RX only
    int8_t rssi;
    int8_t snr;
    uint8_t dr;
    uint32_t counter;
}XBeeLRPacket_t;

// Subclass for XBeeLR
typedef struct {
    XBee base;  // Inherit from XBee
    // Add XBeeLR specific attributes here like methods specific to an XBee type
} XBeeLR;


XBeeLR* XBeeLRCreate(const XBeeCTable* cTable, const XBeeHTable* hTable);
bool XBeeLRGetDevEUI(XBee* self, uint8_t* responseBuffer, uint8_t buffer_size);
bool XBeeLRSetAppEUI(XBee* self, const uint8_t* value);
bool XBeeLRSetAppKey(XBee* self, const uint8_t* value);
bool XBeeLRSetNwkKey(XBee* self, const uint8_t* value);

#endif // XBEELR_H