/*
 *  ELM327_Emu.h
 *
 * Class emulates the serial comm of an ELM327 chip - Used to create an OBDII interface
 *
 * Created: 3/23/2017
 *  Author: Collin Kidder
 */

/*
 Copyright (c) 2017 Collin Kidder

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
List of AT commands to support:
AT E0 (turn echo off)
AT H (0/1) - Turn headers on or off - headers are used to determine how many ECU√≠s present (hint: only send one response to 0100 and emulate a single ECU system to save time coding)
AT L0 (Turn linefeeds off - just use CR)
AT Z (reset)
AT SH - Set header address - seems to set the ECU address to send to (though you may be able to ignore this if you wish)
AT @1 - Display device description - ELM327 returns: Designed by Andy Honecker 2011
AT I - Cause chip to output its ID: ELM327 says: ELM327 v1.3a
AT AT (0/1/2) - Set adaptive timing (though you can ignore this)
AT SP (set protocol) - you can ignore this
AT DP (get protocol by name) - (always return can11/500)
AT DPN (get protocol by number) - (always return 6)
AT RV (adapter voltage) - Send something like 14.4V
*/


#ifndef ELM327_H_
#define ELM327_H_

#include <Arduino.h>
#include <WiFi.h>
#include "commbuffer.h"
#ifndef CONFIG_IDF_TARGET_ESP32S3
#include "BluetoothSerial.h"
#endif

class CAN_FRAME;

class ELM327Emu {
public:

    ELM327Emu();
    void setup(); //initialization on start up
    void handleTick(); //periodic processes
    void loop();
    void setWiFiClient(WiFiClient *client);
    void sendCmd(String cmd);
    void processCANReply(CAN_FRAME &frame);
    bool getMonitorMode();

    void processCANWaiting(CAN_FRAME &frame);
    bool isWaitingReply();

    void processCANSearching(CAN_FRAME &frame);
    bool isWaitingSearch();

    bool isCRAenabled();
    uint32_t getCRA();

private:
#ifndef CONFIG_IDF_TARGET_ESP32S3
    BluetoothSerial serialBT;
#endif
    WiFiClient *mClient;
    CommBuffer txBuffer;
    char incomingBuffer[128]; //storage for one incoming line
    char buffer[30]; // a buffer for various string conversions
    int tickCounter;
    int ibWritePtr;
    int currReply;


    bool     monitorAll_enable;             // OBD Monitor All                   (AT MA)    ELM 1.0
    bool     allowLongMessages;             // OBD Allow Long Messages           (AT AL)    ELM 1.0
    
    uint32_t canTransmitAddress;            // CAN Transmit Address          (AT SH hhh)    ELM 1.0

    bool     canAutomaticFormatting_enable; // CAN Automatic Formatting On/Off (AT CAF1)    ELM 1.0

    
    bool     canExtendedAddressing_enable;  // CAN Extended Address             (AT CEA)    ELM 1.4
    uint32_t canExtendedAddressing_data;    // CAN Extended Address          (AT CEA hh)    ELM 1.4

    bool     canIdFilter_enable;            // CAN ID Filter                     (AT CF)    ELM 1.0
    uint32_t canIdFilter_addr;              // CAN ID Filter      (AT CF hhh | hhhhhhhh)    ELM 1.0
    bool     canIdFilter_addrExtended;

    bool     canFlowControl_enable;         // CAN Flow Control On/Off         (AT CFC1)    ELM 1.0
    uint8_t  canFlowControl_mode;           // CAN Flow Control Mode        (AT FC SM 1)    ELM 1.1
    uint32_t canFlowControl_addr;           // CAN Flow Control Address       (AT FC SH)    ELM 1.1
    bool     canFlowControl_addrExtended;   // CAN Flow Control Extended Address Y/N
    uint8_t  canFlowControl_data[5];        // CAN Flow Control Data          (AT FC SD)    ELM 1.1
    uint8_t  canFlowControl_dataLen;        // CAN Flow Control Data Len

    // bool     canAutomaticResponse_enable;   // OBD Automatic Response            (AT AR)    ELM 1.2
    bool     canReceiveAddress_enable;      // CAN Receive Address On/Off       (AT CRA)    ELM 1.4b
    uint32_t canReceiveAddress_addr;        // CAN Receive Address Address  (AT CRA hhh)    ELM 1.3
    bool     canReceiveAddress_addrExtended;// CAN Receive Address Extended Address
    
    bool     print_linefeed;                // General Printing with Linefeed    (AT L1)    ELM 1.0
    bool     print_echo;                    // OBD Printing with ECHO            (AT E1)    ELM 1.0
    bool     print_header;                  // OBD Printing with Header          (AT H1)    ELM 1.0
    bool     print_dlc;                     // CAN Printing with DLC             (AT D1)    ELM 1.3
    bool     print_space;                   // OBD Printing with Space           (AT S1)    ELM 1.3

    uint16_t timeout;                       // OBD Timeout                    (AT ST hh)    ELM 1.0


    bool     waitingForRequest;
    uint32_t waitingForRequest_millis;
    uint16_t waitingForRequest_CANcount;

    bool     waitingForSearching;
    uint32_t waitingForSearching_millis;

    void processCmd();
    String processELMCmd(char *cmd);
    void sendTxBuffer();
};

#endif
