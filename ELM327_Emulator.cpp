/*
 *  ELM327_Emu.cpp
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

#include "ELM327_Emulator.h"
#include "config.h"
#include "Logger.h"
#include "utility.h"
#include "esp32_can.h"
#include "can_manager.h"
#ifndef CONFIG_IDF_TARGET_ESP32S3
#include "BluetoothSerial.h"
#endif

#define RESP_UNKNOWN_COMMAND    "?"
#define RESP_NODATA             "NO DATA"

/*
 * Constructor. Nothing at the moment
 */
ELM327Emu::ELM327Emu() 
{
    tickCounter = 0;
    ibWritePtr = 0;
    mClient = 0;

    monitorAll_enable = false;
    allowLongMessages = false;
        
    canTransmitAddress = 0x7E0;

    canAutomaticFormatting_enable = true;
        
    canExtendedAddressing_enable = false;
    canExtendedAddressing_data = 0x00;

    canIdFilter_enable = false;
    canIdFilter_addr = 0x00;
    canIdFilter_addrExtended = false;

    canFlowControl_enable = false;
    canFlowControl_mode = 0;
    canFlowControl_addr = 0;
    canFlowControl_addrExtended = false;
    canFlowControl_data[0] = 0;
    canFlowControl_data[1] = 0;
    canFlowControl_data[2] = 0;
    canFlowControl_data[3] = 0;
    canFlowControl_data[4] = 0;
    canFlowControl_dataLen = 0;

    canReceiveAddress_enable = false;
    canReceiveAddress_addr = 0x00;
    canReceiveAddress_addrExtended = false;
    
    print_linefeed = false;
    print_echo = false;
    print_header = true;
    print_dlc = false;
    print_space = true;

    timeout = 200;       // Default Timeout : 200ms (0x32 * 4)

    waitingForRequest = false;
    waitingForRequest_millis = 0;
    waitingForRequest_CANcount = 0;
}

/*
 * Initialization of hardware and parameters
 */
void ELM327Emu::setup() {
#ifndef CONFIG_IDF_TARGET_ESP32S3
    serialBT.begin(settings.btName);
#endif
}

void ELM327Emu::setWiFiClient(WiFiClient *client)
{
    mClient = client;
}

bool ELM327Emu::getMonitorMode()
{
    return monitorAll_enable;
}

/*
 * Send a command to ichip. The "AT+i" part will be added.
 */
void ELM327Emu::sendCmd(String cmd) {
    txBuffer.sendString("AT");
    txBuffer.sendString(cmd);
    txBuffer.sendByteToBuffer(13);

    sendTxBuffer();

    loop(); // parse the response
}

/*
 * Called in the main loop (hopefully) in order to process serial input waiting for us
 * from the wifi module. It should always terminate its answers with 13 so buffer
 * until we get 13 (CR) and then process it.
 * But, for now just echo stuff to our serial port for debugging
 */

void ELM327Emu::loop() {
    int incoming;
    if (!mClient) //bluetooth
    {
#ifndef CONFIG_IDF_TARGET_ESP32S3

        while (serialBT.available()) {
            incoming = serialBT.read();
            if (incoming != -1) { //and there is no reason it should be -1
                if (incoming == 13 || ibWritePtr > 126) { // on CR or full buffer, process the line
                    incomingBuffer[ibWritePtr] = 0; //null terminate the string
                    ibWritePtr = 0; //reset the write pointer

                    if (Logger::isDebug())
                        Logger::debug(incomingBuffer);

                    processCmd();

                } else { // add more characters
                    if (incoming > 20 && monitorAll_enable) 
                    {
                        Logger::debug("Exiting monitor mode");
                        monitorAll_enable = false;
                    }
                    incomingBuffer[ibWritePtr++] = (char)(incoming);
                }
            } 
            else return;
        }

#endif
    }
    else //wifi and there is a client
    {
        while (mClient->available()) {
            incoming = mClient->read();
            if (incoming != -1) { //and there is no reason it should be -1
                if (incoming == 13 || ibWritePtr > 126) { // on CR or full buffer, process the line
                    incomingBuffer[ibWritePtr] = 0; //null terminate the string
                    ibWritePtr = 0; //reset the write pointer

                    if (Logger::isDebug())
                        Logger::debug(incomingBuffer);

                    processCmd();

                } else { // add more characters
                    incomingBuffer[ibWritePtr++] = (char)(incoming);
                }
            } 
            else return;
        }
    }
}

void ELM327Emu::sendTxBuffer() {
    if (mClient)
    {
        size_t wifiLength = txBuffer.numAvailableBytes();
        uint8_t* buff = txBuffer.getBufferedBytes();
        if (mClient->connected())
        {
            mClient->write(buff, wifiLength);
        }
    }
    else //bluetooth then
    {
#ifndef CONFIG_IDF_TARGET_ESP32S3
        serialBT.write(txBuffer.getBufferedBytes(), txBuffer.numAvailableBytes());
#endif
    }
    txBuffer.clearBufferedBytes();
}

/*
*   There is no need to pass the string in here because it is local to the class so this function can grab it by default
*   But, for reference, this cmd processes the command in incomingBuffer
*/
void ELM327Emu::processCmd() {
    String retString = processELMCmd(incomingBuffer);

    txBuffer.sendString(retString);
    sendTxBuffer();
    if (Logger::isDebug()) {
        char buff[300];
        retString = "Reply:" + retString;
        retString.toCharArray(buff, 300);
        Logger::debug(buff);
    }

}

String ELM327Emu::processELMCmd(char *cmd) {
    String retString = String();
    String lineEnding;
    if (print_linefeed) lineEnding = String("\r\n");
    else lineEnding = String("\r");

    if (monitorAll_enable) {
        monitorAll_enable = false;
    }

    // Echo commands without any changes
    if (print_echo)
    {
        retString.concat(cmd);
        retString.concat(lineEnding);
    }

    // remove line feed and spaces
    int r, w;
    char c;
    for (r = 0, w = 0; r < 128; r++) {
        c = cmd[r];
        if (c != '\0') {
            if ( (c == '\r' || c == '\n') || c == ' ' ) {
                // skip this char
            } else {
                cmd[w++] = (char)tolower(c); // convert lowercase to make processing easier
            } 
        } else {
            break;  // end of cmd
        }
    }
    cmd[w] = '\0';

    // process command
    if (!strncmp(cmd, "at", 2)) 
    {

        // ========== General Command ==========

        // ELM 1.0      AT MA       OBD Monitor All
        // ELM 1.0      AT AL       OBD Allow Long Messages
        // ELM 1.0      AT NL       OBD Normal Length (7 byte) messages
        // ELM 1.2      AT AR       OBD Automatic Response
        // ELM 1.0      AR L h      General Printing with Linefeed
        // ELM 1.0      AT E h      OBD Printing with ECHO
        // ELM 1.0      AT h H      OBD Printing with Header
        // ELM 1.3      AT D h      CAN Printing with DLC
        // ELM 1.3      AT S h      OBD Printing with Space
        // ELM 1.0      AT ST hh    OBD Set Timeout to hh x 4 msec
        // ELM 1.0      AT WS       General Warm Start
        // ELM 1.0      AT Z        General Reset All

        // ========== CAN Command ==========

        // ELM 1.0      AT SH hhh   CAN Transmit Adderss
        // ELM 1.0      AT CAF h    CAN Automatic Formatting On/Off
        // ELM 1.4      AT CEA      CAN Extended Address
        // ELM 1.4      AT CEA hh   CAN Extended Address
        // ELM 1.0      AT CF       CAN ID Filter
        // ELM 1.0      AT CF hhh   CAN ID Filter
        // ELM 1.0      AT CFC h    CAN Flow Control On/Off
        // ELM 1.1      AT FC SM h  CAN Flow Control Mode
        // ELM 1.1      AT FC SH h  CAN Flow Control Address
        // ELM 1.1      AT FC SD hh hh hh hh hh CAN Flow Control Data
        // ELM 1.4b     AT CRA      CAN Receive Address On/Off
        // ELM 1.3      AT CRA hhh  CAN Receive Address Address


        if (!strncmp(cmd, "atsh", 4)) //set header address (address we send queries to)
        { 
            size_t idSize = strlen(cmd+4);
            canTransmitAddress = Utility::parseHexString(cmd+4, idSize);
            Logger::debug("New ECU address: %x", canTransmitAddress);
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "ate", 3)) 
        { //turn echo on/off
            if (cmd[3] == '1') print_echo = true;
            if (cmd[3] == '0') print_echo = false;
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "ath", 3)) 
        { //turn headers on/off
            if (cmd[3] == '1') print_header = true;
            else print_header = false;
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "atl", 3)) 
        { //turn linefeeds on/off
            if (cmd[3] == '1') {
                print_linefeed = true;
            } else {
                print_linefeed = false;
            }
            retString.concat("OK");
        }
        else if (!strcmp(cmd, "at@1")) 
        { //send device description
            retString.concat("OBDLink MX");
        }
        else if (!strcmp(cmd, "ati")) 
        { //send chip ID
            retString.concat("ELM327 v1.5");
        }
        else if (!strncmp(cmd, "atat", 4)) 
        { //set adaptive timing
            //don't intend to support adaptive timing at all
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "atsp", 4)) 
        { //set protocol
            // 0	Automatic
            // 1	SAE J1850 PWM	41.6 kbaud
            // 2	SAE J1850 VPW	10.4 kbaud
            // 3	ISO 9141-2	5 baud init
            // 4	ISO 14230-4 KWP	5 baud init
            // 5	ISO 14230-4 KWP	Fast init
            // 6	ISO 15765-4 CAN	11 bit ID, 500 kbaud
            // 7	ISO 15765-4 CAN	29 bit ID, 500 kbaud
            // 8	ISO 15765-4 CAN	11 bit ID, 250 kbaud
            // 9	ISO 15765-4 CAN	29 bit ID, 250 kbaud
            // A	SAE J1939 CAN	29 bit ID, 250 kbaud

            if (cmd[4] == '0') {
                // search
                retString.concat("SEARCHING...");
                CAN_FRAME outFrame;
                outFrame.id = 0x7DF;
                outFrame.extended = false;
                outFrame.length = 8;
                outFrame.rtr = 0;
                outFrame.data.byte[0] = 0x02;
                outFrame.data.byte[1] = 0x01;
                outFrame.data.byte[2] = 0x00;
                outFrame.data.byte[3] = 0x00;
                outFrame.data.byte[4] = 0x00;
                outFrame.data.byte[5] = 0x00;
                outFrame.data.byte[6] = 0x00;
                outFrame.data.byte[7] = 0x00;
                canManager.sendFrame(&CAN0, outFrame);
                waitingForSearching_millis = millis();
                waitingForSearching = true;
            } else if (cmd[4] == '6') {
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
            
        }
        else if (!strcmp(cmd, "atdp")) 
        { //show description of protocol
            retString.concat("can11/500");
        }
        else if (!strcmp(cmd, "atdpn")) 
        { //show protocol number (same as passed to sp)
            retString.concat("6");
        }
        else if (!strncmp(cmd, "atd0", 4)) 
        { 
            print_dlc = false;
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "atd1", 4)) 
        { 
            print_dlc = true;
            retString.concat("OK");
        }
        else if (!strcmp(cmd, "atd")) 
        { //set to defaults
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "atma", 4)) //monitor all mode
        {
            Logger::debug("ENTERING monitor mode");
            monitorAll_enable = true;
        }
        else if (!strncmp(cmd, "atm", 3)) 
        { //turn memory on/off
            retString.concat("OK");
        }
        else if (!strcmp(cmd, "atrv")) 
        { //show 12v rail voltage
            //TODO: the system should actually have this value so it wouldn't hurt to
            //look it up and report the real value.
            retString.concat("14.2V");
        }
        else if (!strcmp(cmd, "atar")) {
            // AT AR : Automatic Receive
            canReceiveAddress_enable = false;
            retString.concat("OK");
        }
        else if (!strcmp(cmd, "atal")) {
            // AT AL : Allow Long Messages (> 7byte)
            allowLongMessages = true;
            retString.concat("OK");
        }
        else if (!strcmp(cmd, "atnl")) {
            // AT NL : Normal Length (7 byte) messages
            allowLongMessages = false;
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "atst", 4)) {
            // AT ST hh : set Timeout to hh x 4 msec
            timeout = Utility::parseHexString(cmd+4, 2) * 4;
            retString.concat("OK");
        }
        else if (!strncmp(cmd, "ats", 3)) {
            // AT S0 : printing of print_space Off
            // AT S1 : printing of print_space On
            if (cmd[3] == '0') {
                print_space = false;
                retString.concat("OK");
            } else if (cmd[3] == '1') {
                print_space = true;
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atcaf", 5)) {
            // AT CAF 0 : Auto Formatting Off
            // AT CAF 1 : Auto Formatting On
            if (cmd[5] == '0') {
                canAutomaticFormatting_enable = false;
                retString.concat("OK");
            } else if (cmd[5] == '1') {
                canAutomaticFormatting_enable = true;
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atcea", 5)) {
            // AT CEA : CAN Extended Address Off
            // AT CEA hh : CAN Extended Address set to hh

            uint8_t idSize = strlen(cmd+5);
            if (idSize == 0) {
                canExtendedAddressing_enable = false;
            } else if (idSize == 2) {
                canExtendedAddressing_enable = true;
                canExtendedAddressing_data = Utility::parseHexString(cmd+5, idSize);
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atcfc", 5)) {
            // AT CFC 0 : CAN Flow Control Off
            // AT CFC 1 : CAN Flow Control On
            if (cmd[5] == '0') {
                canFlowControl_enable = false;
                retString.concat("OK");
            } else if (cmd[5] == '1') {
                canFlowControl_enable = true;
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atfcsm", 6)) {
            // AT FC SM h : Flow Control, Set Mode to h
            // 0 = (default) fully automatic responses
            // 1 = completely user defined responses (ID & Data)
            // 2 = user defined data bytes in the response (Data)
            if ('0' <= cmd[6] && cmd[6] <= '2') {
                canFlowControl_mode = cmd[6] - '0';
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atfcsh", 6)) {
            // AT FC SH hhh : Flow Control Set Header to hhh
            // AT FC SH hh hh hh hh : Flow Control Set Header to hhhhhhhh
            uint8_t idSize = strlen(cmd+6);

            if (idSize == 3) {
                // Standard Format
                canFlowControl_addr = Utility::parseHexString(cmd+6, idSize);
                canFlowControl_addrExtended = false;
                retString.concat("OK");
            } else if (idSize == 8) {
                // Extended Format
                canFlowControl_addr = Utility::parseHexString(cmd+6, idSize);
                canFlowControl_addrExtended = true;
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atfcsd", 6)) {
            // AT FC SD [1 - 5 bytes] : Flow Control Set Data to [...]
            uint8_t idSize = strlen(cmd+6);
            if ((1 < idSize && idSize < 11) && (idSize % 2 == 0)) {
                for (int i = 0; (i*2) < idSize; i++) {
                    canFlowControl_data[i] = Utility::parseHexString(cmd+6+(i*2), 2);
                }
                canFlowControl_dataLen = idSize / 2;
                retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strncmp(cmd, "atcra", 5)) {
            // AT CRA : Reset the Receive Address filters
            // AT CRA hhh : Set CAN Receive Address to hhh
            // AT CRA hh hh hh hh : Set CAN Receive Address to hhhhhhhh
            uint8_t idSize = strlen(cmd+5);
            
            if (idSize == 0) {
                canReceiveAddress_enable = false;
                retString.concat("OK");
            } else if (idSize == 3) {
                canReceiveAddress_enable = true;
                canReceiveAddress_addr = Utility::parseHexString(cmd+5, idSize);
                canReceiveAddress_addrExtended = false;
                retString.concat("OK");
            // } else if (idSize == 8) {
            //     canReceiveAddress_enable = true;
            //     canReceiveAddress_addr = Utility::parseHexString(cmd+5, idSize);
            //     canReceiveAddress_addrExtended = true;
            //     retString.concat("OK");
            } else {
                retString.concat(RESP_UNKNOWN_COMMAND);
            }
        }
        else if (!strcmp(cmd, "atz") || !strcmp(cmd, "atws")) {

            monitorAll_enable = false;
            allowLongMessages = false;

            canTransmitAddress = 0x7E0;

            canAutomaticFormatting_enable = true;
                
            canExtendedAddressing_enable = false;
            canExtendedAddressing_data = 0x00;

            canIdFilter_enable = false;
            canIdFilter_addr = 0x00;
            canIdFilter_addrExtended = false;

            canFlowControl_enable = false;
            canFlowControl_mode = 0;
            canFlowControl_addr = 0;
            canFlowControl_addrExtended = false;
            canFlowControl_data[0] = 0;
            canFlowControl_data[1] = 0;
            canFlowControl_data[2] = 0;
            canFlowControl_data[3] = 0;
            canFlowControl_data[4] = 0;
            canFlowControl_dataLen = 0;

            canReceiveAddress_enable = false;
            canReceiveAddress_addr = 0x00;
            canReceiveAddress_addrExtended = false;
            
            print_linefeed = false;
            print_echo = false;
            print_header = true;
            print_dlc = false;
            print_space = true;

            timeout = 200;       // Default Timeout : 200ms (0x32 * 4)

            waitingForRequest = false;
            waitingForRequest_millis = 0;
            waitingForRequest_CANcount = 0;

            retString.concat("ELM327 v1.4 (ESP32RET)");
            retString.concat(lineEnding);
        }


        else 
        { //unknown command
            retString.concat(RESP_UNKNOWN_COMMAND);
        }
        
        retString.concat(lineEnding);
        retString.concat(">");

    }
    else if (('0' <= cmd[0] && cmd[0] <= '9') || cmd[0] == 'a' || cmd[0] == 'b' || cmd[0] == 'c' || cmd[0] == 'd' || cmd[0] == 'e' || cmd[0] == 'f')
    { //if no AT then assume it is a PID request. This takes the form of four bytes which form the alpha hex digit encoding for two bytes
        //there should be four or six characters here forming the ascii representation of the PID request. Easiest for now is to turn the ascii into
        //a 16 bit number and mask off to get the bytes
        CAN_FRAME outFrame;
        outFrame.id = canTransmitAddress;
        outFrame.extended = false;
        outFrame.length = 8;
        outFrame.rtr = 0;
        outFrame.data.byte[3] = 0xAA; outFrame.data.byte[4] = 0xAA;
        outFrame.data.byte[5] = 0xAA; outFrame.data.byte[6] = 0xAA;
        outFrame.data.byte[7] = 0xAA;
        size_t cmdSize = strlen(cmd);

        // for stability issues
        if (cmdSize % 2 == 1)
            cmdSize--;

        if ((0 < cmdSize && cmdSize < 17) && (cmdSize % 2 == 0)) {
            uint64_t val = strtol((char *) cmd, NULL, 16);
            uint8_t data;
            int i = 0;

            if (canExtendedAddressing_enable) {
                outFrame.data.byte[i] = canExtendedAddressing_data;
                i++;
            }

            if (canAutomaticFormatting_enable) {
                outFrame.data.byte[i] = (cmdSize/2);
                i++;
            }

            for (int j = 0; i < 8; i++, j+=2) {
                if ((j) < cmdSize) {
                    data = ((Utility::parseHexCharacter(cmd[j]) << 4) & 0xF0) + (Utility::parseHexCharacter(cmd[j+1]) & 0x0F);
                    outFrame.data.byte[i] = data;
                } else {
                    outFrame.data.byte[i] = 0xAA;
                }
            }
        } else {
            retString.concat(RESP_UNKNOWN_COMMAND);
        }

        waitingForRequest_millis = millis();
        waitingForRequest = true;
        
        canManager.sendFrame(&CAN0, outFrame);

        retString.concat(lineEnding);
    } else {
        // unknown command
        retString.concat(RESP_UNKNOWN_COMMAND);
        retString.concat(lineEnding);
        retString.concat(">");
    }

    // retString.concat(lineEnding);
    // retString.concat(">"); //prompt to show we're ready to receive again

    return retString;
}

void ELM327Emu::processCANReply(CAN_FRAME &frame) {
    //at the moment assume anything sent here is a legit reply to something we sent. Package it up properly
    //and send it down the line

    String retString = String();
    String lineEnding;
    if (print_linefeed) lineEnding = String("\r\n");
    else lineEnding = String("\r");
    bool multiframe = false;
    bool send = true;

    char buff[8];
    if (print_header)
    {
        sprintf(buff, "%03X", frame.id);
        // txBuffer.sendString(buff);
        retString.concat(buff);
    }

    if (print_space) retString.concat(" ");

    if (print_dlc)
    {
        sprintf(buff, "%u", frame.length);
        // txBuffer.sendString(buff);
        retString.concat(buff);

        if (print_space) retString.concat(" ");
    }
    
    if (waitingForRequest == true) {
        
        if (((canReceiveAddress_enable == false) && ( (frame.id == (canTransmitAddress - 8)) || (frame.id == (canTransmitAddress + 8)) )) || ((canReceiveAddress_enable == true) && (frame.id == canReceiveAddress_addr))) {

            uint8_t multi_frame_id = (frame.data.byte[0] & 0xF0);
            if (multi_frame_id == 0x10 || multi_frame_id == 0x20 || multi_frame_id == 0x30) {
                // Multi Frame 
                multiframe = true;
                if (allowLongMessages) {
                    // Only Work in AT AL mode
                    waitingForRequest_CANcount++;

                    if (multi_frame_id == 0x10) {

                        if (canFlowControl_enable == true) {
                            if (canFlowControl_mode == 0) {         // Flow Control : Auto Mode
                                CAN_FRAME outFrame;
                                if (0x7E8 <= frame.id)
                                    outFrame.id = frame.id - 8;
                                else
                                    outFrame.id = frame.id + 8;
                                outFrame.extended = false;
                                outFrame.length = 8;
                                outFrame.rtr = 0;

                                outFrame.data.byte[0] = 0x30;
                                outFrame.data.byte[1] = 0x00;
                                outFrame.data.byte[2] = 0x00;
                                outFrame.data.byte[3] = 0x00;
                                outFrame.data.byte[4] = 0x00;
                                outFrame.data.byte[5] = 0x00;
                                outFrame.data.byte[6] = 0x00;
                                outFrame.data.byte[7] = 0x00;

                                canManager.sendFrame(&CAN0, outFrame);

                                waitingForRequest_millis = millis();
                                waitingForRequest = true;

                            } else if (canFlowControl_mode == 1) {  // Flow Control : User Mode
                                CAN_FRAME outFrame;
                                outFrame.id = canFlowControl_addr;
                                outFrame.extended = canFlowControl_addrExtended;
                                outFrame.length = 8;
                                outFrame.rtr = 0;

                                for (int i = 0; i < 8; i++) {
                                    if (i < canFlowControl_dataLen)
                                        outFrame.data.byte[i] = canFlowControl_data[i];   // Flow Control Data
                                    else
                                        outFrame.data.byte[i] = 0xAA;       // padding
                                }
                                canManager.sendFrame(&CAN0, outFrame);
                                waitingForRequest_millis = millis();
                                waitingForRequest = true;

                            } else if (canFlowControl_mode == 2) {  // Flow Control : Data Mode
                                CAN_FRAME outFrame;
                                if (0x7E8 <= frame.id)
                                    outFrame.id = frame.id - 8;
                                else
                                    outFrame.id = frame.id + 8;
                                outFrame.extended = false;
                                outFrame.length = 8;
                                outFrame.rtr = 0;

                                for (int i = 0; i < 8; i++) {
                                    if (i < canFlowControl_dataLen)
                                        outFrame.data.byte[i] = canFlowControl_data[i];   // Flow Control Data
                                    else
                                        outFrame.data.byte[i] = 0xAA;       // padding
                                }
                                canManager.sendFrame(&CAN0, outFrame);
                                waitingForRequest_millis = millis();
                                waitingForRequest = true;
                            }
                        }

                    } else if (multi_frame_id == 0x30) {

                    } else if (multi_frame_id == 0x20) {

                    }

                } else {
                    send = false;
                }
            } else {
                // Single Frame
                multiframe = false;
                waitingForRequest_CANcount++;
            }

        }

    }

    if (multiframe) {
        for (int i = 0; i < 8; i++) {
            if (i > 0) if (print_space) retString.concat(" ");
            sprintf(buff, "%02X", frame.data.byte[i]);
            retString.concat(buff);
        }
        retString.concat(lineEnding);
    } else {

        if (canAutomaticFormatting_enable) {
            for (int i = 0; i < frame.data.byte[0]; i++) {
                if (i > 0) if (print_space) retString.concat(" ");
                sprintf(buff, "%02X", frame.data.byte[i+1]);
                retString.concat(buff);
            }
        } else {
            for (int i = 0; i < 8; i++) {
                if (i > 0) if (print_space) retString.concat(" ");
                sprintf(buff, "%02X", frame.data.byte[i]);
                retString.concat(buff);
            }
        }


        retString.concat(lineEnding);
    }

    if (send) {
        txBuffer.sendString(retString);
        sendTxBuffer();
    }

}

void ELM327Emu::processCANWaiting(CAN_FRAME &frame) {

    String retString = String();
    String lineEnding;            
    if (print_linefeed) lineEnding = String("\r\n");
    else lineEnding = String("\r");

    if ( (millis() - waitingForRequest_millis) > timeout ) {
        // 수신되는 응답은 즉각 전달
        // timeout 까지 CAN 메시지가 몇개 수신되었는지 카운트하고, 없다면 NO DATA 를 전송
        if (waitingForRequest_CANcount == 0) {
            retString.concat(RESP_NODATA);
        }
        retString.concat(lineEnding);
        retString.concat(">");

        txBuffer.sendString(retString);
        sendTxBuffer();

        waitingForRequest = false;
        waitingForRequest_CANcount = 0;
    }
}

bool ELM327Emu::isWaitingReply() {
    return waitingForRequest;
}

bool ELM327Emu::isCRAenabled() {
    return canReceiveAddress_enable;
}

uint32_t ELM327Emu::getCRA() {
    return canReceiveAddress_addr;
}

void ELM327Emu::processCANSearching(CAN_FRAME &frame) {

    String retString = String();
    String lineEnding;
    if (print_linefeed) lineEnding = String("\r\n");
    else lineEnding = String("\r");
    char buff[8];

    if (waitingForSearching) {

        if ( (millis() - waitingForSearching_millis) > 1000 ) {
            // 1초동안 search 응답 수신
            retString.concat("UNABLE TO CONNECT");
            retString.concat(lineEnding);
            retString.concat(">");
            txBuffer.sendString(retString);
            sendTxBuffer();
            waitingForSearching = false;
        } else {
            // 수신된 CAN 확인
            if ( (0x7E8 <= frame.id) && (frame.id <= 0x7EF) ) {
                if ((frame.data.byte[1] == 0x41) &&(frame.data.byte[2] == 0x00)) {
                    // Success
                    
                    if (print_header) {
                        sprintf(buff, "%03X", frame.id);
                        retString.concat(buff);
                    }

                    if (print_space) retString.concat(" ");

                    if (print_dlc) {
                        sprintf(buff, "%u", frame.length);
                        retString.concat(buff);

                        if (print_space) retString.concat(" ");
                    }

                    if (canAutomaticFormatting_enable) {
                        for (int i = 0; i < frame.data.byte[0]; i++) {
                            if (i > 0) if (print_space) retString.concat(" ");
                            sprintf(buff, "%02X", frame.data.byte[i+1]);
                            retString.concat(buff);
                        }
                    } else {
                        for (int i = 0; i < 8; i++) {
                            if (i > 0) if (print_space) retString.concat(" ");
                            sprintf(buff, "%02X", frame.data.byte[i]);
                            retString.concat(buff);
                        }
                    }
                    retString.concat(lineEnding);
                    retString.concat(">");

                    txBuffer.sendString(retString);
                    sendTxBuffer();

                    waitingForSearching = false;
                }
            }
        }
    }
}

bool ELM327Emu::isWaitingSearch() {
    return waitingForSearching;
}