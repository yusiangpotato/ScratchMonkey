// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1        - STK500v2 compatible programmer for Arduino
//
// File: SMoISP.cpp         - In-System Programming commands
//
// Copyright (c) 2013 Matthias Neeracher <microtherion@gmail.com>
// All rights reserved.
//
// See license at bottom of this file or at
// http://opensource.org/licenses/bsd-license.php
//
// Derived from Randall Bohn's ArduinoISP sketch
//

#include <SPI.h>

#include "SMoISP.h"
#include "SMoHWIF.h"
#include "SMoGeneral.h"
#include "SMoCommand.h"
#include "SMoConfig.h"
#ifdef DEBUG_ISP
#include "SMoDebug.h"
#endif

//
// Pin definitions
//
enum {
#if SMO_LAYOUT==SMO_LAYOUT_STANDARD
    ISP_RESET       = SS,
    MCU_CLOCK       = 9,    // OC1A    
#elif SMO_LAYOUT==SMO_LAYOUT_LEONARDO
    ISP_RESET       = 10,
    MCU_CLOCK       = 9,    // OC1A
#else
    ISP_RESET       = SS,
    MCU_CLOCK       = 11,   // OC1A
#endif
};

//
// If an MCU has been set to use the 125kHz internal oscillator, 
// regular SPI speeds are much too fast, so we do a software 
// emulation that's deliberately slow.
//
static int currentSpeed    = 0;

static uint8_t 
SPITransfer(uint8_t out)
{
    SMoHWIF::ISP::transferSPI(out,currentSpeed);
}

static uint8_t 
SPITransaction(const uint8_t * sendData, int8_t responseIndex = 3)
{
    uint8_t response;

#ifdef DEBUG_ISP
    SMoDebug.print("SPI ");
#endif
    for (int8_t ix=0; ix<4; ++ix) {
#ifdef DEBUG_ISP
        SMoDebug.print(*sendData, HEX);
#endif
        uint8_t recv = SPITransfer(*sendData++);
#ifdef DEBUG_ISP
        SMoDebug.print(ix == responseIndex ? " ![" : " [");
        SMoDebug.print(recv, HEX);
        SMoDebug.print("] ");
#endif
        if (ix == responseIndex)
            response = recv;
    }
#ifdef DEBUG_ISP
    SMoDebug.println();
#endif
    return response;
}

static uint8_t
SPITransaction(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
#ifdef DEBUG_ISP
    SMoDebug.print("SPI ");
    SMoDebug.print(b1, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b2, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b3, HEX);
    SMoDebug.print(" ");
    SMoDebug.print(b4, HEX);
#endif
    SPITransfer(b1);
    SPITransfer(b2);
    SPITransfer(b3);
#ifdef DEBUG_ISP
    uint8_t result = SPITransfer(b4);
    SMoDebug.print(" [");
    SMoDebug.print(result, HEX);
    SMoDebug.println("]");
  
    return result;  
#else
    return SPITransfer(b4);
#endif
}

static uint8_t
SPITransaction(uint8_t b1, uint16_t address, uint8_t b4)
{
    return SPITransaction(b1, (address >> 8) & 0xFF, address & 0xFF, b4);
}

static bool
ISPPollReady()
{
    uint32_t timeout = millis()+100;
    uint32_t now;
    do {
        if (SPITransaction(0xF0, 0, 0, 0))
            return true;
        now = millis();
    } while ((timeout < 100 && now & 0x800000) || now < timeout);

    SMoCommand::SendResponse(STATUS_RDY_BSY_TOUT);

    return false;
}

static void
LoadExtendedAddress()
{
#ifdef DEBUG_ISP
        SMoDebug.print("Address was: ");
        SMoDebug.println(SMoGeneral::gAddress, HEX);
#endif
    uint16_t highBits = SMoGeneral::gAddress >> 16;
    if ((highBits >> 8) != (highBits & 0xFF)) {
        SPITransaction(0x4D, 0, highBits & 0x7F, 0);
        highBits    = 0x80 | (highBits & 0x7F);
        highBits   |= highBits << 8;
        SMoGeneral::gAddress = (SMoGeneral::gAddress & 0xFFFF) | (uint32_t(highBits) << 16);
#ifdef DEBUG_ISP
        SMoDebug.print("Address now: ");
        SMoDebug.println(SMoGeneral::gAddress, HEX);
#endif
    }
}

void
SMoISP::EnterProgmode()
{
#ifdef DEBUG_ISP
    SMoDebugInit();
    SMoDebug.print("Pin layout ");
    SMoDebug.print(SMO_LAYOUT);
    SMoDebug.print(" RESET ");
    SMoDebug.println(ISP_RESET);
#endif
    // const uint8_t   timeOut     =   SMoCommand::gBody[1];
    // const uint8_t   stabDelay   =   SMoCommand::gBody[2];
    // const uint8_t   cmdexeDelay =   SMoCommand::gBody[3];
    // const uint8_t   synchLoops  =   SMoCommand::gBody[4];
    // const uint8_t   byteDelay   =   SMoCommand::gBody[5];
    const uint8_t   pollValue   =   SMoCommand::gBody[6];
    const uint8_t   pollIndex   =   SMoCommand::gBody[7];
    const uint8_t * command     =  &SMoCommand::gBody[8];

    SMoHWIF::ISP::init();

    //
    // Now reset the chip and issue the programming mode instruction
    //
    //digitalWrite(SCK, LOW); //Why is this here anyway? IIRC it doesnt do anything when pin is switched to alt fn SPI, and anyway the SPI idles low.
    delay(50);
    SMoHWIF::ISP::writeReset(LOW);
    delay(50);
    
    
    uint8_t response=0; 
    currentSpeed=0; //Reset speed between invocations
    do {
#ifdef DEBUG_ISP
        SMoDebug.print("Trying speed No. ");
        SMoDebug.println(currentSpeed);
#endif
        SMoHWIF::ISP::writeReset(HIGH);
        //digitalWrite(SCK, LOW); //Ditto
        delay(50);
        SMoHWIF::ISP::writeReset(LOW);
        delay(50);
        response     = SPITransaction(command, pollIndex-1);
#ifdef DEBUG_ISP
        SMoDebug.print("Received ");
        SMoDebug.println(response,HEX);
#endif
    } while (response != pollValue && ++currentSpeed <= SMoHWIF::ISP::minSpeed()); //Try speeds from 0(fastest) to the given maximum value(slowest)
    SMoCommand::SendResponse(response==pollValue ? STATUS_CMD_OK : STATUS_CMD_FAILED);
}

void
SMoISP::LeaveProgmode()
{
    SMoHWIF::ISP::cleanup();
    SMoHWIF::ISP::writeReset(HIGH);
    SMoCommand::SendResponse();
}

void
SMoISP::ChipErase()
{
    const uint8_t   eraseDelay  =   SMoCommand::gBody[1];
    const uint8_t   pollMethod  =   SMoCommand::gBody[2];
    const uint8_t * command     =  &SMoCommand::gBody[3];    

    SPITransaction(command);
    if (pollMethod) {
        if (!ISPPollReady())
            return;
    } else
        delay(eraseDelay);
    SMoCommand::SendResponse();
}

static void
ProgramMemory(bool wordBased)
{
    uint16_t  numBytes          =  (SMoCommand::gBody[1]<<8)|SMoCommand::gBody[2];
    uint8_t   mode              =   SMoCommand::gBody[3];
    const uint8_t   cmdDelay    =   SMoCommand::gBody[4];
    const uint8_t   cmd1        =   SMoCommand::gBody[5];
    const uint8_t   cmd2        =   SMoCommand::gBody[6];
    const uint8_t   cmd3        =   SMoCommand::gBody[7];
    const uint8_t   pollVal1    =   SMoCommand::gBody[8];
    const uint8_t   pollVal2    =   SMoCommand::gBody[9];
    const uint8_t * data        =  &SMoCommand::gBody[10];

    LoadExtendedAddress();
    uint32_t address = SMoGeneral::gAddress;
    while (numBytes--) {
        SPITransaction(cmd1, SMoGeneral::gAddress, *data++);
        if (wordBased) {
            --numBytes;
            SPITransaction(cmd1|8, SMoGeneral::gAddress, *data++);
        }
        ++SMoGeneral::gAddress;
    }
    if ((mode & 0x81) == 0x81) {
        //
        // Write page
        //
        SPITransaction(cmd2, address, 0);
        mode >>= 3;
    } else if (mode & 0x01) {
        mode   = 0;
    }
    if (mode & 0x02)
        delay(cmdDelay);
    if (mode & 0x04) {
        uint8_t pollVal = wordBased ? pollVal1 : pollVal2;
        if (pollVal == SMoCommand::gBody[10])
            delay(cmdDelay); // Values are identical - don't poll
        else
            while (SPITransaction(cmd3, address, 0) == pollVal)
                ;
    }
    if (mode & 0x08)
        if (!ISPPollReady())
            return;
    SMoCommand::SendResponse();
}

static void
ReadMemory(bool wordBased)
{
    uint16_t  numBytes    =  (SMoCommand::gBody[1]<<8)|SMoCommand::gBody[2];
    const uint8_t   cmd   =   SMoCommand::gBody[3];
    uint8_t * data        =  &SMoCommand::gBody[2];

    LoadExtendedAddress();
    while (numBytes--) {
        *data++ = SPITransaction(cmd, SMoGeneral::gAddress, 0);
        if (wordBased) {
            --numBytes;
            *data++ = SPITransaction(cmd|8, SMoGeneral::gAddress, 0);
        }
        ++SMoGeneral::gAddress;
    }
    *data++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, data-&SMoCommand::gBody[0]);   
}

void
SMoISP::ProgramFlash()
{
    ProgramMemory(true);
}

void
SMoISP::ReadFlash()
{
    ReadMemory(true);
}

void
SMoISP::ProgramEEPROM()
{
    ProgramMemory(false);
}

void
SMoISP::ReadEEPROM()
{
    ReadMemory(false);
}

void
SMoISP::ProgramFuse()
{
    SPITransaction(&SMoCommand::gBody[1]);
    SMoCommand::gBody[2] = STATUS_CMD_OK;
    SMoCommand::SendResponse();
}

void
SMoISP::ReadFuse()
{
    uint8_t pollIndex   = SMoCommand::gBody[1];
    SMoCommand::gBody[2]= SPITransaction(&SMoCommand::gBody[2], pollIndex);
    SMoCommand::gBody[2] = STATUS_CMD_OK;
    SMoCommand::SendResponse();
}

void
SMoISP::SPIMulti()
{
    uint8_t         numTX   =   SMoCommand::gBody[1];
    uint8_t         numRX   =   SMoCommand::gBody[2];
    uint8_t         rxStart =   SMoCommand::gBody[3];
    const uint8_t * txData  =  &SMoCommand::gBody[4];
    uint8_t *       rxData  =  &SMoCommand::gBody[2];

#ifdef DEBUG_ISP
    SMoDebug.print("!SPI");
#endif
    while (numTX) {
#ifdef DEBUG_ISP
        SMoDebug.print(" ");
        SMoDebug.print(*txData, HEX);
#endif
        *rxData = SPITransfer(*txData++);
        --numTX;
#ifdef DEBUG_ISP
        SMoDebug.print(rxStart ? " (" : " [");
        SMoDebug.print(*rxData, HEX);
        SMoDebug.print(rxStart ? ")" : "]");
#endif
        if (rxStart) {
            --rxStart;
        } else if (numRX) {
            ++rxData;
            --numRX;
        }
    }
    while (numRX) {
        *rxData = SPITransfer(0);
#ifdef DEBUG_ISP
        SMoDebug.print(rxStart ? " . (" : " . [");
        SMoDebug.print(*rxData, HEX);
        SMoDebug.print(rxStart ? ")" : "]");
#endif
        if (rxStart) {
            --rxStart;
        } else {
            ++rxData;
            --numRX;
        }
    }
    *rxData++ = STATUS_CMD_OK;
    SMoCommand::SendResponse(STATUS_CMD_OK, rxData-&SMoCommand::gBody[0]);
#ifdef DEBUG_ISP
    SMoDebug.println();
#endif
}

//
// LICENSE
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, this 
//    list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation 
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
