// -*- mode: c++; tab-width: 4; indent-tabs-mode: nil -*-
//
// ScratchMonkey 0.1            - STK500v2 compatible programmer for Arduino
//
// File: ScratchMonkey.ino      - Main program of sketch
//
// Copyright (c) 2013 Matthias Neeracher <microtherion@gmail.com>
// All rights reserved.
//
// See license at bottom of this file or at
// http://opensource.org/licenses/bsd-license.php
//
// This sketch turns an Arduino into an AVR programmer, supporting the following
// protocols:
//
//  * stk500v2    for ISP programming (largely pin compatible with ArduinoISP
//                sketch). See SMoISP.h for pinout.
//  * stk500hvsp  for HVSP programming (high voltage serial, for 8 and 14 pin
//                ATtinys). See SMoHVSP.h for pinout.
//  * stk500pp    for HVPP programming (high voltage parallel, for 20 pin
//                ATtinys and all ATmegas). See SMoHVPP.h for pinout.
//

#include <SPI.h>
//#include <SoftwareSerial.h>

#include "SMoCommand.h"
#include "SMoGeneral.h"
#include "SMoISP.h"
#include "SMoHVSP.h"
#include "SMoHVPP.h"
#include "SMoHWIF.h"

void
setup()
{
  Serial.begin(115200);
  SMoHWIF::releasePins();
  #ifdef _SMoHWIF_led1_
    SMoHWIF::trisLed1(OUTPUT);
  #endif
  #ifdef _SMoHWIF_led2_
    SMoHWIF::trisLed2(OUTPUT);
  #endif
  #ifdef _SMoHWIF_led1_
    while(!Serial){ //Until we've connected to a computer (and opened the port)
      SMoHWIF::writeLed1(HIGH); //Blink the LED1 to show power but not data
      delay(500);
      SMoHWIF::writeLed1(LOW);
      delay(500);
    }
  #endif
}

void
loop()
{
  #ifdef _SMoHWIF_led1_ //After connected to computer once, LED1 is steady except
    SMoHWIF::writeLed1(Serial.available()?LOW:HIGH); //when acting as !RX indicator
  #endif
  
  switch (SMoCommand::GetNextCommand()) {
    //
    // General commands
    //
    case CMD_SIGN_ON:
      SMoGeneral::SignOn();
      break;
    case CMD_SET_PARAMETER:
      SMoGeneral::SetParam();
      break;
    case CMD_GET_PARAMETER:
      SMoGeneral::GetParam();
      break;
    case CMD_LOAD_ADDRESS:
      SMoGeneral::LoadAddress();
      break;
    case CMD_SET_CONTROL_STACK:
      SMoGeneral::SetControlStack();
      break;
    //
    // ISP Commands
    //
    case CMD_ENTER_PROGMODE_ISP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(HIGH);
      #endif
      SMoISP::EnterProgmode();
      break;
    case CMD_LEAVE_PROGMODE_ISP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(LOW);
      #endif
      SMoISP::LeaveProgmode();
      break;
    case CMD_CHIP_ERASE_ISP:
      SMoISP::ChipErase();
      break;
    case CMD_PROGRAM_FLASH_ISP:
      SMoISP::ProgramFlash();
      break;
    case CMD_READ_FLASH_ISP:
      SMoISP::ReadFlash();
      break;
    case CMD_PROGRAM_EEPROM_ISP:
      SMoISP::ProgramEEPROM();
      break;
    case CMD_READ_EEPROM_ISP:
      SMoISP::ReadEEPROM();
      break;
    case CMD_PROGRAM_FUSE_ISP:
      SMoISP::ProgramFuse();
      break;
    case CMD_READ_FUSE_ISP:
      SMoISP::ReadFuse();
      break;
    case CMD_PROGRAM_LOCK_ISP:
      SMoISP::ProgramLock();
      break;
    case CMD_READ_LOCK_ISP:
      SMoISP::ReadLock();
      break;
    case CMD_READ_SIGNATURE_ISP:
      SMoISP::ReadSignature();
      break;
    case CMD_READ_OSCCAL_ISP:
      SMoISP::ReadOscCal();
      break;
    case CMD_SPI_MULTI:
      SMoISP::SPIMulti();
      break;
    //
    // HVSP Commands
    //
    case CMD_ENTER_PROGMODE_HVSP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(HIGH);
      #endif
      SMoHVSP::EnterProgmode();
      break;
    case CMD_LEAVE_PROGMODE_HVSP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(LOW);
      #endif
      SMoHVSP::LeaveProgmode();
      break;
    case CMD_CHIP_ERASE_HVSP:
      SMoHVSP::ChipErase();
      break;
    case CMD_PROGRAM_FLASH_HVSP:
      SMoHVSP::ProgramFlash();
      break;
    case CMD_READ_FLASH_HVSP:
      SMoHVSP::ReadFlash();
      break;
    case CMD_PROGRAM_EEPROM_HVSP:
      SMoHVSP::ProgramEEPROM();
      break;
    case CMD_READ_EEPROM_HVSP:
      SMoHVSP::ReadEEPROM();
      break;
    case CMD_PROGRAM_FUSE_HVSP:
      SMoHVSP::ProgramFuse();
      break;
    case CMD_READ_FUSE_HVSP:
      SMoHVSP::ReadFuse();
      break;
    case CMD_PROGRAM_LOCK_HVSP:
      SMoHVSP::ProgramLock();
      break;
    case CMD_READ_LOCK_HVSP:
      SMoHVSP::ReadLock();
      break;
    case CMD_READ_SIGNATURE_HVSP:
      SMoHVSP::ReadSignature();
      break;
    case CMD_READ_OSCCAL_HVSP:
      SMoHVSP::ReadOscCal();
      break;
    //
    // HVPP Commands
    //
    case CMD_ENTER_PROGMODE_PP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(HIGH);
      #endif
      SMoHVPP::EnterProgmode();
      break;
    case CMD_LEAVE_PROGMODE_PP:
      #ifdef _SMoHWIF_led2_
        SMoHWIF::writeLed2(LOW);
      #endif
      SMoHVPP::LeaveProgmode();
      break;
    case CMD_CHIP_ERASE_PP:
      SMoHVPP::ChipErase();
      break;
    case CMD_PROGRAM_FLASH_PP:
      SMoHVPP::ProgramFlash();
      break;
    case CMD_READ_FLASH_PP:
      SMoHVPP::ReadFlash();
      break;
    case CMD_PROGRAM_EEPROM_PP:
      SMoHVPP::ProgramEEPROM();
      break;
    case CMD_READ_EEPROM_PP:
      SMoHVPP::ReadEEPROM();
      break;
    case CMD_PROGRAM_FUSE_PP:
      SMoHVPP::ProgramFuse();
      break;
    case CMD_READ_FUSE_PP:
      SMoHVPP::ReadFuse();
      break;
    case CMD_PROGRAM_LOCK_PP:
      SMoHVPP::ProgramLock();
      break;
    case CMD_READ_LOCK_PP:
      SMoHVPP::ReadLock();
      break;
    case CMD_READ_SIGNATURE_PP:
      SMoHVPP::ReadSignature();
      break;
    case CMD_READ_OSCCAL_PP:
      SMoHVPP::ReadOscCal();
      break;
    // Pseudocommands
    case SMoCommand::kHeaderError:
    case SMoCommand::kChecksumError:
    case SMoCommand::kIncomplete:
      break;  // Ignore
    default:
      SMoCommand::SendResponse(STATUS_CMD_UNKNOWN);
      break;
  }
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
