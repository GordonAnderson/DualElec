#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "DualElec.h"
#include "Errors.h"
#include "Serial.h"
#include "ADC.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

//
// Gordon Anderson
//
// The Dual electrometer hardware used an Adafruit ItsyBitsy M4 processor.
// This is a very simple app that runs both the ADC modules to record and 
// average the 2 electrometer channels. The system is configured as a TWI 
// slave and passes the values on request.
//
// On M4 board, jumper A1 to A3
//  
// To do:
//

// Wire slave does not work, details at this link:
//    https://github.com/adafruit/ArduinoCore-samd/issues/45
// The following 4 lines of code fix the issus on M4 processors
//void SERCOM2_0_Handler() { Wire.onService(); }
//void SERCOM2_1_Handler() { Wire.onService(); }
//void SERCOM2_2_Handler() { Wire.onService(); }
//void SERCOM2_3_Handler() { Wire.onService(); }

const char   Version[] PROGMEM = "Dual Electrometer version 1.0, May 3, 2021";
DualElecData  dualelecdata;

bool ReturnAvalible = false;

SerialBuffer sb;

DualElecData Rev_1_dualelecdata = {
  sizeof(DualElecData), "DualElec", 1,
  0x20,
  SIGNATURE
};

// System variables
State state;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_dualelecdata, DualElecData);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

// This function is called when the master asks for data.
// Send up to 32 bytes from the sb structure
void requestEventProcessor(void)
{
  int num = sb.available();

  if (ReturnAvalible)
  {
    ReturnAvalible = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;
  }
  for (int i = 0; i < num; i++)
  {
    if (i >= 30) break;
    Wire.write(sb.read());
  }
}

// This function is called when the master asks for data.
// Send up to 32 bytes.
void requestEvent(void)
{
  requestEventProcessor();
}

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  int i;
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendWord(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 16 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 24 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  // Send the float to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint8_t cmd;
  int i, j, off, count, startI, stopI;
  int8_t b;
  int16_t shortint;
  float fval;
  unsigned long tm;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &Serial;
      else PutCh(cmd);
    }
    else switch (cmd)
      {
        case TWI_SERIAL:
          serial = &sb;
          break;
        case TWI_CMD:
          // Process command using the serial processor
          sb.clear();
          serial = &sb;
          tm = millis();
          // Read the ascii string and place in the serial processor ring buffer.
          while (true)
          {
            if (Wire.available() != 0)
            {
              cmd = Wire.read();
              PutCh(cmd);
              if (cmd == '\n') break;
            }
            if ((tm + 250) < millis())
            {
              // Timeout, so put a \n in the command ring buffer and exit
              PutCh('\n');
              break;
            }
          }
          // Process any commands fill sb buffer with results
          while (RB_Commands(&RB) > 0) ProcessSerial();
          serial = &Serial;
          break;
        case TWI_READ_ADC:
          digitalWrite(12,HIGH);
          digitalWrite(12,LOW);
          SendWord(LastADCval[0]);
          SendWord(LastADCval[1]);
          break;
        default:
          break;
      }
  }
}

// This function is called at 40 Hz
void Update(void)
{
  float val;

}

void ElectrometerAD5593init(int8_t addr)
{
  // General purpose configuration, set range for 5 volts
  AD5593write(addr, 3, 0x0130);
  // Set ext reference
  AD5593write(addr, 11, 0x0200);
  // Set LDAC mode
  AD5593write(addr, 7, 0x0000);
  // Set DAC outputs channels
  AD5593write(addr, 5, 0x003C);
  // Set ADC input channels
  AD5593write(addr, 4, 0x0003);
  // Turn off all pulldowns
  AD5593write(addr, 6, 0x0000);

  // Set all DACs to zero
  AD5593writeDAC(addr, 2, 10000);
  AD5593writeDAC(addr, 3, 10000);
  AD5593writeDAC(addr, 4, 0);
  AD5593writeDAC(addr, 5, 0);
}

void setup()
{
  // Read the flash config contents and test the signature
  dualelecdata = flash_dualelecdata.read();
  if(dualelecdata.Signature != SIGNATURE) dualelecdata = Rev_1_dualelecdata;
  // Init serial communications
  SerialInit();
  // Init the TWI interface
  //Wire.begin();
  // Setup TWI as slave to communicate with MIPS.
  Wire.begin(dualelecdata.TWIadd);         // join i2c bus
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  //ElectrometerAD5593init(0x11);
  // Init the DIO
  analogReadResolution(12);
  analogWriteResolution(12);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  state.update = true; // force full update.
  ADCchangeDet(ADC0);
  ADCchangeDet(ADC1);
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

void loop()
{
  ProcessSerial();
  control.run();
}

//
// Host command functions
//

void SaveSettings(void)
{
  dualelecdata.Signature = SIGNATURE;
  flash_dualelecdata.write(dualelecdata);
  SendACK;
}

void RestoreSettings(void)
{
  static DualElecData ded;

  // Read the flash config contents and test the signature
  ded = flash_dualelecdata.read();
  if (ded.Signature == SIGNATURE) dualelecdata = ded;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;
}

void Software_Reset(void)
{
  NVIC_SystemReset();
}

void FormatFLASH(void)
{
  flash_dualelecdata.write(Rev_1_dualelecdata);
  SendACK;
}

void ReadADC(int chan)
{
  SendACKonly;
  serial->println(GetADCvalue(chan, 20));
}

void ReadAD5593(int chan)
{
  SendACKonly;
  serial->println(AD5593readADC(0x11, chan, 4));
}

void Debug(int i)
{
  for (int j = 0; j < i; j++)
  {
    serial->print(LastADCval[0]);
    serial->print(",");
    serial->println(LastADCval[1]);
    delay(5);
  }
}
