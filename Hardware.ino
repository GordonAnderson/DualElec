#include "Hardware.h"
#include "AtomicBlock.h"

// Reads the selected ADC channel for the number of averages defined by num
int GetADCvalue(int chan, int num)
{
  int i=0,j;

  for(j=0;j<num;j++) i += analogRead(chan);
  return i/num;
}

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// This function reads and returns an ADC channel value. The raw ADC
// value is read and converted to engineering units.
float ReadADCchannel(ADCchan adch)
{
  int adc = GetADCvalue(adch.Chan,20);
  return Counts2Value(adc,&adch);
}

// Adafruit Itsybity M4 Only: Set-up digital pin D7 to 50KHz PWM output
// and set to near 100% duty cycle 
void setupD10pwm()
{
  // Set up the generic clock (GCLK7) to clock timer TCC0
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(4) |       // Divide the 48MHz clock source by divisor 3: 100MHz/2 = 50MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         //GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization 

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 perhipheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC0

  // Enable the peripheral multiplexer on pin D10
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
 
  // Set the D10 (PORT_PA20) peripheral multiplexer to peripheral (even port number) E(6): TCC0, Channel 0, 6 = group G
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);
 
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 16, 50MHz/1 = 50MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization
 
  TCC0->PER.reg = 999;                               // Set-up the PER (period) register 50KHz PWM
  while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization
 
  TCC0->CC[0].reg = 740;                             // close to 100%
  while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}

// Set PWM to 0 to 999
void setD10pwm(int value)
{ 
  if(value < 0) value = 0;
  if(value > 999) value = 999;
  TCC0->CCBUF[0].reg = value;
}

// Set drive to 0 to 100%
void setDrive(float percent)
{
  int value;
  
  if(percent < 0.0)   percent = 0.0;
  if(percent > 100.0) percent = 100.0;
  value = (MAXDRIVE * (100.0 - percent)) / 100.0;
  if(value >= MAXDRIVE)
  {
    digitalWrite(ENADRV, HIGH);
    value = MAXDRIVE - 1;
    setD10pwm(value);
    return;
  }
  if(value < 0) value = 0;
  setD10pwm(value);
  digitalWrite(ENADRV, LOW);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  static String sToken;
  static uint32_t FlashAddress;
  static int    numBytes,fi,val,tcrc;
  static char   c,buf[3],*Token;
  static byte   fbuf[256],b,crc;
  static byte   vbuf[256];
  static uint32_t start;

  crc = 0;
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  fi = 0;
  FlashClass fc((void *)FlashAddress,numBytes);
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      fc.erase((void *)FlashAddress, 256);
      fc.write((void *)FlashAddress, fbuf, 256);
      // Read back and verify
      fc.read((void *)FlashAddress, vbuf, 256);
      for(int j=0; j<256; j++)
      {
        if(fbuf[j] != vbuf[j])
        {
           interrupts();
           serial->println("FLASH data write error!");
           SendNAK;
           return;   
        }
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    fc.erase((void *)FlashAddress, fi);
    fc.write((void *)FlashAddress, fbuf, fi);
    // Read back and verify
    fc.read((void *)FlashAddress, vbuf, fi);
    for(int j=0; j<fi; j++)
    {
      if(fbuf[j] != vbuf[j])
      {
         interrupts();
         serial->println("FLASH data write error!");
         SendNAK;
         return;   
      }
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}

// AD5593 IO routines. This is a analog and digitial IO chip with
// a TWI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5593
// Return 0 if no error else an error code is returned
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val)
{
  int iStat;
  
  //AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write(pb);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  //ReleaseTWI();
  return (iStat);
}

// Read from AD5593R
// returns -1 on any error
int AD5593readWord(uint8_t addr, uint8_t pb)
{
  int iStat;
  
  //AcquireTWI();
  Wire.beginTransmission(addr);
  Wire.write(pb);
  {
    //AtomicBlock< Atomic_RestoreState > a_Block;
    iStat = Wire.endTransmission();
  }
  if(iStat != 0)
  {
    //ReleaseTWI();
    return (-1);
  }
  // Now read the data word
  int i = 0,j = 0;
  Wire.requestFrom(addr, (uint8_t)2);
//  while(Wire.available())
//  {
//     if(i==0) j = Wire.read() << 8;
//     if(i==1) j |= Wire.read();
//     i++;
//  }
  j = Wire.read() << 8;
  j |= Wire.read();
  //ReleaseTWI();
  return(j);
}

int AD5593readADC(int8_t addr, int8_t chan)
{
   int iStat;
   
   // Select the ADC channel number
   if((iStat = AD5593write(addr, 0x02, (1 << chan))) != 0) return(-1);
   // Read the data and make sure the address is correct then left 
   // justify in 16 bits
   int i = AD5593readWord(addr, 0x40);
   if(((i >> 12) & 0x7) != chan) return(-1);
   i <<= 4;
   return(i & 0xFFFF);
}


int AD5593readADC(int8_t addr, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5593readADC(addr, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

int AD5593writeDAC(int8_t addr, int8_t chan, int val)
{
   uint16_t  d;
   // convert 16 bit DAC value into the DAC data data reg format
   d = (val>>4) | (chan << 12) | 0x8000;
   return(AD5593write(addr, 0x10 | chan, d));
}
