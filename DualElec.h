#ifndef Fragmentor_h
#define Fragmentor_h
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

#define ESC   27
#define ENQ   5

// TWI commands and constants

#define TWI_SERIAL          0x27      // This command enables the TWI port to process serial commands
#define TWI_CMD             0x7F      // This command sends a ascii string to the serial command processor
                                      // and returns the response through the TWI interface

#define TWI_READ_ADC        0x81      // Returns ADC raw counts for channel 1 and channel 2, 2 16 bit words

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "Fragmentor"
  int8_t        Rev;                    // Holds the board revision number
  int           TWIadd;
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} DualElecData;

typedef struct
{
  bool          update;
} State;


// Prototypes...
void ProcessSerial(bool scan = true);
void SaveSettings(void);
void RestoreSettings(void);
void Software_Reset(void);
void FormatFLASH(void);
void Debug(int i);
void ReadADC(int chan);
void ReadAD5593(int chan);

#endif
