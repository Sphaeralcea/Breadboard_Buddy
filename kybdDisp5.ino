// <NEW>
#include <avr/interrupt.h>
// </NEW>

// Revision number
#define MO 0x09
#define YR 0x24

// VrefInt1.1V offset in uV (long int fmt)
#define DeltaVref -19800L

// Keyboard and display pin definitions
#define SData A0
#define SClk A1
#define ColLd 2
#define ColIn A3
#define Rrst A4
#define RCclk A5

// Output and Input signal pin definitions
#define O1 13
#define O2 12
#define O3 11
#define O4 10
#define O5 9
#define O6 8
#define O7 7
#define O8 6
#define O9 5
#define I1 4
#define I2O10 3
#define I3O11 A2
#define O12 1
#define I4 0 

// Special numbers
#define _NUMROWS 4
#define _NUMCOLS 5
#define _NUMDISPDIGITS 4
#define _DEBOUNCETIME_MS 50
#define _CLKPER_US 5
#define _STACKSIZE 256
#define _CSTKSIZE 16
#define _PGMSIZE 85

#define _SMROWS 16
#define _SMCOLS 16

// Shorthand print number and print string 
#define _PN(arg) Serial.print(arg,HEX);Serial.print("h ")
#define _PNL(arg) Serial.print(arg,HEX);Serial.println("h")
#define _PS(arg) Serial.print(arg);Serial.print(" ")
#define _PSL(arg) Serial.println(arg)

// Performance debug macros to count clock cycles - max=65536 clks
#define _STARTCLKCOUNTER TCCR1A=0;TCCR1B=bit(CS10);TCNT1=0
#define _STOPCLKCOUNTER unsigned int _CLKCNTR=TCNT1;Serial.println(_CLKCNTR-1);


void setup() {
  // signals to run the keyboard and display
  pinMode(SData, OUTPUT); digitalWrite(SData, LOW);    // Serial display data
  pinMode(SClk, OUTPUT); digitalWrite(SClk, LOW);      // Serial display clock
  pinMode(RCclk, OUTPUT); digitalWrite(RCclk, LOW);    // Row assert and column shift clock
  pinMode(Rrst, OUTPUT); digitalWrite(Rrst, LOW);      // Row reset
  pinMode(ColLd, OUTPUT); digitalWrite(ColLd, HIGH);   // Column shift register load
  pinMode(ColIn, INPUT);                               // Column shift data in

  // Signals for external use
  pinMode(O1, OUTPUT); digitalWrite(O1, LOW);          // Always outputs
  pinMode(O2, OUTPUT); digitalWrite(O2, LOW);
  pinMode(O3, OUTPUT); digitalWrite(O3, LOW);
  pinMode(O4, OUTPUT); digitalWrite(O4, LOW);
  pinMode(O5, OUTPUT); digitalWrite(O5, LOW);
  pinMode(O6, OUTPUT); digitalWrite(O6, LOW);
  pinMode(O7, OUTPUT); digitalWrite(O7, LOW);
  pinMode(O8, OUTPUT); digitalWrite(O8, LOW);
  pinMode(O9, OUTPUT); digitalWrite(O9, LOW); 

  pinMode(I1, INPUT);                                  // Always input
  pinMode(I2O10, INPUT);                               // Configurable in/out (default input short ckt safety)
  pinMode(I3O11, INPUT);                               // Configurable in/out (default input short ckt safety)
  // O12 and I4 use the same pin as Tx and Rx.
                        
  Serial.begin(9600);                                  // O12/I4 are TxRx by default
}

// Code macros 
//  read 74ls165 shift register output Qh
#define _COLPIN (PINC & B00001000) >> 3

// write data to clock and data pins for display
#define _setSegData(state) (state==LOW)?PORTC=PORTC&~(1<<0):PORTC=PORTC|(1<<0)
#define _setSegClk(state) (state==LOW)?PORTC=PORTC&~(1<<1):PORTC=PORTC|(1<<1)

// assert a value to cd4022 johnson decoder reset pin
#define _setRowRst(state) (state==LOW)?PORTC=PORTC&~(1<<4):PORTC=PORTC|(1<<4)
// assert a value to cd4022 and 74ls165 clock pins
#define _setRCclk(state) (state==LOW)?PORTC=PORTC&~(1<<5):PORTC=PORTC|(1<<5)
// assert a value to 74ls165 parallel data load pin
#define _setColRegLd(state) (state==LOW)?PORTD=PORTD&~(1<<2):PORTD=PORTD|(1<<2)

// pulse the clock that shifts the 75hc595
#define _SEGCLK _setSegClk(HIGH);delayMicroseconds(_CLKPER_US);_setSegClk(LOW)
// pulse the reset pin to the cs4022 johnson decoder
#define _ROWRST _setRowRst(HIGH);delayMicroseconds(_CLKPER_US);_setRowRst(LOW)
// pulse the clock that goes to the cd4022 and the 74ls165
#define _RCCLK _setRCclk(HIGH);delayMicroseconds(_CLKPER_US);_setRCclk(LOW)
// pulse the parallel data load pin on the 74ls165
#define _COLREGLD _setColRegLd(LOW);delayMicroseconds(_CLKPER_US);_setColRegLd(HIGH)

// Load display buffer with literal segment data
#define _DISPMSG(b0, b1, b2, b3) dispBuf[0]=b0;dispBuf[1]=b1;dispBuf[2]=b2;dispBuf[3]=b3; dispPtr=0


// lookup table to translate keypad inputs to a key code
const byte keyTable [] = {
  0xFF,  // position 0 is a dummy location (not accessed)
  0x00, 0x01, 0x02, 0x03, 0x10,  // fnc  
  0x04, 0x05, 0x06, 0x07, 0x11,  // up
  0x08, 0x09, 0x0A, 0x0B, 0x12,  // dn
  0x0C, 0x0D, 0x0E, 0x0F, 0x13,  // ent
};

// lookup table to translate key codes to display segment patterns
const byte digitTable [] = {
// abcdefg.
  B11111100,  // 0
  B01100000,  // 1
  B11011010,  // 2
  B11110010,  // 3
  B01100110,  // 4
  B10110110,  // 5
  B10111110,  // 6
  B11100000,  // 7
  B11111110,  // 8
  B11100110,  // 9
  B11101110,  // A
  B00111110,  // B
  B10011100,  // C
  B01111010,  // D
  B10011110,  // E
  B10001110,  // F
  B01100010,  // fnc   place holder not referenced
  B11000100,  // ^     place holder not referenced
  B00111000,  // v     place holder not referenced
  B10000100   // ent   place holder not referenced
};

///////////////////////////////////////////////////////////////////////
///////////////// Virtual Machine Data Structures /////////////////////
byte pgmPause = 0;          // Prevents instructions from executing
byte pgmRunning = 0;        // 0=interactive, 1=program running, 2+=subroutine running
byte pgmMem[_PGMSIZE][3];   // Program memory
byte pgmPtr = 0;            // Program counter
byte dStack[_STACKSIZE];    // Data stack
byte stackPtr = 0;          // Stack pointer
byte callStack[_CSTKSIZE];  // Subroutine pgmPtr call stack
byte callPtr=0;             // Call stack pointer
byte pgmLbls[16];           // Lable look up table
byte wfEnables[5];          // Waitfor enables
byte wfValues[5];           // Waitfor values
byte clkMux=0;              // System clock enable
int clkPer;                 // System clock period
byte clkdisabled = 0;       // Option to disable clock pulse for select instructions
byte loopCnt = 0;           // Decrement on SKIF loop counter2
byte loopCnt2 = 0;          // Decrement on SKIF loop counter
byte clkCount = 0;          // Decrements on every clock enabled instruction
byte equFlag = 0;           // Indicates equality comparison to TOS
byte grtFlag = 0;           // Indicates greater than comparison to TOS
unsigned long time0;        // Time capture for start of WFOR measure
unsigned long time1;        // Time capture for close of WFOR measure
byte dispBuf[_NUMDISPDIGITS] = {0x1, 0x1, 0x1, 0x1};  // Display buffer
byte dispPtr = 0;           // Display pointer
byte anaRead = 0;           // 8 bit input mode is analog
byte digRead = 0;           // 8 bit input mode is digital
byte addrCntr = 0;          // Output incrementer vlaue
byte cntrSize = 0;          // Output incrementer counter width
byte cntrmskB;              // Mask for writing REGB
byte cntrmskBbar;
byte cntrmskD;              // Mask for writing REGD
byte cntrmskDbar;
byte smEnable = 0;          // Enable for state machine execution mode
byte smState = 0;           // State variable for state machine execution mode
byte openRow = 0;           // Row index for state table programming
byte openCol = 0;           // Column index for state table programming
byte stateTable[_SMROWS][_SMCOLS];  // State machine execution mode state table
volatile byte intrptInProgress = 0; // Interrupt has been requested on I1
volatile byte intrptType = 0;       // rising edge, falling edge
byte breakPtEn = 0;         // Enable break point capability (1), disable (0)
byte breakPtAddr;           // Address to stop execution when break point is enabled

// Function shifts out segment pattern data to the display shift register ('595)
void dispDigit(byte outsegs) {
  // shift 8 bits of segment data out msb first
  for (byte j=0; j<8; j++) {
    (outsegs & 0x01) ? _setSegData(HIGH) : _setSegData(LOW);
    outsegs = outsegs >> 1;  // shift seg dp out first, seg a out last
    _SEGCLK;
  } 
    // need to give '595 one extra clock as shclk and rclk tied together
    _SEGCLK;
}

// ISR to interrupt a running program
void ISR_stop_program () {
  pgmRunning = 0;
}
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#include "execute.h"
#include "processkey.h"

// keyboard logic globals
byte newkey = 0;
byte oldkey = 0;
byte keycode;
int keyevent = 0;
byte keyaccepted = 0;

// keyboard and display driver
void loop() {
  byte row;  // The keyboard row currently asserted.  Also the display digit currenty active
  byte col;  // Holds the vlaues of the column bits sensed from column shift reg
  
  // Sense the keyboard for full cycle of row selects or until key detected
  for (row=1; row<=_NUMROWS; row++ ) {  // Select row to assert
    _ROWRST;
    for (byte i=1; i<=row; i++) {       // Walk to the row to assert
      _RCCLK;
    }

    // Get data on column lines
    _COLREGLD;                           // Xfer column states into shift reg
    for (col=_NUMCOLS; col>0; col--) {   // Read shift reg shr(4)=col1 shr(0)=col5
      if (_COLPIN) break;                // If we find a bit that is set, break out of loop
      _RCCLK;
    }

    // Decode key 
    if (col != 0) {                      // if key detected, decode key and break out of loop
      newkey = ((row-1)*_NUMCOLS)+col;
      break;
    } else newkey = 0;
  }
  
  // Key debounce routine
  if (newkey != oldkey) {    // if the depressed key changed
    keyevent = millis();     // mark the time
    keyaccepted = 0;         // indicate that prevoiusly accepted key has been released
  }
  if ((millis() - keyevent) > _DEBOUNCETIME_MS) { // if debouncetime since last key change, then keyboard is stable
    if (newkey && !keyaccepted) {    // accept the new keypress, we are debounced.
      keyaccepted = 1;               // after keyaccepted, do not accept the same unless previously released
      processkey(keyTable[newkey]);
    }
  }
  oldkey = newkey;

  // render contents of display buffer
  _ROWRST;
  for (byte i=0; i<_NUMDISPDIGITS; i++) {  // loop through display buffer
    _RCCLK;
    dispDigit(dispBuf[i]);
    delay(2);
  }
}

// <NEW>
// NOTE _getIn1 definition is redundant to that in execute.h
#define _getIn1 (PIND&0x10)
ISR(PCINT2_vect) { // PortD PCINT20
    PCMSK2 &= 0b11101111;  // turn PCINT20 off
    if (_getIn1) intrptType = 1;  // set intterupt type
    else intrptType = 0;
    intrptInProgress = 1;
}
// </NEW>
