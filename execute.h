#include <EEPROM.h>

// Instruction execution helpers
// Set signal state for outputs 1 to 6
#define _setOutSig1to6(pin,state) (state==LOW)?PORTB=PORTB&~(B00100000>>pin):PORTB=PORTB|(B00100000>>pin)
// Set signal state for outputs 7 to 9
#define _setOutSig7to9(pin,state) (state==LOW)?PORTD=PORTD&~(B10000000>>pin):PORTD=PORTD|(B10000000>>pin)
// Set signal state for output 10
#define _setOutSig10(state) (state==LOW)?PORTD=PORTD&~(1<<3):PORTD=PORTD|(1<<3)
// Set signal state for output 11
#define _setOutSig11(state) (state==LOW)?PORTC=PORTC&~(1<<2):PORTC=PORTC|(1<<2)
// Set signal state for output 12
#define _setOutSig12(state) (state==LOW)?PORTD=PORTD&~(1<<1):PORTD=PORTD|(1<<1)


// Read intputs 1-4
#define _getIn1 (PIND&0x10)
#define _getIn2 (PIND&0x08)
#define _getIn3 (PINC&0x04)
#define _getIn4 (PIND&0x01)
// Read input 3 with data in bit 0
#define _I3O11 ((PINC & B00000100) >> 2)

// Waitfor instruction trigger statements for each input
#define _waitforIn1 if (wfEnables[0]) { \
                      if (((_getIn1 > 0) == wfValues[0]) && (!in1m1 == wfValues[0])) \
                        wfEnables[0] = 0; \
                      in1m1 = (_getIn1 > 0); }
#define _waitforIn2 if (wfEnables[1]) { \
                      if (((_getIn2 > 0) == wfValues[1]) && (!in2m1 == wfValues[1])) \
                        {wfEnables[1] = 0; time0=time1; time1=micros();}\
                      in2m1 = (_getIn2 > 0); }
#define _waitforIn3 if (wfEnables[2]) { \
                      if (((_getIn3 > 0) == wfValues[2]) && (!in3m1 == wfValues[2])) \
                        wfEnables[2] = 0; \
                      in3m1 = (_getIn3 > 0); }
#define _waitforIn4 if (wfEnables[3]) { \
                      if (((_getIn4 > 0) == wfValues[3]) && (!in4m1 == wfValues[3])) \
                        wfEnables[3] = 0; \
                      in4m1 = (_getIn4 > 0); }
                      
// Pulse the system clock output period width
#define _setSysClk(state) (state==LOW)?PORTB=PORTB&~(1<<5):PORTB=PORTB|(1<<5)
#define _PULSESYSCLK(del) _setSysClk(HIGH);delay(del);_setSysClk(LOW);delay(del)
//                                                           22us compensation on low time for execution loop
#define _PULSESYSCLKUS(del) _setSysClk(HIGH);delayMicroseconds(del);_setSysClk(LOW);delayMicroseconds(del-22)

// Set I/O direction of column load signal
#define _setColRegLdDir(state) (state==LOW)?DDRD=DDRD&~(1<<2):DDRD=DDRD|(1<<2)

// Increment the output counter and output to selected output signals
#define _incrCountIfEn \
          if (cntrSize > 0) { \
            addrCntr++; \
            PORTB = (PORTB&cntrmskBbar) | (addrCntr&cntrmskB); \
            PORTD = (PORTD&cntrmskDbar) | (addrCntr&cntrmskD); \
          }
#define _resetCntr PORTB=PORTB&0xE0;PORTD=PORTD&0x1F;

// display contents of dispBuf for 100*dur (ms)
#define _LIGHTDISP(dur)  for (byte i=1; i<=(5*dur); i++) { \
                           _ROWRST; \
                           for (byte j=0; j<_NUMDISPDIGITS; j++) { \
                             _RCCLK; \
                             dispDigit(dispBuf[j]); \
                             delay(5); \
                           } \
                         }
byte dispDur=0x0F;  // display duration default 1.5sec (15*100ms)

enum opcodes {NUL=0x0, HLT=0x1, POP=0x2, 
              GOTO=0x3, LABL=0x4, XEQ=0x5,
              SET=0x6, WFOR=0x7, SKIF=0x8,
              OPR=0x9, PUSH=0xA, DIN=0xB,
              OCEL=0xC, WRC=0xD,
              CFG=0xE, SYS=0xF};

unsigned long newtime=0;  // for measuring pulse width on input 2
unsigned long oldtime=0;
         long VCCmeasured;  // Measure actual VCC for ADC on I3

// Next state look up function for state machine execution mode
byte stateLookup (byte currentstate) {
  byte arc;
  
  for (byte arc=0; arc<_SMROWS; arc++) {
    if (stateTable[arc][currentstate] != 0) 
      switch (arc) { // first arc in state column that is true AND that has a next state entry wins
        case 0x0: return (stateTable[arc][currentstate]);break;  // Always
        case 0x1: if (_getIn1 == 1) return (stateTable[arc][currentstate]);break;
        case 0x2: if (_getIn2 == 1) return (stateTable[arc][currentstate]);break;
        case 0x3: if (_getIn3 == 1) return (stateTable[arc][currentstate]);break;
        case 0x4: if (_getIn4 == 1) return (stateTable[arc][currentstate]);break;
        case 0x5: if (equFlag == 1) return (stateTable[arc][currentstate]);break;
        case 0x6: if (grtFlag == 1) return (stateTable[arc][currentstate]);break; 
        case 0x7: if (clkCount == 0) return (stateTable[arc][currentstate]);break;
        case 0x8: if ((grtFlag == 0) && (equFlag == 0)) return (stateTable[arc][currentstate]);break;   // Less Than is true
        case 0x9: if (_getIn1 == 0) return (stateTable[arc][currentstate]);break;
        case 0xA: if (_getIn2 == 0) return (stateTable[arc][currentstate]);break;
        case 0xB: if (_getIn3 == 0) return (stateTable[arc][currentstate]);break;
        case 0xC: if (_getIn4 == 0) return (stateTable[arc][currentstate]);break;      
        case 0xD: if (equFlag == 0) return (stateTable[arc][currentstate]);break;
        case 0xE: if (grtFlag == 0) return (stateTable[arc][currentstate]);break;   
        case 0xF: if (clkCount != 0) return (stateTable[arc][currentstate]);break;
      }
  }
}

// execute system command function definition
#include "xeqsyscmd.h"

// Execution procedures for each instruction
void execute (byte IR0, byte IR1, byte IR2) {

  byte digReadVal;
  byte indat;   // temporary read data from inputs, serial
  byte tos;     // top of stack popped into this for OPR command
  
  do {
// <DEBUG>    
//_PS(pgmRunning);_PS(":");_PN(pgmPtr);_PS(" - ");_PN(IR0);_PS(" ");_PN(IR1);_PS(" ");_PNL(IR2);
//_STARTCLKCOUNTER;
// </DEBUG>
    
    clkdisabled = 0;          // increment system clock by default, if enabled
    if (!pgmPause) {

      switch (IR0) {
        case NUL: break;
        case HLT: clkdisabled = 1;
                  if (pgmRunning > 1) {
                    callPtr = (callPtr-1)%_CSTKSIZE; pgmPtr = callStack[callPtr];  // Return from subroutine
                    pgmRunning--;
                  } else if (! smEnable) pgmRunning--;
                  else {  // else (Smen) so don’t let pgmRunning decrement to 0
            // why shouldn't i let pgmRunning decrement?        
                    smState = stateLookup(smState);
                    pgmPtr = pgmLbls[smState];
                  }
                  break;
        case POP: stackPtr--;
                  clkdisabled = 1;   // Disable clock for all modalities except those that affect hardware
                  switch (IR1) {
                    case 0x0: break;                         // Pop into bit bucket
                    case 0x1: clkdisabled = 0;               // Enable clock to advance on pop to hardware
                              dispDigit(dStack[stackPtr]);   // Pop and send data to the display shift register as Dout[7:0]
                              break;
                    case 0x2: clkdisabled = 0;               // Enable clock to advance on pop to hardware
                              dispDigit(dStack[stackPtr]);   // Pop and send data to the display shift register as Dout[7:0]
                              _incrCountIfEn;                // If addrcntr mode enabled, then increment addrcntr
                              break;
                    case 0xB: loopCnt2 = dStack[stackPtr];    // Pop into the loop2 counter
                              break;
                    case 0xD: breakPtAddr = dStack[stackPtr];    // Set the break point address pointer
                              break;
                    case 0xE: clkCount = dStack[stackPtr];   // Pop into system state counter (clock counter)
                              break;
                    case 0xF: loopCnt = dStack[stackPtr];    // Pop into the loop counter
                              break;
                    default: break;                          // all other operands pop the stack into the bit bucket
                  }
                  break;
        case GOTO: clkdisabled = 1; 
                   pgmPtr = pgmLbls[IR1];
                   break;
        case LABL: clkdisabled = 1; 
                   break;
        case SET: if (IR1<=6)_setOutSig1to6((IR1-1),IR2);
                  else if (IR1<=9) _setOutSig7to9((IR1-7),IR2);
                  else if (IR1==10) _setOutSig10(IR2);
                  else if (IR1==11) _setOutSig11(IR2);
                  else if (IR1==12) _setOutSig12(IR2);
                  break;
        case WFOR: if (IR1>=0xC) {  // wait for loop counter else wait for in1-4
                     wfEnables[4]=1; wfValues[4]=IR2;
                   } else {
                     wfEnables[IR1-1]=1; wfValues[IR1-1]=IR2;
                   }
                   pgmPause=1;
                   break;
        case SKIF: switch(IR1) {
                     case 0x1: if ((_getIn1>0) == IR2) pgmPtr++; break;
                     case 0x2: if ((_getIn2>0) == IR2) pgmPtr++; break;
                     case 0x3: if ((_getIn3>0) == IR2) pgmPtr++; break;
                     case 0x4: if ((_getIn4>0) == IR2) pgmPtr++; break;
                     case 0xB: if (loopCnt2 == IR2) pgmPtr++; else loopCnt2--; break;
                     case 0xC: if (grtFlag == IR2) pgmPtr++; break;
                     case 0xD: if (equFlag == IR2) pgmPtr++; break;
                     case 0xE: if (clkCount == IR2) pgmPtr++; break;
                     case 0xF: if (loopCnt == IR2) pgmPtr++; else loopCnt--; break;
                   }
                   break;
        case OPR: clkdisabled = 1;
                  switch (IR1) {   // pop second operand into tos variable, leave operand 1 on stack
                    case 0x0: tos=dStack[--stackPtr];                                 // Tst
                              equFlag = (tos == dStack[stackPtr-1]);  // Test equ
                              grtFlag = (tos > dStack[stackPtr-1]);   // Test gtr
                              break;
                    case 0x1: equFlag = (dStack[stackPtr-1] == 0); break;             // Zero
                    case 0x2: tos=dStack[--stackPtr]; dStack[stackPtr-1]+=tos; break; // Add
                    case 0x3: tos=dStack[--stackPtr]; dStack[stackPtr-1]-=tos; break; // Sub
                    case 0x4: dStack[stackPtr-1] += 1; break;                         // Incr
                    case 0x5: dStack[stackPtr-1] -= 1; break;                         // Decr
                    case 0x6: tos=dStack[--stackPtr]; dStack[stackPtr-1]&=tos; break; // AND
                    case 0x7: tos=dStack[--stackPtr]; dStack[stackPtr-1]|=tos; break; // OR
                    case 0x8: tos=dStack[--stackPtr]; dStack[stackPtr-1]^=tos; break; // XOR
                    case 0x9: dStack[stackPtr-1] = ~dStack[stackPtr-1]; break;        // NOT
                    case 0xA: dStack[stackPtr-1]<<=1; break;                          // SHL
                    case 0xB: dStack[stackPtr-1]>>=1; break;                          // SHR
                    default: break;
                  }
                  break;
        case PUSH: clkdisabled = 1; dStack[stackPtr++] = (IR1*16)+IR2;
                   break;
        case DIN: switch (IR1) {  // read 8 bits from D or A and push onto stack
                    case 0x0: if (digRead) {
                                _COLREGLD;
                                for (byte i=8; i>0; i--) {                // Read shift reg
                                  indat = _I3O11;
                                  digReadVal = (digReadVal << 1) + indat; // shift val left and add I3 bit
                                  _RCCLK;
                                }
                                dStack[stackPtr++] = digReadVal;
                                _incrCountIfEn;
                              } else if (anaRead) {
                                dStack[stackPtr++] = lowByte(analogRead(I3O11)>>2);
                              }
                              break;
                    case 0x1: dStack[stackPtr-1] = (dStack[stackPtr-1]<<1) | (_getIn1 != 0);  // read In1 and shift left the value into the TOS byte
                              break;
                    case 0x2: dStack[stackPtr-1] = (dStack[stackPtr-1]<<1) | (_getIn2 != 0);  // read In2 and SHL into TOS
                              break;
                    case 0x3: dStack[stackPtr-1] = (dStack[stackPtr-1]<<1) | (_getIn3 != 0);  // read In2 and SHL into TOS
                              break;
                    case 0x4: dStack[stackPtr-1] = (dStack[stackPtr-1]<<1) | (_getIn4 != 0);  // read In2 and SHL into TOS
                              break;  
                    case 0x5: dStack[stackPtr++] = ((time1-time0)&0x00001FE0)>>5;   // push time diff (32us to 4.096ms range) of 2 In2 wfor events onto stack
                              clkdisabled = 1;
                              break;
                    case 0x6: dStack[stackPtr++] = ((time1-time0)&0x001FE000)>>13;  // push time diff (8.192ms to 1.05s range) of 2 In2 wfor events onto stack
                              clkdisabled = 1;
                              break;
                    case 0x8: dStack[stackPtr] = dStack[stackPtr-1];   // Copy TOS
                              stackPtr++;
                              clkdisabled = 1;
                              break;
                    case 0x9: dStack[stackPtr++] = addrCntr;
                              clkdisabled = 1;
                              break;
                    case 0xB: dStack[stackPtr++] = loopCnt2;
                              clkdisabled = 1;
                              break;    
                    case 0xE: dStack[stackPtr++] = clkCount;
                              clkdisabled = 1;
                              break;  
                    case 0xF: dStack[stackPtr++] = loopCnt;
                              clkdisabled = 1;
                              break;                                            
                  }
                  break;
        case OCEL: clkdisabled = 1;
                   openRow = IR1;
                   openCol = IR2;
                   break;
        case WRC: clkdisabled = 1;
                  stateTable[openRow][openCol] = IR1;
                  break;
        case CFG: clkdisabled = 1;
                  switch (IR1) {
                         // Configure system clock and freq
                    case 0x0: if (IR2==0) clkMux=0;    // System clock is disabled
                              else {
                                clkMux=1;              // System clock is enabled, compute period/2
                                switch (IR2&0x03) {
                                  case 0x1: clkPer = 1; break; 
                                  case 0x2: clkPer = 2; break;
                                  case 0x3: clkPer = 5; break;
                                }
                                switch(IR2&0xC) {
                                  case 0x0: clkPer *= 500; clkMux=2; break; // 1ms, 2ms, 5ms
                                  case 0x4: clkPer *= 5; break;             // 10ms, 20ms, 50ms
                                  case 0x8: clkPer *= 50; break;            // 100ms, 200ms, 500ms
                                  case 0xC: clkPer *= 500; break;           // 1s, 2s, 5s
                                }
                              }
                              break;
                         // Configure State Machine Mode
                    case 0x1: smEnable = IR2;  
                              break;
                         // Select I2 or O10
                    case 0x2: if (IR2) {  // I2O10  0=input, 1=output
                                pinMode(I2O10, OUTPUT);
                                digitalWrite(I2O10, LOW);
                              }
                              else pinMode(I2O10, INPUT);
                              break;
                         // Select I3 or O11
                    case 0x3: if (IR2) {  // I3O11  0=input, 1=output
                                pinMode(I3O11, OUTPUT);
                                digitalWrite(I3O11, LOW);
                              }
                              else pinMode(I3O11, INPUT);
                              break;
                         // Select I4/O12 or Tx/Rx
                    case 0x4: if (IR2) {  // 0=RxTx, 1=I4O12
                                Serial.end();
                                pinMode(O12, OUTPUT);
                                digitalWrite(O12, LOW);
                                pinMode(I4, INPUT);
                              } else Serial.begin(9600); 
                              break;
                         // Select I3 Input mode.  0: 1 bit digital, 1: 8 bit digital, 2+: 8 bit analog
                    case 0x5: if (IR2==0) {
                                digRead = 0;
                                anaRead = 0;
                              } else if (IR2==1)
                                  digRead = 1;
                                else 
                                  anaRead = 1;
                              break;
                         // Configure Counter Mode Outputs
                    case 0x6: addrCntr = 0;
                              _resetCntr;
                              if (IR2>1) {
                                cntrSize = IR2;
                                switch (IR2) {
                                  case 2: cntrmskB=0x03; cntrmskD=0x00; break;
                                  case 3: cntrmskB=0x07; cntrmskD=0x00; break;
                                  case 4: cntrmskB=0x0F; cntrmskD=0x00; break;
                                  case 5: cntrmskB=0x1F; cntrmskD=0x00; break;
                                  case 6: cntrmskB=0x1F; cntrmskD=0x20; break;
                                  case 7: cntrmskB=0x1F; cntrmskD=0x60; break;
                                  case 8: cntrmskB=0x1F; cntrmskD=0xE0; break;
                                }
                                cntrmskBbar = ~cntrmskB;
                                cntrmskDbar = ~cntrmskD;
                              }
                              break;
                    // Enable I1 to be HW Interrut Request
                    case 0x7: switch (IR2) {
                                 case 0:
                                   cli();
                                   PCICR &= 0b11111011;  // Disables Port D pin change interrupts
                                   PCMSK2 &= 0b11101111; // Disables PCINT20 (I1)
                                   sei();
                                   break;
                                 case 1:
                                   cli();
                                   PCICR |= 0b00000100;  // Enables Port D pin change interrupts
                                   PCMSK2 |= 0b00010000; // Enables PCINT20 (I1)
                                   sei();
                                   break;
                                 }
                              break;  // case 0x8:
                    case 0x8: if (IR2) breakPtEn = 1;
                              break;
                  }
                  break;  // case CFG:
        case SYS: clkdisabled = 1;
                  xeqSysCmd((IR1*16)+IR2);
                  break;
        case XEQ: clkdisabled = 1;
                  if (pgmRunning) {callStack[callPtr] = pgmPtr; callPtr = (callPtr+1)%_CSTKSIZE;} // enter subroutine
                  if (smEnable) smState=IR1;                   // Label target when SMenable is the inital state of FSM
                  pgmPtr = pgmLbls[IR1];   // Begin execution at instruction after target label             
                  pgmRunning++;
                  break;
        default: Serial.print("Invalid Opcode:  ");Serial.println(IR1);
                 break;
      } // switch (IR0)

// <DEBUG>    
//_PS(pgmRunning);_PS(":");_PN(pgmPtr);_PS(" - ");_PN(IR0);_PS(" ");_PN(IR1);_PS(" ");_PNL(IR2);
//_PN(stackPtr);_PS(":");_PN(dStack[stackPtr-1]);_PN(dStack[stackPtr-2]);_PN(dStack[stackPtr-3]);_PNL(dStack[stackPtr-4]);
// </DEBUG>

      // Enable program interrupt button
      _setColRegLdDir(LOW);  //pinMode(ColLd, INPUT);
      EIFR = (1 << INTF0);   //use before attachInterrupt(1,isr,xxxx) to clear interrupt 1 flag
      attachInterrupt(0, ISR_stop_program, RISING);

      if (!clkdisabled) {
        clkCount--;
        if (clkMux)   {           // Pulse clock if not disabled
          if (clkMux==2) {_PULSESYSCLKUS(clkPer);}
          else {_PULSESYSCLK(clkPer);}
        }
      }
      // Disable program interrupt button
      detachInterrupt(0);
      _setColRegLdDir(HIGH); // configure ColLd pin to output
      _setColRegLd(HIGH);

      if (! intrptInProgress) {
      pgmPtr++;               // Advance program counter
      // <NORMAL FLOW>
        if (breakPtEn)        // If brkpten fall into an infinite loop until a key is pressed on IDE serial console
          if (pgmPtr == breakPtAddr) {
            while (! Serial.available()) {
              dispBuf[0]=digitTable[(pgmPtr&0xF0)>>4]; dispBuf[1]=(digitTable[pgmPtr&0x0F] | 0x01); dispBuf[2]=digitTable[(dStack[stackPtr-1]&0xF0)>>4]; dispBuf[3]=digitTable[dStack[stackPtr-1]&0x0F];
              _LIGHTDISP(2);
            }
            indat = Serial.read();Serial.read();   // Read in the key that was hit and then bit bucket the linefeed
            //Serial.println(indat);
            if (indat == 's')         // If The key hit was 's' then single step, otherwise continue
              breakPtAddr++;
          }

      IR0=pgmMem[pgmPtr][0]; IR1=pgmMem[pgmPtr][1]; IR2=pgmMem[pgmPtr][2];    // Load next instruction
      // </NORMAL FLOW>
      } else {
          dStack[stackPtr++] = intrptType;    // store interrupt edge type for ISR
          IR0=0x5; IR1=0xF; IR2=0x0;          // put XEQ F (ISR) in instruction register
          intrptInProgress = 0;
      }

      // Test and clear waitfor conditions if satisfied
    } else {  // if (!pgmPause)                   
      // Check each waitfor input if enabled and clear if satisfied
      if (wfEnables[0]) if ((_getIn1 > 0) == wfValues[0]) wfEnables[0] = 0;  // I1
      if (wfEnables[1]) if ((_getIn2 > 0) == wfValues[1]) {wfEnables[1] = 0; time0=time1; time1=micros();}  // I2
      if (wfEnables[2]) if ((_getIn3 > 0) == wfValues[2]) wfEnables[2] = 0;  // I3 // PINC
      if (wfEnables[3]) if ((_getIn4 > 0) == wfValues[3]) wfEnables[3] = 0;  // I4
      if (wfEnables[4]) if (clkCount == wfValues[4])       wfEnables[4] = 0;  // Clock Counter
                        else clkCount--;
      pgmPause = (wfEnables[4] | wfEnables[3] | wfEnables[2] | wfEnables[1] | wfEnables[0]);
    }  // if (!pgmPause)
 //_STOPCLKCOUNTER;
  } while (pgmRunning>0);  // end do loop

} // end execute function
