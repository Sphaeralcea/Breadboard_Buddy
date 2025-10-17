
// Map SDA/SCL to output signal numbers:  SDA=O2oc,In1 SCL=O3oc
#define _SDA 1
#define _SCL 2
byte inval;  // temp holder for read data
// debug
#define _tik _setOutSig1to6(3,1);delayMicroseconds(20);_setOutSig1to6(3,0)
// IIC delay quanta (half of an iic clk period).
#define _IICDEL delay(1);
// Macros that implement IIC primatives
// START:  both start at 1.  SDA goes 0 then SCL goes 0.
#define _IICSTART _setOutSig1to6(_SDA,LOW); _setOutSig1to6(_SCL,LOW);_IICDEL; _setOutSig1to6(_SDA,HIGH);_IICDEL; _setOutSig1to6(_SCL,HIGH);_IICDEL;_IICDEL;_IICDEL;_IICDEL;
// STOP:  both start out at 0.  SCL goes 1 then SDA goes 1.
#define _IICSTOP _setOutSig1to6(_SDA,HIGH); _setOutSig1to6(_SCL,HIGH);_IICDEL; _setOutSig1to6(_SCL,LOW);_IICDEL; _setOutSig1to6(_SDA,LOW);_IICDEL;_IICDEL;
// CLK:  delay half T, clk 1, delay half T, clk 0.
#define _IICCLK_HI _setOutSig1to6(_SCL,LOW)
#define _IICCLK_LO _setOutSig1to6(_SCL,HIGH)
#define _IICCLK _IICDEL;_IICCLK_HI;_IICDEL;_IICCLK_LO;
// CMD:                  Dev Addr                                                                     Assert cmd                             Release(Z)               Ack
#define _IICCMD(dev,cmd) for(int i=6; i>=0; i--){_setOutSig1to6(_SDA,(((dev>>i)&0x01)==0));_IICCLK;} _setOutSig1to6(_SDA,(cmd==0));_IICCLK; _setOutSig1to6(_SDA,LOW);_IICCLK;
// REG:              Reg Addr                                                                     Release(Z)               Ack
#define _IICREG(reg) for(int i=7; i>=0; i--){_setOutSig1to6(_SDA,(((reg>>i)&0x01)==0));_IICCLK;} _setOutSig1to6(_SDA,LOW);_IICCLK;
// WR:                     Wr Data                                                                      Release(Z)               Ack
#define _IICWRBYTE(val) for(int i=7; i>=0; i--){_setOutSig1to6(_SDA,(((val>>i)&0x01)==0));_IICCLK;} _setOutSig1to6(_SDA,LOW);_IICCLK;
// RD:             Rd Data into TOS         Tsu/hd           Rd                   Clk                                                                          NACK to end read
#define _IICRDBYTE inval=0; for(byte i=0; i<8; i++){inval<<=1;_IICCLK_HI;_IICDEL;inval+=(_getIn1!=0);_IICDEL;_IICCLK_LO;} dStack[stackPtr++]=inval; _setOutSig1to6(_SDA,LOW);_IICCLK;

// Map SPIO to O9, SPII to In1, SPIClk to O8 
#define _SPIDEL _IICDEL;
#define _SPICLK(lvl) _setOutSig7to9(9-8,lvl)
#define _SPIWR(dat) _setOutSig7to9(9-7,((dat) ? HIGH : LOW));

// Map 1-wire signal to O2oc,In1
#define _1WO 1
// 1W primatives
#define _1WDRIVE0 _setOutSig1to6(_1WO,HIGH)
#define _1WRELEASE _setOutSig1to6(_1WO,LOW)
#define _1WRESET _1WDRIVE0;delayMicroseconds(480);_1WRELEASE;delayMicroseconds(480)
#define _1WWR1 _1WDRIVE0;delayMicroseconds(6);_1WRELEASE;delayMicroseconds(64)
#define _1WWR0 _1WDRIVE0;delayMicroseconds(60);_1WRELEASE;delayMicroseconds(10)
#define _1WWRBYTE(val) for(byte i=0x01; i; i<<1) if(val&i) {_1WWR1;} else {_1WWR0;}
#define _1WRDBYTE(val) for(byte i=0; i<8; i++) {_1WDRIVE0;delayMicroseconds(6);_1WRELEASE;delayMicroseconds(9);val<<=1;val+=_getIn1;delayMicroseconds(55);}

byte dumpnum = 1; // VM debug dump record serial number
int baudrate;     // baudrate for my bit bang rs232

// Save the volatile page to a nonvolatile (EEPROM) page
void saveProgram (byte page) {
  // Up to 4, 256 byte (85 instruction) pages 1-4 
  int destptr = (page-1)*256;      // EEPROM Byte address

  for (byte srcptr=0; srcptr<_PGMSIZE; srcptr++) {
    EEPROM.update(destptr++, pgmMem[srcptr][0]);
    EEPROM.update(destptr++, pgmMem[srcptr][1]);
    EEPROM.update(destptr++, pgmMem[srcptr][2]);
  }
}

// Restore a saved nonvolatile page to the volatile page
void restoreProgram (byte page) {
  int srcptr = (page-1)*256;
  byte opcode = NUL;
  byte label = 0;

  for (byte destptr=0; destptr<_PGMSIZE; destptr++) {
    opcode = EEPROM.read(srcptr++);
    pgmMem[destptr][0] = opcode;
    if (opcode == LABL) pgmLbls[EEPROM.read(srcptr)] = destptr;
    pgmMem[destptr][1] = EEPROM.read(srcptr++);
    pgmMem[destptr][2] = EEPROM.read(srcptr++);
  }
}

// Save the stack image to a nonvolatile page
void saveStack (byte page) {
  // Up to 4, 256 byte pages 1-4 
  int destptr = (page-1)*256;      // EEPROM Byte address
  int srcptr = 0;
  
  for (int i=0; i<_STACKSIZE; i++) {
    EEPROM.update(destptr, dStack[srcptr]);
    srcptr++; destptr++;
  }
}

// Restore the stack image saved in a nonvolatile page to the stack
void restoreStack (byte page) {
  int srcptr = (page-1)*256;
  int destptr = 0;

  for (int i=0; i<_STACKSIZE; i++) {
    dStack[destptr] = EEPROM.read(srcptr);
    destptr++; srcptr++;
  }
}


void xeqSysCmd (byte syscmd){

//  byte syscmd;
  byte sysarg1;
  byte sysarg2;
  byte sysarg3;
  byte sysarg4;
  int sysargint;
  long sysarglong;

  clkdisabled = 1; 
//  syscmd = (IR1*16)+IR2;
  switch (syscmd) {
         // VM Debug Dump
    case 0x00: Serial.print("REC#:    ");Serial.println(dumpnum++);
               Serial.print("RUN PSE: ");Serial.print(pgmRunning); Serial.print(" "); Serial.println(pgmPause);
               Serial.print("CMX PER: ");Serial.print(clkMux); Serial.print(" "); Serial.println(clkPer,HEX);
               Serial.print("PGM STK: ");Serial.print(pgmPtr,HEX); Serial.print(" "); Serial.print(stackPtr,HEX); Serial.print(": "); for (byte i=0; i<8; i++){Serial.print(dStack[i],HEX);Serial.print(" ");}Serial.println();
               Serial.print("LBLS:    ");for (byte i=0; i<16; i++) {Serial.print(pgmLbls[i],HEX);Serial.print(" ");} Serial.println();
               Serial.print("CALLSTK: ");Serial.print(callPtr,HEX);Serial.print(": "); for (byte i=0; i<_CSTKSIZE; i++) {Serial.print(callStack[i],HEX);Serial.print(" ");}Serial.println();
               Serial.print("WFE WFV: ");for (byte i=0; i<4; i++) {Serial.print(wfEnables[i]);Serial.print(wfValues[i]);Serial.print(":");} Serial.println();
               Serial.print("LPC CLC: ");Serial.print(loopCnt,HEX);Serial.print(" ");Serial.println(clkCount,HEX);
               Serial.print("EQU GRT: ");Serial.print(equFlag);Serial.print(" ");Serial.println(grtFlag);
               Serial.print("T0 T1:   ");Serial.print(time0,HEX);Serial.print(" ");Serial.println(time1,HEX);
               Serial.print("DIG ANA: ");Serial.print(digRead); Serial.print(" "); Serial.println(anaRead);
               Serial.print("CSZ CNT: ");Serial.print(cntrSize); Serial.print(" "); Serial.println(addrCntr,HEX);    
               Serial.print("DPT DBF: ");Serial.print(dispPtr);Serial.print(": ");for (byte i=0; i<4; i++) {Serial.print(dispBuf[i],HEX);Serial.print(" ");} Serial.println();            
               Serial.print("DIR BCD: ");Serial.print(DDRB,HEX);Serial.print(" : ");Serial.print(DDRC,HEX);Serial.print(" : ");Serial.println(DDRD,HEX);
               Serial.print("PRT BCD: ");Serial.print(PINB,HEX);Serial.print(" : ");Serial.print(PINC,HEX);Serial.print(" : ");Serial.println(PIND,HEX);
               Serial.print("ROW COL: ");Serial.print(openRow,HEX);Serial.print(" ");Serial.println(openCol,HEX);
               Serial.print("BPE BPA: ");Serial.print(breakPtEn);Serial.print(" ");Serial.println(breakPtAddr,HEX);
               Serial.print("SMEN ST: ");Serial.print(smEnable);Serial.print(" ");Serial.println(smState,HEX);
               if (smEnable)
                 for (byte r=0;r<_SMROWS;r++) {
                   for (byte c=0;c<_SMCOLS;c++) 
                     Serial.print(stateTable[r][c],HEX);
                   Serial.println();
                 }
                 Serial.println();
                 break;
         // Stack Debug Dump
    case 0x01: sysarg1 = 0;
               Serial.print("Ptr: ");Serial.println(stackPtr,HEX);
               while (sysargint <= _STACKSIZE-1) {
                 Serial.print(sysargint,HEX);Serial.print(": ");
                 for (byte j=0; j<=15; j++) {
                   Serial.print(dStack[sysargint++],HEX); Serial.print(" ");
                 }
                 Serial.println();
               }
               break;  
         // Save Page
    case 0x02: sysarg1 = dStack[--stackPtr];  // Pop source label
               sysarg2 = dStack[--stackPtr];  // Pop destination page
               saveProgram(sysarg1);
               break;
         // Restore Page
    case 0x03: sysarg1 = dStack[--stackPtr];  // Pop source page
               restoreProgram(sysarg1);
               break;
         // Save Stack Image
    case 0x04: sysarg1 = dStack[--stackPtr];    // Pop destination page
               saveStack(sysarg1);
               break;
         // Restore Stack Image
    case 0x05: sysarg1 = dStack[--stackPtr];    // Pop source page
               restoreStack(sysarg1);
               break;
         //
         // Display Message Literal (pops stack x4)
    case 0x10: sysarg1 = dStack[--stackPtr]; sysarg2 = dStack[--stackPtr];
               sysarg3 = dStack[--stackPtr]; sysarg4 = dStack[--stackPtr];
               _DISPMSG(sysarg4, sysarg3, sysarg2, sysarg1);
               _LIGHTDISP(dispDur);  // Display for (def) 1.5 sec
               break;
         // Display Message Hex (pops stack)
    case 0x11: sysarg1 = dStack[--stackPtr];
               _DISPMSG(0, 0, digitTable[(sysarg1&0xF0)>>4], digitTable[sysarg1&0x0F]);
               _LIGHTDISP(dispDur);  // Display for (def) 1.5 sec
               break;
         // Display Long Message Hex (pops stack X2)
    case 0x12: sysarg1 = dStack[--stackPtr];
               sysarg2 = dStack[--stackPtr];
               _DISPMSG(digitTable[(sysarg2&0xF0)>>4], digitTable[sysarg2&0x0F], digitTable[(sysarg1&0xF0)>>4], digitTable[sysarg1&0x0F]);
               _LIGHTDISP(dispDur);  // Display for (def) 1.5 sec
               break;
         // Insert Time Delay (0ms-255ms) (pops stack)
    case 0x13: sysarg1 = dStack[--stackPtr];
               delay(sysarg1);
               break;
         // Display Message Illumination Duration (x100ms)
    case 0x14: dispDur = dStack[--stackPtr];
               break;
         // Display "HErE" and hang -- For Debug
    case 0x15: _DISPMSG(0x6E,0x9E,0x0A,0x9E);  // "HErE"
               _LIGHTDISP(dispDur);
               while (1) delay(2000);
               break;
         // Display implicit message "----"
    case 0x16: _DISPMSG(0x02,0x02,0x02,0x02);  // "----"
               _LIGHTDISP(dispDur);
               break;
                       // Display TOS no pop
    case 0x17: sysarg1 = dStack[stackPtr-1];
               _DISPMSG(0, 0, digitTable[(sysarg1&0xF0)>>4], digitTable[sysarg1&0x0F]);
               _LIGHTDISP(dispDur);
               break;
         //
         // Serial Byte Out:  uses O9 as data and O8 as clock
    case 0x20: sysarg1 = dStack[--stackPtr];
               for (byte i=1; i<=8; i++) {
                 _setOutSig7to9(2,((sysarg1&0x80)>0));  // O9 data
                 sysarg1 <<= 1;                          // Shift out from msb to lsb.
                 delayMicroseconds(50);
                 _setOutSig7to9(1, 1);                   // O8 clk
                 delayMicroseconds(50);
                 _setOutSig7to9(1, 0);
               }
               break;
         // Serial Byte In:  uses I1 as data and O8 as clock
    case 0x21: for (byte i=1; i<=8; i++) {
                 sysarg1 = sysarg1 << 1;   // Shift in from lsb to msb
                 if (_getIn1) sysarg1 |= 1;             // I1 data
                   delayMicroseconds(50);
                   _setOutSig7to9(1, 1);                  // O8 clk
                   delayMicroseconds(50);
                   _setOutSig7to9(1, 0);
                 }
                 dStack[stackPtr++] = sysarg1;
                 break;
         // Set Baud rate on HW serial (Tx, Rx)
    case 0x22: sysarg1 = dStack[--stackPtr];
               // 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
               // 300,  600,  1200, 2400, 4800, 9600 (default 9600_
               Serial.begin(long (sysarg1*300));    // setup
               break;
         // Serial avail (Rx, Tx); put number of available bytes on stack
    case 0x23: dStack[stackPtr++] = Serial.available();
               break;
         //
         // HW RS232 Out
    case 0x24: sysarg1 = dStack[--stackPtr];
               Serial.write(sysarg1);
               Serial.flush();
               break;
         //  HW RS232 In
    case 0x25: for (int i=0; i<900; i++) {  // wait up to 5 sec for serial data
                 if (Serial.available()) {
                   dStack[stackPtr++] = Serial.read();
                   break;  // break out of for loop
                 }
                 delay(10);
               }
               break;
         // HW RS232 data in from monitor (reads ASCII string)
    case 0x26:  while (! Serial.available()) delay(100);  // wait for serial data present
                while (Serial.available()) dStack[stackPtr++]=Serial.read();
                break;
         // Initialize SW RS232 on I1 and O9 and set Baud rate
    case 0x27: sysarg1 = dStack[--stackPtr];
               // 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
               // 300,  600,  1200, 2400, 4800, 9600  (no default, must be configured)
               baudrate = 3333;      // starts at 300 baud (3333us)
               while (sysarg1>1) {
                 baudrate >>= 1; sysarg1 >>= 1;
               }
               digitalWrite(O9, HIGH);
               break;
         //
         //0x28    reserved for future avail check
         //
         // SW RS232 Out
    case 0x29: sysarg1 = dStack[--stackPtr];
               _setOutSig7to9(2,0);             // assert start bit on O9
               delayMicroseconds(baudrate);
               for (byte i=1; i>0; (i<<=1)) {
                 sysarg2 = ((sysarg1&i)>0);     // shift out lsb to msb
                 _setOutSig7to9(2,sysarg2);     // data out on O9
                 delayMicroseconds(baudrate);
               }
               _setOutSig7to9(2,1);             // assert idle state on O9
               delayMicroseconds(baudrate);  
               break;
         // SW RS232 In (blocks indefinitly for input)  
    case 0x2A: while (_getIn1 != 0) {;}         // wait for start bit leading edge
               delayMicroseconds(baudrate/2);   // sample at the middle of a period
               for (byte i=1; i<=8; i++) {
                  delayMicroseconds(baudrate);
                  sysarg1 = sysarg1 >> 1;        // Shift in from lsb to msb
                  if (_getIn1) sysarg1 |= 0x80;  // In1
               }
               delayMicroseconds(baudrate);     // wait one stop bit
               delayMicroseconds(baudrate/2);   // delay until end of 10th period
               dStack[stackPtr++] = sysarg1;
               break;
         // IIC Write
    case 0x2B: sysarg1 = dStack[--stackPtr];    // wr data
               sysarg2 = dStack[--stackPtr];    // reg addr
               sysarg3 = dStack[--stackPtr];    // dev addr
               _IICSTART;
               _IICCMD(sysarg3, 0);             // write cmd
               _IICREG(sysarg2);                // reg addr
               _IICWRBYTE(sysarg1);             // wr data
               _IICSTOP;
               break;
          // IIC Read
    case 0x2C: sysarg1 = dStack[--stackPtr];    // reg addr
               sysarg2 = dStack[--stackPtr];    // dev addr
               _IICSTART;
               _IICCMD(sysarg2, 0);             // dev addr
               _IICREG(sysarg1);                // reg addr
               _IICDEL;
               _IICSTART;                       // repeat start
               _IICCMD(sysarg2, 1);             // read cmd
               _IICRDBYTE;                      // read byte, push on stack
               _IICSTOP;
               break;
         // SPI Out
    case 0x2D: sysarg1 = dStack[--stackPtr];    // send 8 bits out, msb first
               for (byte i=0x80; i; i >>= 1) {
                 // Shift-out bit msb to lsb on SPIO (signal 9)
                 _SPIWR(sysarg1 & i);
                 // Delay for at least the peer's setup time
                 _SPIDEL;
                 // Pull the SPICLK line high (signal 8)
                 _SPICLK(HIGH);
                 // Shift-in a bit from the MISO line
                 //if (_getIn1 == HIGH)
                 //  dStack[stackPtr] |= i;
                 // Delay for at least the peer's hold time
                 _SPIDEL;
                 // Pull the SPICLK low
                 _SPICLK(LOW);
               }
               break;
         // SPI In
    case 0x2E: dStack[stackPtr]=0;         // take 8 bits in, msb first
               for (byte i=0x80; i; i >>= 1) {
                 // Shift-out bit msb to lsb on SPIO (signal 9)
                 // _setOutSig7to9(9-7,((sysarg1 & i) ? HIGH : LOW));
                 // Delay for at least the peer's setup time
                 _SPIDEL;
                 // Pull the SPICLK line high (signal 8)
                 _SPICLK(HIGH);
                 // Shift-in a bit from the MISO line
                 if (_getIn1 == HIGH)
                   dStack[stackPtr] |= i;
                // Delay for at least the peer's hold time
                 _SPIDEL;
                 // Pull the SPICLK low
                 _SPICLK(LOW);
               }
               stackPtr++;
               break;
         // 1-Wire Write 
    case 0x2F: sysarg1 = dStack[--stackPtr];    // number of bytes to write
               _1WRESET;
               for (byte i=sysarg1; i>0; i--) {
                 _1WWRBYTE(dStack[--stackPtr]);
               }
               break;
         // 1-Wire Read
    case 0x30: sysarg1 = dStack[--stackPtr];    // number of bytes to read
               _1WRESET;
               for (byte i=sysarg1; i>0; i--) {
                 _1WRDBYTE(sysarg2);
                 dStack[stackPtr++]=sysarg2;
               }
               break;
         //
         // Configure PWM pins O3,O4,O5,O8*,O9*
    case 0x40: sysarg1 = dStack[--stackPtr];  // duty cycle
               sysarg2 = dStack[--stackPtr];  // signal:  03,04,05,08,09
               //Serial.print(14-sysarg1);Serial.print(" ");Serial.println(sysarg2);
               if (sysarg2<10) analogWrite((14-sysarg2),sysarg1);
               break;
         // Display VCC in mV.
    case 0x41: ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // set ref to AVcc, select 1.1V int ref to ADC input
               delay(2);
               ADCSRA |= _BV(ADSC);               // Start A/D
               while (bit_is_set(ADCSRA,ADSC));   // Wait until done
               //Serial.println(ADCL|(ADCH<<8));
               // Vcc(mV) = [[1100(mV) * 1024] + DeltaRef(uV)] / ADC
               sysarglong = (1126400L + (DeltaVref)) / (ADCL|(ADCH<<8)); // Back-calculate AVcc in mV
               //Serial.print("VCC=");Serial.print(sysarglong);Serial.println("mV");
               sysarg1 = sysarglong/1000; sysarglong -= (sysarg1*1000);
               sysarg2 = sysarglong/100; sysarglong -= (sysarg2*100);
               sysarg3 = sysarglong/10; sysarglong -= (sysarg3*10);
               sysarg4 = sysarglong;
               _DISPMSG(digitTable[(sysarg1&0x0F)], digitTable[(sysarg2&0x0F)], digitTable[(sysarg3&0x0F)], digitTable[sysarg4&0x0F]);
               _LIGHTDISP(50);  // Display for 5 sec
               break;
         //
         // TEST:  Clk Freq Ctl:  T/2(uS)=arg1+arg2+arg3+arg4
    case 0xF0: sysargint = dStack[--stackPtr];
               sysargint += dStack[--stackPtr];
               sysargint += dStack[--stackPtr];
               sysargint += dStack[--stackPtr];
               clkMux = 2;
               clkPer = sysargint;
               break;
         // TEST:  ADC Read using 1.1V Internal Ref
    case 0xF1: analogReference(INTERNAL);
               delay(2);
               analogRead(I3O11);analogRead(I3O11);
               sysargint = analogRead(I3O11);
               _PNL(sysargint);
               break;
        // TEST: Make a 50us sync pulse on out7
    case 0xF2: _setOutSig7to9(0, 1);                  // O7 sync pulse
               delayMicroseconds(50);
               _setOutSig7to9(0, 0);
               break;
         // Display revision number
    case 0xFF: _DISPMSG(digitTable[(MO&0xF0)>>4], digitTable[MO&0x0F]|0x01, digitTable[(YR&0xF0)>>4], digitTable[YR&0x0F]);
               _LIGHTDISP(25);  // Display for 2.5 sec
               break;
  }  // switch SYScmd
}
