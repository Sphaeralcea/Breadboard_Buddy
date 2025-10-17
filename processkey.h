
// keyboard state machine helpers
// Reset the cmd assembler state queue
#define _CLRSTATEQ for (byte i=0; i<8; i++) stateQ[i]=0; sQptr=0
// Put opcode into command accumulator
#define _PUTOPC accumulator[accumPtr++]=key;dispBuf[dispPtr++]=digitTable[key];dispBuf[dispPtr++] = B00000010
// Put operand into command accumulator
#define _PUTOPR accumulator[accumPtr++]=key;dispBuf[dispPtr++]=digitTable[key]
// Put the dash after the opcode in the display buffer
#define _DISPDASH dispBuf[dispPtr]=B00000010
// The backspace operation on the accumulator and display buffer
#define _DISPBSP accumPtr--;accumulator[accumPtr]=0;dispPtr--;dispBuf[dispPtr]=0
// Reset the accumulator
#define _CLEARACCUM accumulator[0]=0;accumulator[1]=0;accumulator[2]=0; accumPtr=0
// Display the current program counter
#define _DISPROGADDR dispBuf[0]=0; dispBuf[1]=0; dispBuf[2]=digitTable[pgmPtr>>4]; dispBuf[3]=digitTable[pgmPtr&0x0F]; dispPtr=0
// Display the current stack pointer
#define _DISPSTAKADDR dispBuf[0]=0; dispBuf[1]=0; dispBuf[2]=digitTable[stackPtr>>4]; dispBuf[3]=digitTable[stackPtr&0x0F]; dispPtr=0
// Display instruction opcode at current program counter
#define _DISPINST dispBuf[0]=digitTable[pgmMem[pgmPtr][0]]; dispBuf[1]=B00000010; dispBuf[2]=digitTable[pgmMem[pgmPtr][1]]; dispBuf[3]=digitTable[pgmMem[pgmPtr][2]]; dispPtr=0 
// Display data at current stack pointer
#define _DISPDATA dispBuf[0]=digitTable[dStack[stackPtr]>>4]; dispBuf[1]=digitTable[dStack[stackPtr]&0x0F]; dispBuf[2]=0; dispBuf[3]=0; dispPtr=0
#define _DISPSTACK dispBuf[0]=digitTable[stackPtr>>4]; dispBuf[1]=(digitTable[stackPtr&0x0F]|0x01); dispBuf[2]=digitTable[dStack[stackPtr]>>4]; dispBuf[3]=digitTable[dStack[stackPtr]&0x0F]; dispPtr=0

//////////////////////////////// Keyboard Parser State Tables ///////////////////////////////////
enum symbols {
  IDL=0, OP0=1, OP1=2, O2O=3, O2I=4, OPC=5, OPD=6, LBL=7,
  SIG=8, WFS=9, NB1=10, LVL=11, NB2=12, PER=13, FNC=14, FN2=15,
  ENT=16, BSP=17, ADR=18, UPP=19, DWN=20, NRM=21, PGM=22, SRV=23, Err=24
};

#define _STCOLS 20
#define _STROWS 25

// Parser state tables stored in program flash memory
const byte STbl [3][_STROWS][_STCOLS] PROGMEM = {

 // Normal (interactive) mode (0) state table
 // 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F   fnc   ^    v   ent
 {{OP0, OP0, OP1, OP1, OP1, OP1, O2O, O2I, O2I, OPC, OPD, OP1, OPD, OP1, OPD, OPD, FNC, Err, Err, Err}, //IDL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //OP0
  {LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, FN2, Err, Err, Err}, //OP1
  {Err, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, Err, Err, Err, FN2, Err, Err, Err}, //O2O
  {Err, WFS, WFS, WFS, WFS, Err, Err, Err, Err, Err, Err, WFS, WFS, WFS, WFS, WFS, FN2, Err, Err, Err}, //O2I
  {PER, PER, PER, PER, PER, PER, PER, PER, PER, PER, PER, PER, Err, Err, Err, Err, FN2, Err, Err, Err}, //OPC
  {NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, FN2, Err, Err, Err}, //OPD
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //LBL
  {LVL, LVL, LVL, LVL, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, Err}, //SIG
  {LVL, LVL, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, Err}, //WFS
  {NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, FN2, Err, Err, Err}, //NB1
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //LVL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //NB2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //PER
  {Err, PGM, SRV, ADR, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //FNC
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, BSP}, //FN2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, ENT}, //ENT
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //BSP
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //ADR
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //UPP
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //DWN
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //NRM
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //PRG
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //SRV
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}}, //Err
 
 // Program mode (1) state table
 // 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F   fnc   ^    v   ent
 {{OP0, OP0, OP1, OP1, OP1, OP1, O2O, O2I, O2I, OPC, OPD, OP1, OPD, OP1, OPD, OPD, FNC, UPP, DWN, Err}, //IDL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //OP0
  {LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, LBL, FN2, Err, Err, Err}, //OP1
  {Err, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, SIG, Err, Err, Err, FN2, Err, Err, Err}, //O2O
  {Err, WFS, WFS, WFS, WFS, Err, Err, Err, Err, Err, Err, WFS, WFS, WFS, WFS, WFS, FN2, Err, Err, Err}, //O2I
  {PER, PER, PER, PER, Err, PER, PER, PER, Err, PER, PER, PER, Err, PER, PER, PER, FN2, Err, Err, Err}, //OPC
  {NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, FN2, Err, Err, Err}, //OPD
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //LBL
  {LVL, LVL, LVL, LVL, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, Err}, //SIG
  {LVL, LVL, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, Err}, //WFS
  {NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, FN2, Err, Err, Err}, //NB1
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //LVL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //NB2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //PER
  {NRM, Err, SRV, ADR, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //FNC
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, BSP}, //FN2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, ENT}, //ENT
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //BSP
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //ADR
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, UPP, DWN, IDL}, //UPP
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, UPP, DWN, IDL}, //DWN
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //NRM
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //PRG
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //SRV
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}},//Err

 // Stack Review mode (2) state table
 // 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F   fnc   ^    v   ent
 {{NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, NB1, FNC, UPP, DWN, Err}, //IDL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //OP0
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //OP1
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //O2O
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //O2I
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //OPC
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //OPD
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //LBL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //SIG
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //WFS
  {NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, NB2, FN2, Err, Err, Err}, //NB1
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //LVL
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, FN2, Err, Err, ENT}, //NB2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, ENT}, //PER
  {NRM, PGM, Err, ADR, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //FNC
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, BSP}, //FC2
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, ENT}, //ENT
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err}, //BSP
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //ADR
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, UPP, DWN, IDL}, //UPP
  {Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, Err, UPP, DWN, IDL}, //DWN
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //NRM
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //PRG
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}, //SRV
  {IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL, IDL}} //Err
};

enum stables {NORM=0, PROG=1, SREV=2};

// keyboard parser state machine globals
byte state = IDL;
byte mode = NORM;
byte stateQ[8];  // hold current parser state for back space handling (its really a stack)
byte sQptr = 0;
byte accumulator[3];
byte accumPtr = 0;

// state table lookup function
byte nextState (byte key) {
  return(pgm_read_word_near(&STbl[mode][state][key]));
}

// With new key, look up next state of parser machine and take state actions
void processkey(byte key) {

//  Serial.print("mode=");Serial.print(mode);Serial.print("  prevstate=");Serial.print(state);Serial.print("  key=");Serial.print(key);
  stateQ[sQptr] = state;  // push current state into back space que
  sQptr = (sQptr+1)&0x7;
  state = nextState(key);
//  Serial.print("  state=");Serial.println(state);
  switch (state) {
    case IDL: switch (mode) {
                case NORM: _DISPMSG(0x01,0x01,0x01,0x01); _CLRSTATEQ; break;
                case PROG: _DISPROGADDR;
                           break;
                case SREV: _DISPSTACK; break;
              } 
              break;
    case OP0: 
    case OP1: 
    case O2O:
    case O2I:
    case OPC:
    case OPD: _PUTOPC; dispBuf[2]=0; dispBuf[3]=0;    // Put opcode into accumulator
              break;
    case LBL:
    case LVL:
    case SIG:
    case WFS:
    case NB2:
    case PER: _PUTOPR;     // Put operand into accumulator
              break;
    case NB1: if (mode==SREV) {_DISPMSG(0,0,0,0);} _PUTOPR; 
              break;
    case FNC: _DISPDASH; 
              break;
    case FN2: break;
    case NRM: _DISPMSG(0xEC,0x3A,0x0A,0x2A); mode=NORM;   // display "Norm"
              break;
    case PGM: _DISPMSG(0xCE,0x0A,0x3A,0xF6); mode=PROG;   // display "Prog"
              break;
    case SRV: _DISPMSG(0xB6,0x0A,0x9E,0x38); mode=SREV;   // display "Srev"
              break;
    case BSP: _DISPBSP; sQptr = (sQptr-3)&0x7; state=stateQ[sQptr]; if (sQptr==0) {dispPtr=0; dispBuf[0]=0;}   // backspace implementation
              break;
    case ADR: if (mode==PROG) {_DISPROGADDR;}
              else if ((mode==NORM) || (mode==SREV)) {_DISPSTAKADDR;} 
              break;
    case UPP: if (mode==PROG) {
                if (pgmPtr==0) pgmPtr=_PGMSIZE-1;
                else pgmPtr = (pgmPtr-1)%_PGMSIZE;
                _DISPINST;
              } 
              else if (mode==SREV) {stackPtr = (stackPtr-1)%_STACKSIZE; _DISPSTACK;}
              break;
    case DWN: if (mode==PROG) {
                if (pgmPtr==(_PGMSIZE-1)) pgmPtr=0; 
                else pgmPtr = (pgmPtr+1)%_PGMSIZE;
                _DISPINST;
              }
              else if (mode==SREV) {stackPtr = (stackPtr+1)%_STACKSIZE; _DISPSTACK;}
              break;
    case ENT: switch (mode) {
                case NORM: execute(accumulator[0],accumulator[1],accumulator[2]); _DISPMSG(0x1,0x1,0x1,0x1); 
                           break;
                case PROG: if (accumulator[0] == LABL) pgmLbls[accumulator[1]] = pgmPtr;
                           pgmMem[pgmPtr][0] = accumulator[0];
                           pgmMem[pgmPtr][1] = accumulator[1];
                           pgmMem[pgmPtr][2] = accumulator[2];
                           pgmPtr = (pgmPtr+1)%_PGMSIZE;
                           _DISPROGADDR;
                           break;
                case SREV: dStack[stackPtr] = (accumulator[0]*16) + accumulator[1];
                           stackPtr = (stackPtr+1)%_STACKSIZE;
                           _DISPSTACK; 
                           break;
              }
              state=IDL; _CLRSTATEQ; _CLEARACCUM; 
                         break;
    case Err: _DISPMSG(0x9E,0x0A,0x3A,0x0A);       // Display "Error"
              break;
  }
  //Serial.print("  SQ="); for (byte i=0; i<=6; i++) {Serial.print(stateQ[i]); Serial.print(" ");}
  //Serial.print(" sQptr=");Serial.println(sQptr);
}
