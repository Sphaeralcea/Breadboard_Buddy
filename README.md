Breadboard Buddy first started as a fun project involving the Arduino, a keyboard and an LED display.  It evolved into something much more interesting that has some practical use.  It uses the Arduino’s ATMega328 programmed in the Arduino IDE combined with a circuit and PCB developed in EasyEDA.   
The Arduino based software application is a Virtual Machine of an abstract, simple CPU architecture with its own machine language, whose instructions are bound to keys on the keyboard.  The device is used somewhat like a programmable calculator but rather than doing math, it drives and tests logic signals, like a primitive automatic test machine, under the control of a program keyed in at the keyboard.  It has command syntax for forming loops, conditional branches, and function calls, and has one NMI type interrupt. Much of the functionality can also be used interactively to control the device or circuit under test.   
It is good for exercising Sparkfun or other maker modules to check them out as well as providing the control and data to digital breadboard projects.  In addition to the native instruction set it has a library of macro calls to do a variety of things from displaying data, measuring voltages, and performing byte sized reads and writes in serial formats such as IIC and RS232.  It can also store programs keyed in for later retrieval and use.   
One interesting feature of this VM is that in addition to the normal linear execution flow of a CPU, it has a finite state machine mode of execution.  In FSM mode, a program is formed by a 
combination of a state transition table and a list of functions that perform state actions.  The state of every input signal and several internal conditions form the state transition arcs and the resulting state machine can have up to 16 states. 
A revision of this project using a higher performing and higher pin-count controller would make this an even more useful tool.  I can also see using this board of a reimagining of it as a trainer for basic computer and circuits classes. 
 
Contents 
KeybrdDisp5.ino:  Arduino source code for the main module of the VM 
Processkey.h: keyboard symbol parser and command formation 
Execute.h:  Keyboard command execution loop 
Xeqsyscmd.h: Expandable system macro routines 
BreadBoardBuddy.pptx: Keyboard Command CheatSheet and additional design detail 
Topview.jpg:  Top view of the completed project 
KeyBrdClose.jpg:  Close up of the keyboard labeling 
Schematic Picture.pdf:  Schematic diagram of the circuit 
PCBPicture.jpg: Picture of the PCB layout 
EasyEDA_SCH_PCB:  Easy EDA project zip file – Schematic and PCB layout
