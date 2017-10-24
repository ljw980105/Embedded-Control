/*  Names: Jing Wei Li
    Section: 4
    Date: 9/15/17
    File name: lab1-1
    Program description:
*/
#include <c8051_SDCC.h> // include files. This file is available online
#include <stdio.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);  // Initialize ports for input and output
int isSlideSwitchOn(void); // function that checks the Slide switch
int isPBOneOn(void);//function that checks PushButton 1
int isPBTwoOn(void);// function that checks PushButton 2
void Set_outputs(void);// function to set output bits
void testInputs(void);

//-----------------------------------------------------------------------------
// Global Variables - outputs 1
//-----------------------------------------------------------------------------
__sbit __at 0xB6 LED0; // LED0, associated with Port 3 Pin 6
__sbit __at 0xB3 BILED0; // BILED0, associated with P3.3
__sbit __at 0xB4 BILED1; // BILED1, associated with P3.4
__sbit __at 0xB7 BUZZER; // Buzzer, associated with P3.7

//inputs 0
__sbit __at 0xA0 SS;   // Slide switch, associated with Port 2 Pin 0
__sbit __at 0xB0 PB1;  // Push button 1, associated with Port 3, Pin 0
__sbit __at 0xB1 PB2; // Push button 2, P3.1


//***************
void main(void)
{
    Sys_Init();        // System Initialization
    putchar(' ');      // the quote fonts may not copy correctly into SiLabs IDE
    Port_Init();       // Initialize ports 2 and 3

    while (1) {         // infinite loop
        // main program manages the function calls
        Set_outputs();
    }
}


//***************
/* Port_Init - Initializes Ports 2 and 3 in the desired modes for input and output */

void Port_Init(void) {
    // Port 3
    P3MDOUT &= 0xFC; // set Port 3 output pins to push-pull mode (fill in the blank)
    P3MDOUT |= 0xD8; // set Port 3 input pins to open drain mode (fill in the blank) // use to be 0x98
    P3 |= ~0xFC; // set Port 3 input pins to high impedance state (fill in the blank)

    // Port 2
    P2MDOUT &= 0xFE;
    P2 |= ~0xFE;
}

//***************
// Set outputs:
//    The following code is incomplete, lighting an LED depending
//    on the state of a single pushbutton.

/*
 * Group Member 3
1. When the Slide switch is ‘off’ (input is a HIGH voltage), all outputs are off
2. When the Slide switch is ‘on’, the LED is on
3. When the Slide switch is ‘on’ and both Pushbuttons are pushed, the Buzzer is turned on
4. When the Slide switch is ‘on’ and only Pushbutton 1 is pushed, the BiLED is red
5. When the Slide switch is ‘on’ and only Pushbutton 2 is pushed, the BiLED is green
 */

void Set_outputs(void) {
	testInputs();

    if (!isSlideSwitchOn()) { // slide switch is off
        printf("\r All outputs are off \n");
        LED0 = 1;
        BILED0 = 0;
        BILED1 = 0;
        BUZZER = 1;
    }

    if (isSlideSwitchOn()){ // slide switch is on
        printf("\r LED is On \n");
        LED0 = 0;
    } else { // turn LED off is slide switch is off
        LED0 = 1;
    }

    if (isSlideSwitchOn() && isPBOneOn() && isPBTwoOn()){
        printf("\r Turning on Buzzer, both pushbutton pressed \n");
        BUZZER = 0;
    } else {
        BUZZER = 1;
    }

    if (isSlideSwitchOn() && !isPBOneOn() && isPBTwoOn()){
        printf("\r BILED is green, pushbutton 2 pressed\n");
        BILED0 = 0;
        BILED1 = 1;
    } else if (isSlideSwitchOn() && isPBOneOn() && !isPBTwoOn()){
        printf("\r BILED is red, pushbutton 1 pressed\n");
        BILED0 = 1;
        BILED1 = 0;
    } else {
        BILED0 = 0;
        BILED1 = 0;

    }
}

//***************
// Sensor - Returns a 0 if Pushbutton 1 not activated
//          or a 1 if Pushbutton 1 is activated.
//          This code reads a single input only, associated with PB0
// Note this code is not used by function yet, you must incorporate it
int isPBOneOn(void) {
    if (!PB1) return 1;
    else      return 0;
}

int isPBTwoOn(void) {
    if (!PB2) return 1;
    else      return 0;
}

//***************
// Sensor - Returns a 0 if Slide Switch is 'off'
//          or a 1 if Slide switch  is 'on'
//          This code reads a single input only, associated with SS
int isSlideSwitchOn(void) {
    if (!SS) return 1;
    else     return 0;
}

void testInputs(void){
	if (isSlideSwitchOn()){
		printf("\r SlideSwitch is ON \n");
	}
	if (isPBOneOn()){
		printf("\r PUSHBUTTON 1 IS ON	\n");
	} else {
		printf("\r PushBUtton 1 is OFF");
	}
	if (isPBTwoOn()){
		printf("\r PUSHBUTTON 2 IS ON	\n");
	} else {
		printf("\r PushBUtton 2 is OFF");
	}
}

