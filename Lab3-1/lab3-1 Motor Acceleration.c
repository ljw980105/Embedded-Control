/* Lab3.1 - PWM Acceleration*/
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init();
void Set_Pulsewidth(void);
void PCA_ISR ( void ) __interrupt 9;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned int PW_CENTER = 2769; // PulseWidth is about 1.5ms 2769
unsigned int PW_MIN = 2031; // 1.1ms
unsigned int PW_MAX = 3508; // 1.9ms
unsigned int PW = 0;
unsigned int PCA_START = 28614; //65535-36921
unsigned int counter_PCA = 0;

__sbit __at 0xDF CF;

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' ');
    Port_Init();
    XBR0_Init();
    PCA_Init();

    //print beginning message
    printf("Embedded Control Pulsewidth Calibration\r\n");
    PW = PW_CENTER;  // set pw to 1.5ms
    counter_PCA = 0; //reset counter 
    while(counter_PCA < 50);//wait for 1s

    while(1)
        Set_Pulsewidth();
}

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
    P1MDOUT = 0x0D;  //set output pin for CEX0, CEX2 and CEX3 in push-pull mode
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------

// Set up the crossbar
//
void XBR0_Init()
{
    XBR0 = 0x27;  //configure crossbar as directed in the laboratory

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    PCA0MD =0x81; //enable cf interrupt & SYSCLK/12
    PCA0CPM0 = PCA0CPM2 = 0xC2; //Enable CCM2 16bit
    PCA0CN = 0x40; //Enable PCA Counter
    EA = 1; // Enable Global Interrupt
    EIE1 |= 0x08; // Enable PCA Interrupt

}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{
// reference to the sample code in Example 4.5 -Pulse Width Modulation
// implemented using the PCA (Programmable Counter Array), p. 50 in Lab Manual.
    counter_PCA++;
    if (CF){
        CF = 0; // clear overflow flag
        PCA0 = PCA_START; // 20ms period
    }
    PCA0CN &= 0x40;// handle other pca interrupt resources
}

void Set_Pulsewidth()
{
    char input;
    //wait for a key to be pressed
    input = getchar();
    if(input == '+')  // single character input to increase the pulsewidth
    {
        if(PW < PW_MAX){  // check if less than pulsewidth maximum  
			PW += 10;
		} else {
			PW = PW_MIN; // set SERVO_PW to a minimum value
		}
    }
    else if(input == '-')  // single character input to decrease the pulsewidth
    {
        if(PW > PW_MIN){  // check if more than pulsewidth minimum 
		   PW -= 10;
		} else {
			PW = PW_MAX; // set PW to a maximum value
		}
    }
    printf("PW: %u\r\n", PW);
    PCA0CPL2 = 0xFFFF - PW;
	PCA0CPH2 = (0xFFFF - PW) >> 8;

}