/* Sample code for Lab 3.1. This code provides a basic start. */
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
unsigned int PW_CENTER; //= _____;
unsigned int PW_MIN; //= _____;
unsigned int PW_MAX; //= _____;
unsigned int PW = 0;

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    PCA_Init();

    //print beginning message
    printf("Embedded Control Pulsewidth Calibration\n");
    // set the PCA output to a neutral setting
    //__________________________________________
    //__________________________________________
    PW = PW_CENTER;
    //__________________________________________
    //__________________________________________
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
    P1MDOUT |= ;  //set output pin for CEX0 or CEX2 in push-pull mode
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
    PCA0CPM2 = 0xC2; //Enable CCM2 16bit
    PCA0CN = 0x40; //Enable PCA Counter
    EIE1 |= 0x08; // Enable PCA Interrupt
    EA = 1; // Enable Global Interrupt

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
}

void Set_Pulsewidth()
{
    char input;
    //wait for a key to be pressed
    input = getchar();
    if(input == '+')  // single character input to increase the pulsewidth
    {
        // ___________________________
        // ___________________________
        //if(PW > PW_MAX)  // check if greater than pulsewidth maximum
        //PW = _______;    // set SERVO_PW to a minimum value
    }
    else if(input == '-')  // single character input to decrease the pulsewidth
    {
        // ___________________________
        // ___________________________
        //if(PW > _______)  // check if less than pulsewidth minimum
        //PW = _______;     // set PW to a maximum value
    }
    printf("PW: %u\n", PW);
    PCA0CP1 = 0xFFFF - PW;

}