/* Lab3.2 - Ranger Distance*/
#include <c8051_SDCC.h>
#include <i2c.h>
#include <stdio.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init();
void Set_Pulsewidth(void);
void SMB_Init(void);
void PCA_ISR ( void ) __interrupt 9;
unsigned int ReadRanger();

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned int PCA_START = 28614; //65535-36921
unsigned int counter_PCA = 0;
unsigned int new_range = 0; // flag to determine whether a new range is ready to be read
unsigned int r_count = 0;
unsigned char addr = 0xE0; // the address of the ranger is 0xE0
unsigned int ranger_distance;
unsigned char Data[2];


__sbit __at 0xDF CF;


//-----------------------------------------------------------------------------
// Main Function
/*  Initialize everything
    start while (1) loop
        if 80ms has passed
            call ranger distance function
            print range - (printing every 80ms is more than needed)
 */
void main(void)
{
    Sys_Init();
    putchar(' ');
    Port_Init();
    XBR0_Init();
    PCA_Init();
    SMB_Init();
    while (1){
        if (new_range){ //if 80ms has passed
            ranger_distance = ReadRanger(); // read the range
            //start a ping
			
            Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
            i2c_write_data(addr, 0, Data, 1); // write one byte of data to reg 0 at addr
            new_range = 0; //clear new range flag
            printf("The current range is %d cm \r\n",ranger_distance);
        }
    }
}

//
// Port-Init Set up ports for input and output
//
void Port_Init()
{
    P1MDOUT = 0x0D;  //set output pin for CEX0, CEX2 and CEX3 in push-pull mode
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
void XBR0_Init()
{
    XBR0 = 0x27;  //configure crossbar as directed in the laboratory

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
void PCA_Init(void)
{
    PCA0MD =0x81; //enable cf interrupt & SYSCLK/12
    PCA0CPM0 = PCA0CPM2 = 0xC2; //Enable CCM2 16bit
    PCA0CN = 0x40; //Enable PCA Counter
    EA = 1; // Enable Global Interrupt
    EIE1 |= 0x08; // Enable PCA Interrupt

}

/*
 * Function that reads the distance from the ultrasonic ranger
 */
unsigned int ReadRanger() {
    unsigned char Data[2];
    unsigned int range =0;
    i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
    range = (((unsigned int)Data[0] << 8) | Data[1]);
    return range;
}

void SMB_Init(void){
    SMB0CR = 0x93; // set scl to 100khz
    ENSMB = 1; // enable SMBus
}


//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9 {
// reference to the sample code in Example 4.5 -Pulse Width Modulation
// implemented using the PCA (Programmable Counter Array), p. 50 in Lab Manual.
    counter_PCA++;
    if (CF){
        CF = 0; // clear overflow flag
        PCA0 = PCA_START; // 20ms period
        r_count ++;
        if(r_count>=4){
            new_range = 1; // 4 overflows is about 80 ms
            r_count = 0;
        }
    }
        PCA0CN &= 0x40;// handle other pca interrupt resources
}
