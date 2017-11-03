/* Lab3.3 - Speed Controller Using Ranger*/
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
void PreventExtreme(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned int PW;
unsigned int PCA_START = 28614; //65535-36921
unsigned int PW_CENTER = 2769; // PulseWidth is about 1.5ms 2769
unsigned int PW_MIN = 2031; // 1.1ms full reverse
unsigned int PW_MAX = 3508; // 1.9ms full forward
unsigned int counter_PCA = 0;
unsigned int new_range = 0; // flag to determine whether a new range is ready to be read
unsigned int r_count = 0;
unsigned char addr = 0xE0; // the address of the ranger is 0xE0
unsigned int ranger_distance;
unsigned char Data[2];

__sbit __at 0xDF CF;
__sbit __at 0xB6 RANGER_SS;


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

    printf("Embedded Control Pulsewidth Calibration\r\n");
    PW = PW_CENTER;  // set pw to 1.5ms
    counter_PCA = 0; //reset counter
    while(counter_PCA < 50);//wait for 1s

    while (1){
        if (new_range){ //if 80ms has passed
            ranger_distance = ReadRanger(); // read the range
            //start a ping
            Data[0] = 0x51; // write 0x51 to reg 0 of the ranger:
            i2c_write_data(addr, 0, Data, 1); // write one byte of data to reg 0 at addr
            new_range = 0; //clear new range flag

			// Continuous loop control using ranger
        	Set_Pulsewidth();
        }
    }
}

void Set_Pulsewidth()
{
    if(RANGER_SS){ // if slide switch is off
        PW = PW_CENTER;
    } else { // if slide switch is on
        if (ranger_distance <= 10){ // full forward
            PW = PW_MAX;
        }
        if (ranger_distance >= 40 && ranger_distance <= 50){//neutral
            PW = PW_CENTER;
        }
        if (ranger_distance >= 90){ // full reverse
            PW = PW_MIN;
        }
        if (ranger_distance > 10 && ranger_distance < 40){//linear between max forward and neutral
            PW = -1 * 24.63 * ranger_distance + 3754;
        }
        if (ranger_distance > 50 && ranger_distance < 90){ //linear between max reverse and neutral
            PW = -1 * 18.45 * ranger_distance + 3692;
        }
    }
    PreventExtreme(); // ensures that the PW is within the required range
    PCA0CPL2 = 0xFFFF - PW;
    PCA0CPH2 = (0xFFFF - PW) >> 8;

	printf("The current range is %d cm\r\n",ranger_distance);
    printf("The current pulse width is %d\r\n",PW);
}

void PreventExtreme(){
    if (PW > PW_MAX){
        PW = PW_MIN;
    }
    if (PW < PW_MIN){
        PW = PW_MAX;
    }
}


//
// Port-Init Set up ports for input and output
//
void Port_Init()
{
    P1MDOUT = 0x0D;  //set output pin for CEX0, CEX2 and CEX3 in push-pull mode
    P3MDOUT &= ~0x70; // Set 3.6 and 3.7 to input 
    P3 |= 0x70; // Set 3.6 and 3.7 to high impedance 
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
