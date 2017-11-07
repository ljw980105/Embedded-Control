/*
Lab 3_2
David Hoddinott
Electronic compass
*/
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void XBR0_Init();
void PCA_ISR ( void ) __interrupt 9;
void i2c_Init();
unsigned int ReadCompass (void);
void adjust_pw(void);


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char addr;
unsigned char CompassData[2];
unsigned int heading;
unsigned char new_heading = 0;
unsigned char heading_count = 0;
unsigned int print_count = 0;
unsigned int desired_heading = 900;
signed int error;
unsigned int center_pw = 2740;
unsigned int PW;
unsigned int toadj;
//unsigned int SS;

__sbit __at (0xB7) CompassSS; 

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
	i2c_Init();

	while(1)
	{
		if (!CompassSS) // if SS is on
		{
			error = desired_heading - heading; // set error
			if (new_heading) // 40 ms passed
			{
				heading = ReadCompass(); // set heading to heading reported by electronic compass
			
				if (print_count > 5) // only print out every 5th reading
				{
					printf("\r\n heading is %d",heading);
					printf("\r\n desired heading is %d",desired_heading);
					print_count = 0; // reset print counter
					printf("\r\n Error is %d",error);
					printf(" \r\n current PW: %u\n\r", PW);
					toadj = .35*(error) + center_pw;
					printf("\r\n the pulse width is now being adjusted to %d",toadj);
				
				}
				
				print_count++; // incriment print count
				adjust_pw(); // run adj pw function 
				new_heading = 0;

			}
		}
		else //SS is not on
		{
			PW = center_pw; // put wheels straight
			PCA0CPL0 = 0xFFFF - PW;
    		PCA0CPH0 = (0xFFFF - PW) >> 8;
			printf("\r\n turn on the ss for compass readings and adjustments"); // print out a message
		}
	}
}
   

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
   P1MDOUT |= 0x0D;  //set output pin for CEX0 or CEX2 in push-pull mode
   P3MDOUT &= ~0x70;
   P3 |= 0x70;	
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{
    XBR0 = 0x27;  //configure crossbar as directed in the laboratory

}

void i2c_Init()
{
	SMB0CR = 0x93;
	ENSMB = 1;
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    PCA0MD = 0x81;
	PCA0CPM0 = 0xC2;
	EIE1 |= 0x08;
	PCA0CN = 0x40;
	EA = 1;
	

}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{
    if (CF)// if flag 
	{
		CF = 0;// reset flag
		
		heading_count++;
		if(heading_count>=2) // approx 40 ms
		{
			new_heading = 1; // flag new heading can be read
			heading_count = 0; // reset pca count timer
		}
	}
	PCA0CN &= 0xC0;
}

unsigned int ReadCompass()
{
	addr = 0xC0; // adress of compass
	i2c_read_data(addr,2,CompassData,2);   //adress, byte to start, where to story, how many bytes to read
	heading = ((CompassData[0] << 8) | CompassData[1]); // turn 2 8-bit into one 16 bit
	return heading;
}

void adjust_pw()
{	
	if (error > 1800) // if your error is too high, reset it low. this keeps moves efficient
	{
		error = error-3600;
	}
	else if (error < -1800) // if error is too high, reset low. this keeps moves efficient
	{
		error = error +3600;
	}
	
	
	PW = .35*(error) + center_pw; // set new PW	
	PCA0CPL0 = 0xFFFF - PW;
    PCA0CPH0 = (0xFFFF - PW) >> 8;
	
}
