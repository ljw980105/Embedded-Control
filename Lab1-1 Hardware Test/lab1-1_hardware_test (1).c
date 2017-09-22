/*
	Use this code to test your hardware
	At the first prompt to push a key on the keyboard, the code will perform the following
		1) Turn the Buzzer on
		2) Turn the BiLED green/red (depending on you you oriented it)
		3) Turn the LED on
		4) Indicate the status of each pushbutton and the slideswitch
	At the second prompt
		1) All outputs are turned off
*/
#include <c8051_SDCC.h>
#include <stdio.h>

// global variables
unsigned char input; // keyboard character
unsigned char port_byte;

// function prototypes
void Port_Init(void);
void first_press(void);
void second_press(void);


void main(void)
{
	Sys_Init();
	putchar(' ');
	Port_Init();	//initialize digital I/O

	while(1)
	{
		first_press();

		second_press();
	}
}


// The following initialization is not correct with regard to your laboratory.
// You must use bit-masking rather than direct assignments
void Port_Init(void)
{
	P2MDOUT = 0x00;
	P2 |= 0xFF;
	P3MDOUT = 0xFC;
	P3 |= 0x03;
}

void first_press(void)
{
	printf("Push a character on the keyboard to turn on outputs and report status of inputs \r\n");
	input = getchar(); // wait for character from keyboard
	printf(" \r\n \r\n");
	P3 &= 0x03;
	P3 |= 0x08;
	port_byte = P3;
	if (port_byte & 0x01)
	{
		printf("Pushbutton 1 is 'up' \r\n");
	}
	else
	{
		printf("Pushbutton 1 is 'down' \r\n");
	}
	if (port_byte & 0x02)
	{
		printf("Pushbutton 2 is 'up' \r\n");
	}
	else
	{
		printf("Pushbutton 2 is 'down' \r\n");
	}
	port_byte = P2;
	if (port_byte & 0x01)
	{
		printf("Slideswitch is 'off' \r\n");
	}
	else
	{
		printf("Slideswitch is 'on' \r\n");
	}
}

void second_press(void)
{
	printf("Push a character on the keyboard to turn off all outputs \r\n");
	input = getchar(); // wait for character from keyboard
	printf(" \r\n \r\n");
	P3 |= ~0x03;
}