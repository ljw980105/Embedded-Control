/*  Names: David Dawnkaski, David Hoddinott, Jing Wei Li
    Section: 4
    Date: 09/22/17
    File name: lab1-2.c
    Description:
*/
/*
  This program demonstrates the use of T0 interrupts. The code will count the
  number of T0 timer overflows that occur while a slide switch is in the ON position.
*/

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);      // Initialize ports for input and output
void Timer_Init(void);     // Initialize Timer 0 
void Interrupt_Init(void); //Initialize interrupts
void Timer0_ISR(void) __interrupt 1;
unsigned char random(void);
int getRandom(void);
void gameControl(void);
int isPBOneOn(void);//function that checks PushButton 1
int isPBTwoOn(void);// function that checks PushButton 2
void set_outputs(void);
void record_inputs(void);
void results(void);
void game_over(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
__sbit __at 0xB3 BILED1;
__sbit __at 0xB4 BILED2;
__sbit __at 0xB6 LED0; // LED0, associated with Port 3 Pin 6
__sbit __at 0xB5 LED1; // LED0, associated with Port 3 Pin 5

__sbit __at 0xB1 PB1;
__sbit __at 0xB1 PB2; // Push button 2, P3.1
__sbit __at 0xA0 SS;    // Slide Switch associated with Port 2 Pin 0
// sbit settings are incomplete, include those developed 
// in Lab 1-1 and add the sbit setting for LED1
unsigned int Counts = 0;
unsigned int tries = 0;
unsigned int wins = 0;
unsigned char scenario;
unsigned char user_input;

__sbit __at 0x8c TR0;// timer 0
__sbit __at 0xA9 ET0;
__sbit __at 0xAF EA;



//***************
void main(void)
{
    Sys_Init();      // System Initialization
    Port_Init();     // Initialize ports 2 and 3 
    Interrupt_Init();
    Timer_Init();    // Initialize Timer 0 

    putchar(' ');    // the quote fonts may not copy correctly into SiLabs IDE
    printf("Start\r\n");

    while (1) /* the following loop prints the number of overflows that occur
                while the pushbutton is pressed, the BILED is lit while the
                button is pressed */
    {
        BILED1 = 0;  // Turn OFF the BILED
        BILED2 = 0;

        while( SS ) { // while SS0 is ON (high)
            set_outputs();
            TR0 = 1; // turn timer on
			TMR0 = 0 ;
            while (Counts <225);//wait for 1s
            record_inputs();
			results();
			Counts = 0;

            if (tries >= 10){
                game_over();
            }
        }
    }
}

void set_outputs(void){
    scenario = random(); // generate random number
    switch (scenario){
        case 0: //light led 0
            LED0 = 0;
			LED1 = 1;
			break;
        case 1: // light led1
            LED1 = 0;
			LED0 = 1;
			break;
        case 2: // light both leds
            LED1 = 0;
            LED0 = 0;
			break;
		default:
			break;
    }
}

void record_inputs(void){
    if (isPBOneOn() && isPBTwoOn() ){ // push button 1 and 2 pressed
        user_input = 2;
    } else if (isPBOneOn()){ // push button 1  pressed
        user_input = 0;
    } else if (isPBTwoOn()){ // push button 2  pressed
        user_input = 1;
    } else { // no user input
        user_input = 3;
    }
}
void results(void){
    if (scenario == user_input){
        wins ++;
        tries ++;
        BILED1 = 0;
        BILED2 = 1;
    } else if(scenario != user_input){
        tries ++;
        BILED1 = 1;
        BILED2 = 0;
    }
}

void game_over(void){
    printf("You have scored %d wins, switch off and on to play again",wins);
    wins = 0;
    tries = 0;
	LED1 = 1;
    LED0 = 1;
    while (SS);
}




//***************
void Port_Init(void)
{
    P2MDOUT &= 0xFE;
    P2 |= ~0xFE;

	P3MDOUT &= 0x03;
	P3MDOUT |= 0xF8;
	P3 |= ~0x03;
}

void Interrupt_Init(void)
{
    EA = 1;
    ET0 = 1; // enable Timer0 Interrupt request using sbit variable
}
//***************
void Timer_Init(void)
{
    CKCON &= ~0x08;  // Timer0 uses SYSCLK/12
    TMOD &= 0xF0 ;   // clear the 4 least significant bits - 13 bit
    TR0 = 0;         // Stop Timer0
    TMR0 = 0;        // Clear high & low byte of T0
}


//***************
void Timer0_ISR(void) __interrupt 1
{
    Counts ++;
}

/******************************************************************************/
/*
  This function demonstrates how to obtain a random integer between 0 and 1 in
  C. You may modify and use this code to get a random integer between 0 and N.
*/

/*return a random integer number between 0 and 1*/
unsigned char random(void)
{	
	unsigned char rand_var = rand() % 3;
    return (rand_var);  // rand returns a random number between 0 and 32767.
                        // the mod operation (%) returns the remainder of 
                        // dividing this value by 2 and returns the result,
                        // a value of either 0 or 1 or 3;
}

int isPBOneOn(void) {
    if (!PB1) return 1;
    else      return 0;
}

int isPBTwoOn(void) {
    if (!PB2) return 1;
    else      return 0;
}
