/*  Names: David Dawnkaski, David Hoddinott, Jing Wei Li
    Section: 4
    Date: 10/03/17
    File name: lab2.c
    Description:
*/

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

void Port_Init(void);      // Initialize ports for input and output
void Timer_Init(void);     // Initialize Timer 0
void Interrupt_Init(void); //Initialize interrupts
void Timer0_ISR(void) __interrupt 1;
unsigned char read_AD_input(unsigned char n);
void ADC_Init(void);
unsigned char random(void);
int isPBZeroOn(void);// function that checks PushButton 0
int isPBOneOn(void);//function that checks PushButton 1
int isPBTwoOn(void);// function that checks PushButton 2
int isPBThreeOn(void);//function that checks PushButton 3
unsigned char determine_wait_time(unsigned char ADResult);
void mode1(void);
void mode2(void);
void turn_off_LEDs(void);
void random_to_LED(unsigned char rand);
void wait_n_seconds(float n);


// global variables TODO: Complete Global variables init
unsigned int Counts = 0;
unsigned int scores = 0;
unsigned int total_scores = 0;
unsigned char input, last;
unsigned int wait_time,bit1, bit2, bit3, time_elapsed;
unsigned int loop_count = 0;
unsigned int i = 0;

// SBIT Variables
__sbit __at 0x8c TR0;// timer 0
__sbit __at 0xA9 ET0;
__sbit __at 0xAF EA;

__sbit __at 0x91 POT;  // potentiometer at p1.1

__sbit __at 0xA4 PB3; // pushbutton 3 at P2.4
__sbit __at 0xA2 PB1; // pushbutton 1 at P2.2
__sbit __at 0xA3 PB2; // pushbutton 2 at P2.3
__sbit __at 0xA1 PB0; // pushbutton 0 at P2.1
__sbit __at 0xA0 SS; // slide switch at P2.0

__sbit __at 0xB6 LED0; // LED0 at port 3.6
__sbit __at 0xB7 BUZZER; // BUZZER at port 3.7
__sbit __at 0xB4 BILED1; // BILED1 at p3.4
__sbit __at 0xB5 LED1; // LEd1 at p3.5
__sbit __at 0xB3 BILED2; // BILED2 at p3.3
__sbit __at 0xB1 LED2; // LEd2 at p3.1
__sbit __at 0xB0 LED3; // LEd3 at p3.0

/* This program demonstrates how to perform an A/D Conversion */
void main() {
    unsigned char result;
    Sys_Init(); /* Initialize the C8051 board */
    Port_Init(); /* Configure ports for analog input */
    putchar(' ');      // the quote fonts may not copy correctly into SiLabs IDE
    ADC_Init(); /* Initialize A/D conversion */
    Interrupt_Init();
    Timer_Init();    // Initialize Timer 0

    while (1) { // begin infinite loop lol
        // turn off the LEDS, BILEDS and buzzer
        turn_off_LEDs();
        BUZZER= 1;
        TR0 = 0;
        printf("\r\n instructions to be completed later");
        while(!isPBZeroOn()); // wait until the first pushbutton is pressed
        result = read_AD_input(1); /* Read the A/D value on P1.0 */
        wait_time = determine_wait_time(read_AD_input(1)); // get the A/D value, then convert it to wait time
        if (SS){
           mode1();
        } else {
            mode2();
        }
    }
}



void Port_Init(void)
{
    P2MDOUT &= ~0x1F;
    P2 |= 0x1F;
    P3MDOUT |= 0xFB;
    P1MDIN &= ~0x02;
    P1MDOUT &= ~0x02;
    P1 |= 0x02;
}


void ADC_Init(void)
{
    REF0CN = 0x03; /* Set Vref to use internal reference voltage (2.4V) */
    ADC1CN = 0x80; /* Enable A/D converter (ADC1) */
    ADC1CF |= 0x01; /* Set A/D converter gain to 1 */
}


unsigned char read_AD_input(unsigned char n)
{
    AMX1SL = n; /* Set P1.n as the analog input for ADC1 */
    ADC1CN = ADC1CN & ~0x20; /* Clear the “Conversion Completed” flag */
    ADC1CN = ADC1CN | 0x10; /* Initiate A/D conversion */
    while ((ADC1CN & 0x20) == 0x00); /* Wait for conversion to complete */
    return ADC1; /* Return digital value in ADC1 register */
}

/*
 * Generate a random number between 0 - 7
 */
unsigned char random(void) {
    unsigned int rand_var = rand() % 8;
    while (last == rand_var) {
        rand_var = rand() % 8;
        //printf("rand while loop\r\n");
    }
    //printf("last: %d, randvar: %d\r\n",last,rand_var);
    last = rand_var;
    return rand_var;
}


void Interrupt_Init(void) {
    EA = 1;
    ET0 = 1; // enable Timer0 Interrupt request using sbit variable
}

void Timer0_ISR(void) __interrupt 1 {
    Counts ++;
}

void Timer_Init(void)
{
    CKCON &= ~0x08;  // Timer0 uses SYSCLK/12
    TMOD &= 0xF0 ;   // clear the 4 least significant bits - 13 bit
    TMOD |= 0x01;
    TR0 = 0;         // Stop Timer0
    TMR0 = 0;        // Clear high & low byte of T0
}

unsigned char determine_wait_time(unsigned char ADResult){
    unsigned char result = ADResult;
    //TODO: complete wait time conversion
    return result;
}

/*
 * convert the decimal input to 3-bit and lit leds accordingly
 */
void random_to_LED(unsigned char rand){
    unsigned int bits[3] = {0,0,0};// stores the 3 bits of the led
    unsigned int loc = 3;

    while (1){
        unsigned int remainder = rand % 2;
        rand = rand / 2; // integer division: remainder discarded
        if (rand == 0) break; // stops conversion when the rand is 0
        loc -- ;
        bits[loc] = remainder;
    }
    //lit leds by looping thru the bits array
	for (i = 0; i < 3; i ++){
		if (i == 0 && bits[i] == 1 ){
            LED0 = 0;
        } else if (i == 1 && bits[i] == 1){
            LED1 = 0;
        } else if (i == 2 && bits[i] == 1){
            LED2 = 0;
        }
	}
}

void wait_n_seconds(float n){
    Counts = 0;
    while (Counts < n * 225);
}

void mode1(){
    unsigned int total_score = 0;
    printf("\r\n mode1 : instructions to be completed");
    turn_off_LEDs();
    loop_count = 0;
    while (loop_count < 8) {
        unsigned char rand_num = random();//generate a number between 0 and 7
        random_to_LED(rand_num);
        time_elapsed = Counts;
        input = getchar();
        time_elapsed = Counts - time_elapsed; // get the time elasped between displaying LED and pressing
        if (input == rand_num){ // biled green
            BILED1 = 0;
            BILED2 = 1;
        } else { // biled red
            BILED1 = 1;
            BILED2 = 0;
        }

        if (time_elapsed > wait_time){
            scores = 0;
        } else {
            scores = 10 - (10 * time_elapsed)/wait_time;
            total_score += scores;
        }
        printf("\r\nTry score is %d and total score is %d",scores,total_score);
        wait_n_seconds(0.5); //delay 0.5s
        BILED1 = 1;
        BILED2 = 1; // turn off biled
        loop_count ++;
    }
    printf("\r\nThe final score is %d",total_score);
    BUZZER = 0; // turn on buzzer
    wait_n_seconds(0.5);
    BUZZER = 1;

}

void mode2(){
    printf("\r\n mode2 : instructions to be completed");
    turn_off_LEDs();
    loop_count = 0;
    while (loop_count < 8) {
        unsigned char rand_num = random();//generate a number between 0 and 7


        loop_count ++;
    }
}

int isPBZeroOn(void) {
    if (!PB0) return 1;
    else      return 0;
}

int isPBOneOn(void) {
    if (!PB1) return 1;
    else      return 0;
}

int isPBTwoOn(void) {
    if (!PB2) return 1;
    else      return 0;
}
int isPBThreeOn(void) {
    if (!PB3) return 1;
    else      return 0;
}

void turn_off_LEDs(void){
    BILED1 = 1;
    BILED2 = 1;
    LED0 = 1;
    LED1 = 1;
    LED2 = 1;
    LED3 = 1;
}