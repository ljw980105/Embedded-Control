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
void SMB_Init(void);
void PCA_ISR ( void ) __interrupt 9;
unsigned int ReadCompass (void);
unsigned int ReadRanger();
void PreventExtreme(void);
unsigned char read_AD_input(unsigned char n);
void ADC_Init(void);
void update_ranger(void);
void update_compass(void);
void updateRangerArray(void);
unsigned int rangerCompareLess(unsigned int limit);
unsigned int rangerCompareMore(unsigned int limit);

void maintainHeading(void);
void getDesiredHeading(void);
void getDerivativeGain(void);
void getProportionalGain(void);
void makeThrustVertical(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char __xdata input, pw_percentage; // using __xdata to store large variables
unsigned char __xdata CompassData[2], RangerData[2];
unsigned char new_heading = 0;
unsigned char new_range = 0;
unsigned char print_flag = 0;
unsigned char heading_count = 0;
unsigned char ranger_count = 0;
unsigned char increment_flag = 0;
unsigned char increment_count = 0;
unsigned char LCD_count = 0;
unsigned int __xdata heading,multipleInput, i, stops, ranger_distance,initial_heading, original_heading;
signed int error, prev_error, desired_heading;
int Kp_temp;
float Kp, Kd;
unsigned int counter_PCA = 0;
signed long __xdata PWLeftThrust, PWThrustAngle, PWRightThrust, motor_spd;
unsigned int __xdata PCA_START = 28614; //65535-36921
unsigned int __xdata PWCtrThrustAngle = 2779; // PulseWidth is about 1.5ms 2769
unsigned int __xdata PWCtrLeftThrust = 2759;
unsigned int __xdata PWCtrRightThrust = 2779; // needs higher pw
unsigned int PW_MIN = 2031;
unsigned int PW_MAX = 3508;
unsigned char addr_ranger = 0xE0; // address of ranger
unsigned char addr_compass = 0xC0; // address of compass
unsigned int __xdata RangerArray[2] = {0,0}; // implement a queue data structure

__sbit __at 0xB7 SS; // slide switch to enable/ disable servo and motor at P3.7

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void) {
    // initialize board
    Sys_Init();
    putchar(' ');
    Port_Init();
    XBR0_Init();
    PCA_Init();
    ADC_Init();
    SMB_Init();

    printf("Embedded Control Pulsewidth Calibration\r\n");
	// 2900 for gondola 5 gondola 8 3060 gondola 2 2540 gondola 9 2360 gondola 5 2900
	// Gondola ? 2780 // gondola 9 2580?
    PWThrustAngle = 2540; // start out @ vertical
    PWLeftThrust = PWCtrLeftThrust;
    PWRightThrust = PWCtrRightThrust;
    PCA0CP1 = 0xFFFF - PWThrustAngle; // thrust angle fan @ CEX1
    PCA0CP2 = 0xFFFF - PWLeftThrust; // left thrust fan @ CEX2
    PCA0CP3 = 0xFFFF - PWRightThrust; // right thrust fan @ CEX3
    counter_PCA = 0; //reset counter
    while(counter_PCA < 50);//wait for 1s for correct setup

    getDesiredHeading();
    getDerivativeGain();
    getProportionalGain();
    makeThrustVertical();
	// optimal gain: 100 for kd and 0.27 for kp
	original_heading = desired_heading;

    while(1) {
        maintainHeading();
    }
}

/*
 * implement a queue data structure: this array holds 2 of the latest range values
 */
void updateRangerArray(){
    RangerArray[0] = RangerArray[1];
    RangerArray[1] = ranger_distance;
}

/*
 * All 2 values in the array have to be above the limit before the function returns true
 * Prevents the problem where ranger values randomly spike to -1 from happening
 */
unsigned int rangerCompareLess(unsigned int limit){
    i = 0;
    for(i = 0; i < 2; i++){
        if (RangerArray[i] > limit) return 0;
    }
    return 1;
}

/*
 * All 2 values in the array have to be below the limit before the function returns true
 * Prevents the problem where ranger values randomly spike to -1 from happening
 */
unsigned int rangerCompareMore(unsigned int limit){
    i = 0;
    for(i = 0; i < 2; i++){
        if (RangerArray[i] < limit) return 0;
    }
    return 1;
}

/*
 * Using the keyboard, adjust the pw for the angle using + or -. Increment interval is 20 and entering a e
 * exits this function
 */
void makeThrustVertical(){
    printf("Adjust thrust angle using + and - to make sure they are vertical. Key in 'e' to exit \r\n");
    while (1){
        input = getchar();
        if (input == '+'){
            PWThrustAngle += 20;
        } else if (input == '-'){
            PWThrustAngle -= 20;
        } else if (input == 'e'){
            break;
        }
        PreventExtreme();
        PCA0CP1 = 0xFFFF - PWThrustAngle;
    }
}

/*
 * Read and update range values
 */
void update_ranger(){
    if (new_range){ //if 80ms has passed
        ranger_distance = ReadRanger(); // read the range
        //start a ping
        RangerData[0] = 0x51; // write 0x51 to reg 0 of the ranger:
        i2c_write_data(addr_ranger, 0, RangerData, 1); // write one byte of data to reg 0 at addr
		updateRangerArray(); // append range data to the queue

		if (rangerCompareLess(40)){ // if ranger reading is less than 48cm
			desired_heading -= 20;
			if (desired_heading < 0){
				desired_heading += 3600;
			}
    	}
    	if (rangerCompareMore(60)){ // if ranger reading is greater than 52cm
			desired_heading += 20; 
			if (desired_heading > 3600){
				desired_heading -= 3600;
			}
		}
		new_range = 0; //clear new range flag	
    }
}

/*
 * Enter the 3-bit derivative gain using keyboard
 */
void getDerivativeGain(){
    unsigned int __xdata inputArr[3] = {0,0,0};
    i = 0;
    printf("Enter Derivative Gain Bit by Bit Using Keyboard\r\n");
    while(i < 3){
        printf("enter the digit %d \r\n", i);
        input = getchar();
        input -= 48;
        printf("\r\n");
        inputArr[i] = input;
        i ++;
    }
    // convert to a 4 digit number
    Kd = inputArr[0]*100 + inputArr[1] * 10 + inputArr[2];
    printf_fast_f("The Derivatice Gain is %f\r\n",Kd);
}

/*
 * Entered the 4-bit proportional gain using keyboard. The last two bits entered are decimal(fractions)
 */
void getProportionalGain(){
    unsigned int __xdata inputArr[4] = {0,0,0,0};
    i = 0;
    printf("Enter Proportional Gain Bit by Bit Using Keyboard\r\n");
    printf("The Last Two Bits Are Fractions\r\n");
    while(i < 4){
        printf("enter the digit %d \r\n", i);
        input = getchar();
        input -= 48;
        printf("\r\n");
        inputArr[i] = input;
        i ++;
    }
    // convert to a float number in the form of wx.yz
    Kp = inputArr[0]*10 + inputArr[1] + 0.1 * inputArr[2] + 0.01 * inputArr[3];
    printf_fast_f("The Proportional Gain is %f\r\n",Kp);
}

/*
 * Enter the 4-bit derivative gain using keyboard
 */
void getDesiredHeading(){
    unsigned int __xdata inputArr[4] = {0,0,0,0};
    i = 0;
    printf("Enter Desired Heading Bit by Bit Using Keyboard\r\n");
    while(i < 4){
        printf("enter the digit %d \r\n", i);
        input = getchar();
        input -= 48;
        printf("\r\n");
        inputArr[i] = input;
        i ++;
    }
    // convert to a 4 digit number
    desired_heading = inputArr[0] * 1000 + inputArr[1]* 100 + inputArr[2]*10 + inputArr[3];
    printf("The desired heading is %d\r\n", desired_heading);
}

/*
 * Update the current heading every 80ms
 */
void update_compass(){
    if (new_heading){ // if every 80 ms
        heading = ReadCompass();
        new_heading = 0;
    }
}

/*
 * Maintain desired heading using 2 thrust fans
 */
void maintainHeading(){
    update_compass(); // get the newest ranger and compass readings
    update_ranger();

    error = desired_heading - heading;
    
	if (error > 1800){ // if your error is too high, reset it low. this keeps moves efficient
        error = error - 3600;
    } else if (error < -1800){ // if error is too high, reset low. this keeps moves efficient
        error = error + 3600;
    }

	/*
    if (rangerCompareLess(48)){ // if ranger reading is less than 48cm
		//printf("rang_comp less than48\r\n");
		//printf("increment flag is %d",increment_flag);
		//if (increment_flag){
			desired_heading -= 1;
			if (desired_heading < -1800){
				desired_heading = - 1800;
			}
			//increment_flag = 0;
		//}
    }
    if (rangerCompareMore(52)){
		//printf("rang_comp greater than52\r\n");
		//printf("increment flag is %d",increment_flag);
		if (increment_flag){
			desired_heading += 1;
			if (desired_heading > 1800){
				desired_heading = 1800;
			}
			//increment_flag = 0;
		//}
    }
	*/
	
    // using control algorithm 6 from worksheet 11
	PWRightThrust = (signed long) PWCtrRightThrust + Kp * (signed long)(error) + (signed long)Kd * (signed long) (error - prev_error);
	PWLeftThrust = (signed long) PWCtrLeftThrust -  Kp * (signed long)(error) - (signed long)Kd * (signed long) (error - prev_error);

    //makes sure the pulse widths are within the required limit
	PreventExtreme();

    //PCA0CP1 = 0xFFFF - PWThrustAngle; // thrust angle fan @ CEX1
    PCA0CP2 = 0xFFFF - PWLeftThrust; // left thrust fan @ CEX2
    PCA0CP3 = 0xFFFF - PWRightThrust; // right thrust fan @ CEX3

	if (print_flag){ // print the information every 200ms so SecureCRT won't become too cluttered
		printf("%d ", desired_heading);
		printf("%d ", heading);
		printf("%d ", error);
		printf("%ld ", PWLeftThrust);
		printf("%ld ", PWRightThrust);
		printf("%d \r\n", ranger_distance);
		print_flag = 0;
	}
    prev_error = error;
}

/*
 * Port init ->  Set up ports for input and output
 */
void Port_Init() {
    P1MDOUT |= 0x0F;  //set output pin for CEX0, CEX2 and CEX3 in push-pull mode
    P3MDOUT &= ~0x80; //Set Slideswitch at P3.7 for input
    P3 |= 0x80;

	//set up ADC conversion on Pin1.7
	P1MDIN &= ~0x80;
	P1MDOUT &= ~0x80;
	P1 |= 0x08;
}

/*
 * XBR0_Init -  Set up the crossbar
 */
void XBR0_Init() {
    XBR0 = 0x25;  //0x25 for lab 6
}

/*
 * PCA - Init -> Set up Programmable Counter Array
 */
void PCA_Init(void) {
    PCA0MD = 0x81;
    PCA0CPM2 = PCA0CPM1 = PCA0CPM3 = 0xC2;//enable CEX 1-3, disable CEX0
    EIE1 |= 0x08;
    PCA0CN = 0x40;
    EA = 1;
}

/*
 * PCA ISR -> Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
 */
void PCA_ISR ( void ) __interrupt 9 {
    counter_PCA++;
    if (CF){
        CF = 0;// reset flag
        heading_count++;
        LCD_count ++ ;
        ranger_count ++;
		increment_count ++;
        if(heading_count >= 2){ // approx 40 ms
            new_heading = 1; // flag new heading can be read
            heading_count = 0; // reset pca count timer
        }
        if (ranger_count >= 4){
            new_range = 1;//flag new range can be read
            ranger_count = 0;
        }
        if (LCD_count >= 10){// update the display every 200 ms
            print_flag = 1;
            LCD_count = 0;
        }
		if (increment_count >= 24){
			increment_flag = 1;
			increment_count = 0;
		}
        PCA0 = PCA_START;
    }
    PCA0CN &= 0xC0; // handle other pca interrupt resources
}

/*
 * Obtain the compass reading
 */
unsigned int ReadCompass(){
    i2c_read_data(addr_compass,2,CompassData,2);   //adress, byte to start, where to story, how many bytes to read
    heading = ((CompassData[0] << 8) | CompassData[1]); // turn 2 8-bit into one 16 bit
    return heading;
}

/*
 * Function that reads the distance from the ultrasonic ranger
 */
unsigned int ReadRanger() {
    unsigned int range = 0;
    i2c_read_data(addr_ranger, 2, RangerData, 2); // read two bytes, starting at reg 2
    range = (((unsigned int)RangerData[0] << 8) | RangerData[1]);
    return range;
}

/*
 * Ensure both pulse widths are within accepted range
 */
void PreventExtreme(){
    if (PWThrustAngle > PW_MAX){
        PWThrustAngle = PW_MAX;
    }
    if (PWThrustAngle < PW_MIN){
        PWThrustAngle = PW_MIN;
    }
    if (PWLeftThrust > PW_MAX){
        PWLeftThrust = PW_MAX;
    }
    if (PWLeftThrust < PW_MIN){
        PWLeftThrust = PW_MIN;
    }
    if (PWRightThrust < PW_MIN){
        PWRightThrust= PW_MIN;
    }
    if (PWRightThrust > PW_MAX){
        PWRightThrust = PW_MAX;
    }
}

void SMB_Init(void){
    SMB0CR = 0x93; // set scl to 100khz
    ENSMB = 1; // enable SMBus
}

void ADC_Init(void) {
    REF0CN = 0x03; /* Set Vref to use internal reference voltage (2.4V) */
    ADC1CN = 0x80; /* Enable A/D converter (ADC1) */
    ADC1CF |= 0x01; /* Set A/D converter gain to 1 */
}

unsigned char read_AD_input(unsigned char n) {
    AMX1SL = n; /* Set P1.n as the analog input for ADC1 */
    ADC1CN = ADC1CN & ~0x20; /* Clear the “Conversion Completed” flag */
    ADC1CN = ADC1CN | 0x10; /* Initiate A/D conversion */
    while ((ADC1CN & 0x20) == 0x00); /* Wait for conversion to complete */
    return ADC1; /* Return digital value in ADC1 register */
}
