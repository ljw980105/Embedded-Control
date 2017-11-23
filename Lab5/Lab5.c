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
unsigned int ReadRanger();
void PreventExtreme(void);
unsigned char read_AD_input(unsigned char n);
void ADC_Init(void);
void preselectHeading(void);

void read_accel(void);


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char __xdata input, pw_percentage; // using __xdata to store large variables
unsigned char CompassData[2], RangerData[2];
unsigned char new_heading = 0;
unsigned char new_range = 0;
unsigned char print_flag = 0;
unsigned char heading_count = 0;
unsigned char ranger_count = 0;
unsigned char LCD_count = 0;
unsigned int desired_heading, heading,multipleInput, i, stops, ranger_distance,initial_heading;
signed int error;
int Kp_temp;
float Kp;
unsigned int counter_PCA = 0;
unsigned int PW_Servo, PW_Motor, motor_spd;
unsigned int PCA_START = 28614; //65535-36921
unsigned int PW_CENTER = 2779; // PulseWidth is about 1.5ms 2769
unsigned int PW_CTR_SERVO = 2779;
unsigned int PW_MIN = 2031; // 1.1ms full reverse
unsigned int PW_MAX = 3508; // 1.9ms full forward

signed int avg_gx,avg_gy,gx,gy;
unsigned char addr_accel = 0x3A;
unsigned char AccelData[4];

__sbit __at 0xB7 SS; // slide switch to enable/ disable servo and motor at P3.7

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
    ADC_Init();
    SMB_Init();
    Accel_Init_C();

    printf("Embedded Control Pulsewidth Calibration\r\n");
    PW_Motor = PW_CENTER;  // set pw to 1.5ms
    PW_Servo = PW_CTR_SERVO;
    PCA0CP0 = 0xFFFF - PW_Motor;
    PCA0CP2 = 0xFFFF - PW_Servo;
    counter_PCA = 0; //reset counter
    while(counter_PCA < 50);//wait for 1s for correct setup

    // read and ask the user to adjust the gains
    Kp = read_AD_input(7) / 25 ;  // read adc input at pin 1.7

    while(1) {
        if (!SS) { // if SS is on
            // encounters the 1st obstacle: turn left/right based on input

            PreventExtreme();
        } else { //SS is not on: set everything to neutral
            PW_Servo = PW_CTR_SERVO; // Set Servo to neutral
            PCA0CPL0 = 0xFFFF - PW_Servo;
            PCA0CPH0 = (0xFFFF - PW_Servo) >> 8;
            PW_Motor = PW_CENTER; // set motor to neutral
            PCA0CPL2 = 0xFFFF - PW_Motor;
            PCA0CPH2 = (0xFFFF - PW_Motor) >> 8;
            PreventExtreme();
        }
        // print ranger distance and compass heading
        pw_percentage = (abs(PW_Motor- PW_CENTER)*100)/(PW_MAX- PW_MIN);
        if (print_flag){ // update LCD
			printf("%u %u %u\r\n",heading, ranger_distance, PW_Servo);
			lcd_clear();
            lcd_print("direction: %u\nrange: %u\npw perc: %u\nbattery: %u\n",heading,ranger_distance,pw_percentage,read_AD_input(6));
            print_flag = 0;
        }
    }
}

void read_accel(){
    avg_gy = avg_gx = gx = gy =0;

    for (i = 0; i < 8; i++){ //8 iterations to reduce noise

        i2c_read_data(addr_accel,0x27,AccelData,1);
        if (AccelData[0] & 0x03 == 0x03){ // accelerometer ready
            i2c_read_data(addr_accel,0x28|0x80,AccelData,4); //read accelerometer and store data
            avg_gx += ((AccelData[1] << 8) >> 4); //store x data into total x data
            avg_gy += ((AccelData[3] << 8) >> 4); //store y data into total y data
        }
    }
    avg_gy /= 8; //find average of y-direction acceleration
    avg_gx /= 8; //find average of x-direction acceleration
    gx = avg_gx;  // set gx and gy for later use
    gy = avg_gy;
}

/*
 * Adjust gain function: use existing gain or use the provided Update_Value() to adjust gain
 */
void adjust_gain(){
    Kp_temp = Kp;
    printf("Enter 1 to adjust servo gain, enter 2 to use existing gain");
    input = getchar();
	input -= 48;
	printf("\r\n");

    if (input == 1){// this blocked is skipped if input is 2
        printf("Enter 1 to adjust using keyboard, enter 2 to use keypad");
        printf( "'c' - default, 'i' - increment, 'd' - decrement, 'u' - update and return deflt = Constant;\r\n");
        input = getchar();
		input -= 48;
        if (input == 1) { // select an integer value between 10 - 102 (specified in lab description)
            Kp_temp = Update_Value(Kp_temp, 10, 102, 0, 1);
        } else if (input == 2){
            Kp_temp = Update_Value(Kp_temp, 10, 102, 0, 2);
        }
        Kp = Kp_temp / 10; // return a value within accepted range
    }
}


/*
 * sets proper pulsewidth for the motor
 */
void start_driving(){
    PCA0CP2 = 0xFFFF - PW_Motor;
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
        new_range = 0; //clear new range flag
		updateRangerArray(); // append range data to the queue
    }
}

/*
 * Select from a predefined list of heading or select manually
 */
void preselectHeading(){
    //create an array that represents the 4 bits of heading
    unsigned int __xdata inputArr[4] = {0,0,0,0}; // using __xdata to store large variables
    i = 0;
    printf("Enter 1 to select from the list of headings, or enter 2 to select manually\r\n");
    input = getchar();
	input -= 48;
	printf("\r\n");
    if (input == 1){ // select from list
        printf("Enter 1 to to select heading using the keyboard, or enter 2 to select from keypad\r\n" );
        input = getchar();
		input -= 48;
		printf("\r\n");
        if (input == 1){
            printf("Enter 1 for 0 deg, 2 for 90 deg, 3 for 180 deg, or 4 for 270 deg \r\n");
            input = getchar();
			input -= 48;
			printf("\r\n");
        } else if (input == 2){
            lcd_print("Enter 1 for 0 deg, 2 for 90 deg, 3 for 180 deg, or 4 for 270 deg \r\n");
            input = read_keypad();
            input -= 48;
			printf("\r\n");
        }
        switch (input){ // select input based on input values
            case 1:
				desired_heading = 0; //0 deg
                break;
            case 2:
                desired_heading = 900; // 90 deg
                break;
            case 3:
                desired_heading = 1800; // 180 deg
                break;
            case 4:
                desired_heading = 2700; // 270 deg
                break;
            default:
                break;
        }
		initial_heading = desired_heading;
    } else if (input == 2){ // select manually
        printf("Enter 1 to enter the heading using the Keypad, or enter 2 to enter using keyboard\r\n");
        input = getchar();
		input -= 48;
	    printf("\r\n");
        switch (input){
            case 1:
                multipleInput = kpd_input(0);
                desired_heading = multipleInput;
                break;
            case 2:
                // enter the 4 digits of heading digit by digit
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
                break;
            default:
                break;
        }
		initial_heading = desired_heading;
    }
}

/*
 * Select from a predefined list of motor speed or select manually
 */
void preselectMotorSpd(){
    unsigned int __xdata inputArr[4] = {0,0,0,0}; // using __xdata to store large variables
    i = 0;
    printf("Enter 1 to select from the list of motor speeds, or enter 2 to select manually\r\n");
    input = getchar();
	input -= 48;
	printf("\r\n");
    if (input == 1){ // select from list
        printf("Enter 1 to to select speed using the keyboard, or enter 2 to select from keypad\r\n");
        input = getchar();
		input -= 48;
		printf("\r\n");
        if (input == 1){
            printf("Enter 1 for full reverse , 2 for 2/3 full reverse ,3 for 78% full forward, or 4 for full forward \r\n");
            input = getchar();
			input -= 48;
			printf("\r\n");
        } else if (input == 2){
            lcd_print("Enter 1 for full reverse , 2 for 2/3 full reverse , 3 for 78% full forward, or 4 for full forward \r\n");
            input = read_keypad(); // obtain input using keypad
            input -= 48;
			printf("\r\n");
        }
        switch (input){
            case 1:
                PW_Motor = 2031; // full reverse
                break;
            case 2:
                PW_Motor = 2300; // 2/3 full reverse
                break;
            case 3:
                PW_Motor = 3339; // 78% full forward
                break;
            case 4:
                PW_Motor = 3508; // full forward
                break;
            default:
                break;
        }
        motor_spd = PW_Motor;
    } else if (input == 2){ // select manually
        printf("Enter 1 to enter the heading using the Keypad, or enter 2 to enter using keyboard\r\n");
        input = getchar();
		input -= 48;
		printf("\r\n");
        switch (input){
            case 1:
                multipleInput = kpd_input(0);
                PW_Motor = multipleInput;// obtain input using keypad
                break;
            case 2:
                // enter the 4 digits of heading digit by digit
                while(i < 4){
                    printf("enter the digit %d \r\n", i);
                    input = getchar();
					input -= 48;
					printf("\r\n");
                    inputArr[i] = input;
                    i ++;
                }
                //convert array to a 4 digit number
                PW_Motor = inputArr[0] * 1000 + inputArr[1]* 100 + inputArr[2]*10 + inputArr[3];
                break;
            default:
                break;
        }
        motor_spd = PW_Motor;
    }
}

/*
 * execute a hard right
 */
void turn_right(){
    if (new_heading){
        heading = ReadCompass(); // read the current heading
        new_heading = 0;
    }
    // set the desired heading to 90 deg higher than the current heading (right turn)
    desired_heading = heading + 900;
    motor_spd -= 200; // decrease motor speed for accurate turning
    while (getchar_nw() != ' '){ // keep turning until space bar is pressed
        update_ranger();
        adjustServo();
        PW_Motor = motor_spd;
        start_driving();
        pw_percentage = (abs(PW_Motor- PW_CENTER)*100)/(PW_MAX- PW_MIN);
		if (print_flag){ // update LCD
			printf("%u %u %u\r\n",heading, ranger_distance, PW_Servo);
			lcd_clear();
	        lcd_print("direction: %u\nrange: %u\npw perc: %u\nbattery: %u\n",heading,ranger_distance,pw_percentage,read_AD_input(6));
	        print_flag = 0;
    	}
    }
    motor_spd += 200; // set motor speed back to normal
    desired_heading = initial_heading;
}

/*
 * execute a hard left
 */
void turn_left(){
	if (new_heading){
        heading = ReadCompass(); // read the current heading
        new_heading = 0;
    }
    // set the desired heading to 80 deg less than the current heading (leftturn)
    desired_heading = heading - 800;
	motor_spd -= 200; // decrease motor speed for accurate turning
    while (getchar_nw() != ' '){ // keep turning until space bar is pressed
        update_ranger();
		adjustServo();
		PW_Motor = motor_spd;
		start_driving();
        pw_percentage = (abs(PW_Motor- PW_CENTER)*100)/(PW_MAX- PW_MIN);
		if (print_flag){ // update LCD
			printf("%u %u %u\r\n",heading, ranger_distance, PW_Servo);
			lcd_clear();
	        lcd_print("direction: %u\nrange: %u\npw perc: %u\nbattery: %u\n",heading,ranger_distance,pw_percentage,read_AD_input(6));
	        print_flag = 0;
    	}
    }
	motor_spd += 200; // set motor speed back to normal
	desired_heading = initial_heading;
}

/*
 * adjust the servo direction by adjusting the current heading to match the desired heading
 */
void adjustServo(){
    error = desired_heading - heading; // set error
    if (new_heading) { // 40 ms passed
        heading = ReadCompass(); // set heading to heading reported by electronic compass
        adjust_pw(); // run adj pw function
        new_heading = 0;
    }
}

/*
 * Provided code from lab description. This code adjusts Kp without having to recompile code.
 * Mode1 - keyboard input
 * Mode2 - Keypad input
 */
unsigned int Update_Value(int Constant, unsigned char incr, int maxval, int minval, int mode) {
    int deflt = Kp;
    char input = 0;
    while(1) {
        if (mode == 1) input = getchar();
        if (mode == 2) input = read_keypad();
        input -= 48;// convert from ascii to decimal
        if (input == 'c') Constant = deflt;
        if (input == 'i'){
            Constant += incr;
            if (Constant > maxval) Constant = maxval;
            printf("The current value of constant is %d\r\n", Constant);
        }
        if (input == 'd') {
            Constant -= incr;
            if (Constant < minval) Constant = minval;
            printf("The current value of constant is %d\r\n", Constant);
        }
        if (input == 'u') return Constant;
    }
}

/*
 * Port init ->  Set up ports for input and output
 */
void Port_Init() {
    P1MDOUT |= 0x0D;  //set output pin for CEX0, CEX2 and CEX3 in push-pull mode
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
    XBR0 = 0x27;  //configure crossbar as directed in the laboratory
}

/*
 * PCA - Init -> Set up Programmable Counter Array
 */
void PCA_Init(void) {
    PCA0MD = 0x81;
    PCA0CPM2 = PCA0CPM0 = 0xC2;
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
        if(heading_count >= 2){ // approx 40 ms
            new_heading = 1; // flag new heading can be read
            heading_count = 0; // reset pca count timer
        }
        if (ranger_count >= 4){
            new_range = 1;//flag new range can be read
            ranger_count = 0;
        }
        if (LCD_count >= 20){// update the display every 400 ms
            print_flag = 1;
            LCD_count = 0;
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
 * Utilize the proportional gain method to adjust heading
 */
void adjust_pw() {

    if (error > 1800){ // if your error is too high, reset it low. this keeps moves efficient
        error = error - 3600;
    } else if (error < -1800){ // if error is too high, reset low. this keeps moves efficient
        error = error + 3600;
    }
    PW_Servo = Kp*(error) + PW_CTR_SERVO + 100; // set new PW
    PCA0CPL0 = 0xFFFF - PW_Servo;
    PCA0CPH0 = (0xFFFF - PW_Servo) >> 8;
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
    if (PW_Motor > PW_MAX){
        PW_Motor = PW_MAX;
    }
    if (PW_Motor < PW_MIN){
        PW_Motor = PW_MIN;
    }
    if (PW_Servo > PW_MAX){
        PW_Servo = PW_MAX;
    }
    if (PW_Servo < PW_MIN){
        PW_Servo = PW_MIN;
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
