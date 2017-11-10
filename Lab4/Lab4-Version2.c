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
void adjust_pw(void);
unsigned int ReadRanger();
void PreventExtreme(void);
unsigned char read_AD_input(unsigned char n);
void ADC_Init(void);
void preselectHeading(void);
void preselectMotorSpd(void);
void turn_left(void);
void turn_right(void);
void adjustServo(void);
unsigned int Update_Value(int Constant, unsigned char incr, int maxval, int minval, int mode);
void start_driving(void);
void adjust_gain(void);
void update_ranger(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char input;
unsigned char CompassData[2], RangerData[2];
unsigned char new_heading = 0;
unsigned char new_range = 0;
unsigned char print_flag = 0;
unsigned char heading_count = 0;
unsigned char ranger_count = 0;
unsigned char LCD_count = 0;
//unsigned int print_count = 0;
// using __xdata to store large variables
unsigned int __xdata desired_heading, toadj, heading,multipleInput, i, stops, ranger_distance;
signed int error;
int Kp_temp;
float Kp;
unsigned int counter_PCA = 0;
unsigned int PW_Servo, PW_Motor;
unsigned int PCA_START = 28614; //65535-36921
unsigned int PW_CENTER = 2769; // PulseWidth is about 1.5ms 2769
unsigned int PW_MIN = 2031; // 1.1ms full reverse
unsigned int PW_MAX = 3508; // 1.9ms full forward
unsigned char addr_ranger = 0xE0; // address of ranger
unsigned char addr_compass = 0xC0; // address of compass
unsigned char motor_spd;

__sbit __at 0xB7 SS; // slideswitch to enable/ disable servo and motor at P3.7

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

    printf("Embedded Control Pulsewidth Calibration\r\n");
    PW_Motor = PW_CENTER;  // set pw to 1.5ms
    counter_PCA = 0; //reset counter
    while(counter_PCA < 50);//wait for 1s

    // read the gains
    Kp = read_AD_input(4) / 25 ;  // TODO: verify the ports, (pin 1.4 - 1.7)
    printf_fast_f("The ADC Conversion Result is %f\r\n", Kp);
    stops = 0;
    adjust_gain();
    //read the heading and motor speed from secureCRT or keypad
    preselectHeading();
    preselectMotorSpd();
    ranger_distance = 10000; // initialize at a large number for proper functionality


    while(1) {
        if (!SS) { // if SS is on
            if (stops == 0 && ranger_distance < 40){
                // stop the car
                PW_Motor = PW_CENTER;
                start_driving(); // sets the proper pulsewidth for the motor
                //turn according to keyboard input
                input = getchar_nw(); // returns 0xFF is no key is pressed
                if (input != 0xFF){ // pressing a valid key
                    if (input == 'L' || input == 'l'){
                        turn_left();
                    } else if  (input == 'R' || input == 'r'){
                        turn_right();
                    } else if (input == ' '){ // resume driving
                        PW_Motor = motor_spd;
                        start_driving(); // sets the proper pulsewidth for the motor
                    }
                }
                stops ++;
            } else if (stops == 1 && ranger_distance < 40) {
                // stop the car
                PW_Motor = PW_CENTER;
                start_driving();
                break;
            } else {
                adjustServo();
                start_driving();
                update_ranger();
            }
            PreventExtreme();
        } else { //SS is not on
            PW_Servo = PW_CENTER; // Set Servo to neutral
            PCA0CPL0 = 0xFFFF - PW_Servo;
            PCA0CPH0 = (0xFFFF - PW_Servo) >> 8;
            PW_Motor = PW_CENTER; // set motor to neutral
            PCA0CPL2 = 0xFFFF - PW_Motor;
            PCA0CPH2 = (0xFFFF - PW_Motor) >> 8;
            PreventExtreme();
        }
        // print ranger distance and compass heading
        printf("The current compass heading is %d\r\n", heading);
        printf("The current ranger distance is %d cm \r\n", ranger_distance);
        if (print_flag){ // update LCD
            lcd_print("The current compass heading is %d\r\n", heading);
            lcd_print("The current ranger distance is %d cm \r\n", ranger_distance);
            print_flag = 0;
        }
    }
}


void adjust_gain(){
    Kp_temp = Kp;
    printf("Enter 1 to adjust servo gain, enter 2 to use existing gain");
    input = getchar();

    if (input == 1){
        printf("Enter 1 to adjust using keyboard, enter 2 to use keypad");
        printf( "'c' - default, 'i' - increment, 'd' - decrement, 'u' - update and return deflt = Constant;\r\n");
        input = getchar();
        if (input == 1) {
            Kp_temp = Update_Value(Kp_temp, 10, 102, 0, 1);
        } else if (input == 2){
            Kp_temp = Update_Value(Kp_temp, 10, 102, 0, 0);
        }
        Kp = Kp_temp / 10;
    }
}


void start_driving(){
    PCA0CPL2 = 0xFFFF - PW_Motor;
    PCA0CPH2 = (0xFFFF - PW_Motor) >> 8;
}

void update_ranger(){
    if (new_range){ //if 80ms has passed
        ranger_distance = ReadRanger(); // read the range
        //start a ping
        RangerData[0] = 0x51; // write 0x51 to reg 0 of the ranger:
        i2c_write_data(addr_ranger, 0, RangerData, 1); // write one byte of data to reg 0 at addr
        new_range = 0; //clear new range flag
        printf("The current range is %d cm \r\n",ranger_distance);
    }
}

/*
 * Select from a predefined list of heading or select manually
 */
void preselectHeading(){
    unsigned int __xdata inputArr[4] = {0,0,0,0}; // using __xdata to store large variables
    i = 0;
    printf("Enter 1 to select from the list of headings, or enter 2 to select manually\r\n");
    input = getchar();
    if (input == 1){
        printf("Enter 1 to to select heading using the keyboard, or enter 2 to select from keypad\r\n" );
        input = getchar();
        if (input == 1){
            printf("Enter 1 for 0 deg, 2 for 90 deg, 3 for 180 deg, or 4 for 270 deg \r\n");
            input = getchar();
        } else if (input == 2){
            lcd_print("Enter 1 for 0 deg, 2 for 90 deg, 3 for 180 deg, or 4 for 270 deg \r\n");
            input = read_keypad();
            input -= 48;
        }
        switch (input){
            case 1:
                break;
            case 2:
                desired_heading = 900;
                break;
            case 3:
                desired_heading = 1800;
                break;
            case 4:
                desired_heading = 2700;
                break;
            default:
                break;
        }
    } else if (input == 2){
        printf("Enter 1 to enter the heading using the Keypad, or enter 2 to enter using keyboard\r\n");
        input = getchar();
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
                    input -= 48; // convert from ascii to numerical value
                    inputArr[i] = input;
                    i ++;
                }
                desired_heading = inputArr[0] * 1000 + inputArr[1]* 100 + inputArr[2]*10 + inputArr[3];
                break;
            default:
                break;
        }
    }
}

void preselectMotorSpd(){
    unsigned int __xdata inputArr[4] = {0,0,0,0}; // using __xdata to store large variables
    i = 0;
    printf("Enter 1 to select from the list of motor speeds, or enter 2 to select manually\r\n");
    input = getchar();
    if (input == 1){
        printf("Enter 1 to to select speed using the keyboard, or enter 2 to select from keypad\r\n");
        input = getchar();
        if (input == 1){
            printf("Enter 1 for full reverse , 2 for 1/2 full reverse ,3 for 1/2 full forward, or 4 for full forward \r\n");
            input = getchar();
        } else if (input == 2){
            lcd_print("Enter 1 for full reverse , 2 for 1/2 full reverse , 3 for 1/2 full forward, or 4 for full forward \r\n");
            input = read_keypad();
            input -= 48;
        }
        switch (input){
            case 1:
                PW_Motor = 2031;
                break;
            case 2:
                PW_Motor = 2400;
                break;
            case 3:
                PW_Motor = 3139;
                break;
            case 4:
                PW_Motor = 3508;
                break;
            default:
                break;
        }
        motor_spd = PW_Motor;
    } else if (input == 2){
        printf("Enter 1 to enter the heading using the Keypad, or enter 2 to enter using keyboard\r\n");
        input = getchar();
        switch (input){
            case 1:
                multipleInput = kpd_input(0);
                PW_Motor = multipleInput;
                break;
            case 2:
                // enter the 4 digits of heading digit by digit
                while(i < 4){
                    printf("enter the digit %d \r\n", i);
                    input = getchar();
                    input -= 48; // convert from ascii to numerical value
                    inputArr[i] = input;
                    i ++;
                }
                PW_Motor = inputArr[0] * 1000 + inputArr[1]* 100 + inputArr[2]*10 + inputArr[3];
                break;
            default:
                break;
        }
        motor_spd = PW_Motor;
    }
}



//execute a hard left
void turn_left(){
    if (new_heading){
        heading = ReadCompass(); // read the current heading
        new_heading = 0;
    }
    desired_heading = heading - 900;
    while(heading > desired_heading){
//      error = desired_heading - heading; // set error
        adjustServo(); // turn the wheels left
        PW_Motor = motor_spd;
        start_driving();
    }
}

//execute a hard right
void turn_right(){
    if (new_heading){
        heading = ReadCompass(); // read the current heading
        new_heading = 0;
    }
    desired_heading = heading + 900;
    while(heading < desired_heading){
//      error = desired_heading - heading; // set error
        adjustServo(); // turn the wheels left
        PW_Motor = motor_spd;
        start_driving();
    }
}

//adjust the servo direction by adjusting the current heading to match the desired heading
void adjsutServo(){
    error = desired_heading - heading; // set error
    if (new_heading) { // 40 ms passed
        heading = ReadCompass(); // set heading to heading reported by electronic compass
		/*
        if (print_count > 5) { // only print out every 5th reading
            printf("\r\n heading is %d",heading);
            printf("\r\n desired heading is %d",desired_heading);
            print_count = 0; // reset print counter
            printf("\r\n Error is %d",error);
            printf(" \r\n current PW: %u\n\r", PW_Servo);
            toadj = Kp*(error) + PW_CENTER;
            printf("\r\n the pulse width is now being adjusted to %d",toadj);
        }
        print_count++; // increment print count
		*/
        adjust_pw(); // run adj pw function
        new_heading = 0;
    }
}


unsigned int Update_Value(int Constant, unsigned char incr, int maxval, int minval, int mode) {
    int deflt = Kp;
    char input = 0;
    while(1) {
        if (mode == 1) input = getchar();
        if (mode == 2){
            input = read_keypad();
            input -= 48;
        }
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
        if (LCD_count >= 25){
            print_flag = 1;
            LCD_count = 0;
        }
            PCA0 = PCA_START;
    }
    PCA0CN &= 0xC0;
    }

unsigned int ReadCompass(){

    i2c_read_data(addr_compass,2,CompassData,2);   //adress, byte to start, where to story, how many bytes to read
    heading = ((CompassData[0] << 8) | CompassData[1]); // turn 2 8-bit into one 16 bit
    return heading;
}

void adjust_pw() {

    if (error > 1800) // if your error is too high, reset it low. this keeps moves efficient
    {
        error = error - 3600;
    }
    else if (error < -1800) // if error is too high, reset low. this keeps moves efficient
    {
        error = error + 3600;
    }

    PW_Servo = Kp*(error) + PW_CENTER; // set new PW
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
