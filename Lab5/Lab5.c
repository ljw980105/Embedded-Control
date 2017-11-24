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
void PreventExtreme(void);
unsigned char read_AD_input(unsigned char n);
void ADC_Init(void);
void preselectMotorSpd(void);

void read_accel(void);
void set_gains(void);
void set_steering_pw(void);
void set_driving_pw(void);
void start_steering();

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char __xdata input, pw_percentage; // using __xdata to store large variables
unsigned char new_heading = 0;
unsigned char new_range = 0;
unsigned char print_flag = 0;
unsigned char accels_count = 0;
unsigned char accels_flag = 0;
unsigned char toggle_flag = 0;
unsigned int reverse_count = 0;
unsigned char LCD_count = 0;
unsigned int multipleInput, i, stops;
signed int error;
int Kp_temp;
float Kp;
unsigned int counter_PCA = 0;
unsigned int PW_Servo, PW_Motor, motor_spd;
unsigned int PCA_START = 28614; //65535-36921
unsigned int PW_CENTER_MOTOR = 2779; // PulseWidth is about 1.5ms 2769
unsigned int PW_CTR_SERVO = 2779;
unsigned int PW_MIN = 2031; // 1.1ms full reverse
unsigned int PW_MAX = 3508; // 1.9ms full forward

signed int avg_gx,avg_gy,gx,gy;
unsigned char addr_accel = 0x3A;
unsigned char AccelData[4];
//respectively: steering gain, drive gain for x-dir & y-dir, intergal gain
unsigned char ks,kdx,kdy,ki;
unsigned char isReversed = 0;
unsigned char battery_voltage;

__sbit __at 0xB7 SS; // slide switch to enable/ disable servo and motor at P3.7
__sbit __at 0xXX BUZZER; //TODO: Verify Buzzer Port

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void){
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
    PW_Motor = PW_CENTER_MOTOR;  // set pw to 1.5ms
    PW_Servo = PW_CTR_SERVO;
    PCA0CP0 = 0xFFFF - PW_Servo;
    PCA0CP2 = 0xFFFF - PW_Motor;
    counter_PCA = 0; //reset counter
    while(counter_PCA < 50);//wait for 1s for correct setup

    // read and ask the user to adjust the gains
    Kp = read_AD_input(7) / 25 ;  // read adc input at pin 1.7
    stops = 0;
    preselectMotorSpd();

    while(1) {
        if (!SS) { // if SS is on
            if (stops == 0){
                set_gains();
                stops ++;
            }
            if (accels_flag){
                read_accel();
                set_steering_pw();
                set_driving_pw();
                start_driving();
                start_steering();
                accels_flag = 0;
            }

            if (isReversed){
                if(toggle_flag){
                    BUZZER = 1;
                    if(reverse_count % 25 == 0) toggle_flag = 0;
                }
                if (!toggle_flag){
                    BUZZER = 0;
                    if(reverse_count % 50 == 0) toggle_flag = 1;
                }
            }

        } else { //SS is not on: set everything to neutral
            PW_Servo = PW_CTR_SERVO; // Set Servo to neutral
            start_steering();
            PW_Motor = PW_CENTER_MOTOR; // set motor to neutral
            start_driving();
        }


        // print ranger distance and compass heading
        if (print_flag){ // update LCD
            battery_voltage = read_AD_input(6);
			printf("%d %d %d %d %d %d %d %d %d\r\n",gx,gy,kdx,kdy,ks,ki,PW_Motor,PW_Servo,battery_voltage);
			lcd_clear();
            lcd_print("ks: %u, kdx: %u, kdy: %u\nDrive: %u, Servo: %u\n, Battery: %u\n",ks,kdx,kdy,PW_Motor, PW_Servo,battery_voltage);
            print_flag = 0;
        }
    }
}

/*
* Read 8 values from the accelerometer to average out the noise
*/
void read_accel(){
    avg_gy = avg_gx = gx = gy =0;

    for (i = 0; i < 8; i++){ //8 iterations

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
* Enter the ks,kdx,kdy,ki values using the keypad
*/
void set_gains(){
    lcd_clear();
	lcd_print("Enter value for ks\r\n"); //steering feedback gain
	ks = kpd_input(0);
	lcd_clear();
	lcd_print("Enter value for kdx\r\n");// x direction drive gain
	kdx = kpd_input(0);
	lcd_clear();
	lcd_print("Enter value for kdy\r\n");// y direction drive gain
	kdy = kpd_input(0);
	lcd_clear();
	lcd_print("Enter value for ki\n "); // intergal gain
	ki  = kpd_input(0);
	lcd_clear();
}

void set_steering_pw(){
    PW_Servo = PW_CTR_SERVO - ks * gx;
}

void set_driving_pw(){
    PW_Motor = PW_CENTER_MOTOR + kdy * gy;
    PW_Motor += kds * abs(gx);

    /* Optional - use integral gain
    PW_Motor += kdx * abs(gx) + ki * error_sum //ki is the integral gain error_sum += gy + abs(gx)
    error_sum += gy + abs(gx)
    */
}

/*
 * sets proper pulsewidth for the motor
 */
void start_driving(){
    PreventExtreme();
    PCA0CP2 = 0xFFFF - PW_Motor;
}

/*
 * sets proper pulsewidth for the Servo
 */
void start_steering(){
    PreventExtreme();
    PCA0CP0 = 0xFFFF - PW_Servo;
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
                isReversed = 1;
                break;
            case 2:
                PW_Motor = 2300; // 2/3 full reverse
                isReversed = 1;
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
        LCD_count ++ ;
        accels_count ++;
        reverse_count ++;

        if (LCD_count >= 20){// update the display every 400 ms
            print_flag = 1;
            LCD_count = 0;
        }
        if (accels_count >= 1){
            accels_flag = 1;
            accels_count = 0;
        }
        if (reverse_count >= 51){
            reverse_count = 0;
        }
        PCA0 = PCA_START;
    }
    PCA0CN &= 0xC0; // handle other pca interrupt resources
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
