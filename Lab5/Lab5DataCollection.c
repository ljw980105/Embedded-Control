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
void preselectRoll(void);
void read_accel(void);
void set_gains(void);
void set_steering_pw(void);
void set_driving_pw(void);
void start_steering(void);
void start_driving(void);
unsigned int Update_Value(int Constant, unsigned char incr, int maxval, int minval, int mode);
void adjust_gain(void);
void updateFlatArr(void);
unsigned int isFlat(void);

void updateBuzzerArr(void);
unsigned int isReversed(void);
void calibrateAccel(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
unsigned char input,temp;
unsigned char print_flag = 0;
unsigned char accels_count = 0;
unsigned char accels_flag = 0;
unsigned char toggle_flag = 0;
unsigned int reverse_count = 0;
unsigned char LCD_count = 0;
unsigned int multipleInput, i, stops, error_sum, flat_counter;
unsigned int counter_PCA = 0;
unsigned long __xdata PW_Servo, PW_Motor;
unsigned int PCA_START = 28614; //65535-36921
unsigned int PW_CENTER_MOTOR = 2779; // PulseWidth is about 1.5ms 2769
unsigned int PW_CTR_SERVO = 2969;
unsigned int PW_MIN = 2031; // 1.1ms full reverse
unsigned int PW_MAX = 3508; // 1.9ms full forward

signed int __xdata avg_gx,avg_gy,gx,gy, gx_offset, gy_offset;
unsigned char __xdata AccelData[4];
unsigned int __xdata isFlatArr[5]= {0,0,0,0,0}; // determine when the car stops at the top of ramp -> also a queue DS
unsigned int __xdata isReversedArr[3]= {3000,3000,3000}; // determine wh, en the car stops at the top of ramp -> also a queue DS
//respectively: steering gain, drive gain for x-dir & y-dir, integral gain
unsigned char __xdata ks,kdx,kdy,ki;
unsigned char Kfront_back_pitch, Kside_to_side_roll; // pre-selected gain values
unsigned char battery_voltage;
signed int max_slope = 0;

__sbit __at 0xB7 SS; // slide switch to enable/ disable servo and motor at P3.7
__sbit __at 0xB6 BUZZER; //Buzzer at port 3.6

//TODO: Question: is kdx or kdy front-back pitch? (Almost certain it's kdy)

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

    stops = 0;
	lcd_clear();
	BUZZER = 0;
	error_sum = 0;
	printf("Before calibration\r\n");
	calibrateAccel();
	printf("After calibration\r\n");

    while(1) {
        if (!SS) { // if SS is on
            if (stops == 0){
                // read and ask the user to adjust the gains for the front-back pitch value
                Kfront_back_pitch = read_AD_input(7) / 5.1 ;  // read adc input at pin 1.7
                printf("The ADC Conversion Result is %d \r\n", Kfront_back_pitch);
                adjust_gain(); // adjust front-back-pitch using Update_Value() ( inside adjust_gain() )
                preselectRoll(); // enter side-to-side-roll using keypad
                set_gains(); //enter ks using keypad. Optional: use integral gain
                stops ++;

                kdx = Kside_to_side_roll;
                kdy = Kfront_back_pitch;
            }
            if (accels_flag){
                read_accel();
                set_steering_pw();
                set_driving_pw();
				printf("%d %d %ld %ld \r\n", gx,gy, PW_Motor, PW_Servo);
				if (PW_Motor < 2700) PW_Motor -= 300;// increase horsepower for reverse driving
                start_driving();
                start_steering();
                accels_flag = 0;
            }
			updateBuzzerArr();

            // if the car is driving reversely, execute the following code in cycle:
            // sound buzzer for .5 s and turn it off for 1s
            if (isReversed()){			
                if(toggle_flag){
                    BUZZER = 1; // sound buzzer
                    if(reverse_count > 25 ){ // toggles after .5s
						toggle_flag = 0;
						reverse_count = 0;
					}
                }
                if (!toggle_flag){
                    BUZZER = 0; // turn off buzzer
                    if(reverse_count > 50) {
						toggle_flag = 1; // toggles after 1s
						reverse_count = 0;
					}
                }				
            } else {
                BUZZER = 0; // turn off buzzer
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
//			printf("gx: %d gy: %d kdx: %d kdy: %d ks: %d \r\n"
//                           "PW_Motor: %ld PW_Servo: %ld Battery: %d \r\n",gx,gy,kdx,kdy,ks,PW_Motor,PW_Servo,battery_voltage);
			lcd_clear();
            lcd_print("ks: %u, kdx: %u, kdy: %u\nPWDrive: %ld, PWServo: %ld\n, Battery: %u\n",ks,kdx,kdy,PW_Motor, PW_Servo,battery_voltage);
            print_flag = 0;
        }
		

		if (isFlat()){
			// printf("The maximum slope is %d \r\n", max_slope);
			 PW_Motor = PW_CENTER_MOTOR;
			 PW_Servo = PW_CTR_SERVO;
			 start_driving();
             start_steering();
		}
    }
}

/*
* Read 8 values from the accelerometer to average out the noise
*/
void read_accel(){
	//unsigned char addr_accel = 0x3A;
    avg_gy = avg_gx = gx = gy =0; // reset

    for (i = 0; i < 10; i++){ //8 iterations
		while(!accels_flag);
        i2c_read_data(0x3A,0x27,AccelData,1);
        if (AccelData[0] & 0x03 == 0x03){ // accelerometer ready
            i2c_read_data(0x3A,0x28|0x80,AccelData,4); //read accelerometer and store data
            avg_gx += ((AccelData[1] << 8) >> 4) - gx_offset; //store x data into total x data
            avg_gy += ((AccelData[3] << 8) >> 4) - gy_offset;;  //store y data into total y data
        }
		accels_flag = 0;
    }
    avg_gy /= 10; //find average of y-direction acceleration
    avg_gx /= 10; //find average of x-direction acceleration
    gx = avg_gx;  // set gx and gy for later use
    gy = avg_gy;
	updateFlatArr();
	if (gy < 100) gy  *= 0.5;


    if (abs(gx) > max_slope) max_slope = abs(gx); // obtain max slope
}

void calibrateAccel(){
	avg_gy = avg_gx = gx = gy =0; // reset
    for (i = 0; i < 64; i++){ //8 iterations
		while(!accels_flag);
        i2c_read_data(0x3A,0x27,AccelData,1);
        if (AccelData[0] & 0x03 == 0x03){ // accelerometer ready
            i2c_read_data(0x3A,0x28|0x80,AccelData,4); //read accelerometer and store data
            avg_gx += ((AccelData[1] << 8) >> 4); //store x data into total x data
            avg_gy += ((AccelData[3] << 8) >> 4); //store y data into total y data
        }
		accels_flag = 0;
    }
    avg_gy /= 64; //find average of y-direction acceleration
    avg_gx /= 64; //find average of x-direction acceleration
    gx_offset = avg_gx;  // set gx and gy for later use
    gy_offset = avg_gy;
}

/*
 * Adjust gain function: use existing gain or use the provided Update_Value() to adjust gain
 */
void adjust_gain(){
    temp = Kfront_back_pitch;
    lcd_print("1 ADJUST, 2 SKIP\r\n");
    input = kpd_input(0);
	printf("\r\n");

    if (input == 1){// this code block is skipped if input is 2
        lcd_print("Adjust the front-back pitch using keypad\r\n");
        lcd_print( "'c' - default, 'i' - increment, 'd' - decrement, 'u' - update and return deflt = Constant;\r\n");
        temp = Update_Value(temp, 20, 500, 0, 2);
        Kfront_back_pitch = temp / 10; // return a value within accepted range(0 - 50)
    }
}

/*
 * Enter the ks,ki values using the keypad
 */
void set_gains(){
    lcd_clear();
	lcd_print("Enter value for ks\r\n"); //steering feedback gain
	ks = kpd_input(0);
//	lcd_clear();
//	lcd_print("Enter value for ki\n "); // intergal gain
//	ki  = kpd_input(0);
}

/*
 * Set the pulse width for steering using steering center pulse width, ks and gx
 */
void set_steering_pw(){
    PW_Servo = PW_CTR_SERVO - ks * gx;
}

/*
 * Set the pulse width for driving using driving Center Pulse Width, kdy, gy,kds and gx
 */
void set_driving_pw(){
    PW_Motor = PW_CENTER_MOTOR + kdy * gy;
    if (gy > 0){
		PW_Motor += kdx * abs(gx);
	} else if (gy < 0){
		PW_Motor -= kdx * abs(gx);
	}

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
 * Queue data structure to store the latest 3 values of pitch
 */
void updateFlatArr(){
	for (flat_counter = 0; flat_counter < 4; flat_counter ++){
		isFlatArr[flat_counter] = isFlatArr[flat_counter + 1];
	}
    isFlatArr[4] = gy; // TODO: May be gy
}

/*
 * All 3 values in the array have to be within the limit before the function returns true
 * Prevents the problem ...
 */
unsigned int isFlat(void){
	flat_counter = 0;
    i = 0;
    for ( i = 0; i < 5; i++) {
      	if (abs(isFlatArr[i]) < 16 ){
	  		flat_counter ++;	  	
	  	}
    }
    if (flat_counter >= 2){
		 return 1;
	}
	return 0;
}

void updateBuzzerArr(){
	isReversedArr[0] = isReversedArr[1];
	isReversedArr[1] = isReversedArr[2];
	isReversedArr[2] = PW_Motor;

}
unsigned int isReversed(){
	i = 0;
	for ( i = 0; i < 3; i++) {
      if (isReversedArr[i] > 2700) return 0; // change the limit!
    }
    return 1;

}

/*
 * Select the side to side roll value using keypad
 */
void preselectRoll(){
    lcd_print("Roll: Press 1 to select roll from a list, or press 2 to enter roll manually \r\n");
    input = kpd_input(0);
    lcd_print("\r\n");
    if (input == 1){
        lcd_print("Enter 1 for gain 1 , 2 for gain 15 ,3 for gain 35, or 4 for gain 50 \r\n");
        input = kpd_input(0); // obtain input using keypad
        lcd_print("\r\n");
        switch (input){
            case 1:
                Kside_to_side_roll = 1;
                break;
            case 2:
                Kside_to_side_roll = 15;
                break;
            case 3:
                Kside_to_side_roll = 35;
                break;
            case 4:
                Kside_to_side_roll = 50;
                break;
            default:
                break;
        }
    } else if (input == 2){
        multipleInput = kpd_input(0);
        Kside_to_side_roll = multipleInput;
    }
}


/*
 * Provided code from lab description. This code adjusts pitch without having to recompile code.
 * Mode1 - keyboard input
 * Mode2 - Keypad input
 */
unsigned int Update_Value(int Constant, unsigned char incr, int maxval, int minval, int mode) {
    int deflt = Kfront_back_pitch;
    char input = 0;
    while(1) {
        if (mode == 1) input = getchar();
        if (mode == 2){
			while (read_keypad() == 0xFF) input = read_keypad();
		}
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
	P3MDOUT |= 0x40;
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
        PCA0 = PCA_START;
    }
    PCA0CN &= 0xC0; // handle other pca interrupt resources
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
