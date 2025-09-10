/*
 * Freescale Cup linescan camera code
 *
 *	This method of capturing data from the line
 *	scan cameras uses a flex timer module, periodic
 *	interrupt timer, an ADC, and some GPIOs.
 *	CLK and SI are driven with GPIO because the FTM2
 *	module used doesn't have any output pins on the
 * 	development board. The PIT timer is used to 
 *  control the integration period. When it overflows
 * 	it enables interrupts from the FTM2 module and then
 *	the FTM2 and ADC are active for 128 clock cycles to
 *	generate the camera signals and read the camera 
 *  output.
 *
 *	PTB8		- camera CLK
 *	PTB23 		- camera SI
 *  ADC0_DP1 	- camera AOut
 *
 * Author:  Alex Avery
 * Created:  11/20/15
 * Modified:  11/23/15
 */

#include "MK64F12.h"
#include "uart.h"
#include "isr.h"
#include "pwm.h"
#include "stdio.h"

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 

// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME 0.0075f

// Not sure what this does exactly??
#define STRAIGHT 7.75

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void delay(int del);
void initialize();
void convolve(const int32_t *x, const double *h, int32_t *y, const int xSize, const int hSize);


// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;

// clkval toggles with each FTM interrupt
int clkval = 0;

// line stores the current array of camera data
int32_t line[128];

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 0;
int capcnt = 0;
char str[100];

// Magnitude of increase necessary to update DL/DR
int magnitude = 0;

// Margin for when to turn
int margin = 5;

// Speed to travel on straight sections
int motorSpeed = 45;

// Output of convolve
int32_t yNorm[128];
int32_t yEdge[128];
int32_t yBoost[128];

int32_t motorFreq = 10000; /* Frequency = 10 kHz */
int32_t servoFreq = 50; /* Frequency = 50 Hz */

// ADC0VAL holds the current ADC value
uint32_t ADC0VAL;

int main(void)
{
	// Initialize UART and PWM
	initialize();
    
	// Start off, start running when switch 2 is pressed
	boolean isRunning = false;
	
	// PID Parameters
	int Tdes = 64;				// Desired center of the camera
	int Tact = 0;					// Actual center of the camera
	int servoPos = 50;		// Current servo position
	int servoPosOld = 50; // Previous servo position
	double err = 0;					// Current error between Tdes and Tact
	double errOld1 = 0; 			// Previous error
	double errOld2 = 0;			// Previous previous error
	double absErr = 0;				// Absolute error, to use for motor speed
	double Kp = 0.5; 		// P parameter
	double Ki = 0.00; 		// I parameter
	double Kd = -0.2;			// D parameter
	
	// Car Logic
	while(true){
        
		// Normalize trace
		double hNorm[3] = {1, 1, 1};
		convolve(line, hNorm, yNorm, sizeof(line) / sizeof(line[0]), sizeof(hNorm) / sizeof(hNorm[0]));
		for(int i = 0; i < sizeof(line) / sizeof(line[0]); i++){
			yNorm[i] /= 3;
		}
		
		// Use derivative to find edges
		double hEdge[3] = {-1, 0, 1};
		convolve(yNorm, hEdge, yEdge, sizeof(yNorm) / sizeof(yNorm[0]), sizeof(hEdge) / sizeof(hEdge[0]));
		
		// Boost the peaks of the output
		double hBoost[3] = {-1, 3, -1};
		convolve(yEdge, hBoost, yBoost, sizeof(yEdge) / sizeof(yEdge[0]), sizeof(hBoost) / sizeof(hBoost[0]));
		
		// Matlab plotting
		if(false){
			// send the array over uart
			sprintf(str, "%i\n\r", -1); // start value
			put(str);
				
			for (int i = 0; i < 127; i++) {
				sprintf(str, "%i\n", yBoost[i]);
				put(str);
			}
		
			sprintf(str, "%i\n\r", -2); // end value
			put(str);
		}
        
		// Solve for DR and DL using derivative data
		int leftEdge = 0;
		int rightEdge = 0;
		for (int i = 0; i < 128; i++) {
			if((yBoost[i] < yBoost[leftEdge] - magnitude) && i > 10){
				leftEdge = i;
			}
			if((yBoost[i] > yBoost[rightEdge] + magnitude) && i < 118){
				rightEdge = i;
			}
		}
		
		// If already running, stop when switch 2 is pressed
		if(((GPIOC_PDIR & (1 << 6)) == 0) && isRunning == true){
			isRunning = false;
			delay(50);
		}
		
		// If not running, start when switch 2 is pressed
		else if(((GPIOC_PDIR & (1 << 6)) == 0)){
			isRunning = true;
			delay(100);
		}
		
		else{
			// Do nothing
		}
		
		// PID Servo Control
		Tact = (leftEdge + rightEdge) / 2; // Assign based on camera data
		err = (float)Tdes - (float)Tact; // 64 - Tact
		if(err > 30){
			err = 30;
		}
		if(err < -30){
			err = -30;
		}
		
		//servoPos = STRAIGHT + Kp * ( 64 - Tact );
		servoPos = (int)(50.0 - 
								(Kp * (err)) + 
								(Ki * ((err + errOld1) / 2.0)) + 
								(Kd * (err - (2.0 * errOld1) + errOld2)));
		
		if(servoPos > 100){
			servoPos = 100;
		}
		if(servoPos < 0){
			servoPos = 0;
		}
		
		//servoPosOld = servoPos;
		errOld2 = errOld1;
		errOld1 = err;
		
		// Find error for motor speed
		if(err < 0){
			absErr = -err;
		}
		else{ // err >= 0
			absErr = err;
		}
		
		// TESTING: Check DL/DR values
		if(false){
			put("-----------------------\n\r");
			sprintf(str, "Tact: %i\n\r", Tact);
			put(str);
			sprintf(str, "SPos: %i\n\r", servoPos);
			put(str);
			sprintf(str, " Err: %i\n\r", err);
			put(str);
			sprintf(str, "MSpd: %i\n\r", motorSpeed + (5 - (absErr/4)));
			put(str);
			delay(40);
		}
		
		// Only run when switch 2 is pressed
		// TODO: Update wheel speed using the PID output
		// TODO: Update servo position with servoPos
		if(isRunning == true){
			SetDutyCycle1(motorSpeed + (5 - (absErr)), motorFreq, 0);
			SetDutyCycle2(motorSpeed + (5 - (absErr)), motorFreq, 0);
			SetDutyCycleServo(servoPos, servoFreq);
		}
		else{ // isRunning == false
			// stop
			SetDutyCycle1(0, motorFreq, 0);
			SetDutyCycle2(0, motorFreq, 0);
			SetDutyCycleServo(servoPos, servoFreq);
		}
		delay(5);
	}
	
    // Camera testing
	for(;;) {
		//GPIOB_PTOR = (1 << 9);
		if (debugcamdata) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= 500) {
				GPIOB_PCOR |= (1 << 22);
								
				// send the array over uart
				sprintf(str, "%i\n\r", -1); // start value
				put(str);
				
				for (int i = 0; i < 127; i++) {
					sprintf(str, "%i\n\r", line[i]);
					put(str);
					//sprintf(str,"Norm: %f \n\r", y[i]);
					//put(str);
				}
				
				sprintf(str, "%i\n\r", -2); // end value
				put(str);
				
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}
	} //for
	
	return 0;
}

/**
 * Performs a convolution using x and h, then
 * stores the result into y directly.
 */
void convolve(const int32_t *x, const double *h, int32_t *y, const int xSize, const int hSize){
	for(int i = (hSize-1); i < xSize; i++){
		double sum = 0.0;
		for(int j = (hSize-1); j >= 0; j--){
			sum += h[j] * x[i - j];
		}
		// Convert to positive
		//if(sum < 0){
		//	y[i] = -sum;
		//}
		//else{
		y[i] = sum;
		//}
	}
}

/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}

void initialize()
{
	// Initialize UART
	init_uart();	
	
	// Initialize Switch 2
	init_button();
	
	// Initialize the FlexTimer
	InitPWM();
	
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
}



/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	ADC0VAL = ADC0_RA;
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  FTM2_SC &= ~FTM_SC_TOF_MASK;
	
	// Toggle clk
	GPIOB_PTOR = (1 << 9);
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = (int32_t)ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	FTM2_MOD |= FTM_MOD_MOD_MASK & ((DEFAULT_SYSTEM_CLOCK / 10000) - 1);
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}

/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = FTM_CNT_COUNT_MASK;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN &= ~FTM_CNTIN_INIT_MASK;
	
	// Set the period (~10us)
	FTM2_MOD |= FTM_MOD_MOD_MASK & ((DEFAULT_SYSTEM_CLOCK / 10000) - 1);
	
	// 50% duty
	FTM2_C0V = FTM_CnV_VAL_MASK & ((DEFAULT_SYSTEM_CLOCK / 20000) - 1);
	
	// Set edge-aligned mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	//FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_CH0TRIG_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC |= FTM_SC_PS(0x0);
	FTM2_SC |= FTM_SC_CLKS(0x1);
	
	// Set up interrupt
	FTM2_SC |= FTM_SC_TOIE_MASK;
	NVIC_EnableIRQ(FTM2_IRQn);
	
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	// Setup periodic interrupt timer (PIT)
	//PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	PIT_MCR = 0x00;
	
	// Enable timers to continue in debug mode
	PIT_MCR &= ~PIT_MCR_FRZ_MASK; // In case you need to debug
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 = DEFAULT_SYSTEM_CLOCK / 50;
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}

/* Set up pins for GPIO
* 	PTB9 		- camera clk
*	PTB23		- camera SI
*	PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results

	//initialize clocks for each different port used.
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable clock for Red LED

	//Configure Port Control Register for Inputs with pull enable and pull up resistor
	// Configure mux for Outputs
	PORTB_PCR9 = PORT_PCR_MUX(1);  // Enable camera clk
	PORTB_PCR23 = PORT_PCR_MUX(1); // Enable camera SI
	PORTB_PCR22 = PORT_PCR_MUX(1); // Enable red LED
	
	//Enable GPIO pins (from function header)
	GPIOB_PDDR |= (1 << 9);
	GPIOB_PDDR |= (1 << 23);
	GPIOB_PDDR |= (1 << 22);
	
	GPIOB_PDOR |= (1 << 9);
	GPIOB_PDOR |= (1 << 23);
	GPIOB_PDOR |= (1 << 22);
	
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
	unsigned int calib;
	
	// Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	
	// Single ended 16 bit conversion, no clock divider
	ADC0_CFG1 |= ADC_CFG1_ADIV(0x0);
	ADC0_CFG1 |= ADC_CFG1_MODE(0x3);
    
	// Do ADC Calibration for Singled Ended ADC. Do not touch.
	ADC0_SC3 = ADC_SC3_CAL_MASK;
	while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
	calib = ADC0_CLP0; 
	calib += ADC0_CLP1; 
	calib += ADC0_CLP2;
	calib += ADC0_CLP3; 
	calib += ADC0_CLP4; 
	calib += ADC0_CLPS;
	calib = calib >> 1; 
	calib |= 0x8000;
	ADC0_PG = calib;
    
	// Select hardware trigger.
	ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    
	// Set to single ended mode	
	ADC0_SC1A &= ~ADC_SC1_DIFF_MASK;
	ADC0_SC1A = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0);
	
	// Set up FTM2 trigger on ADC0
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10); // FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK; // Alternative trigger en.
	SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK; // Pretrigger A
	
	// Enable NVIC interrupt
	NVIC_EnableIRQ(ADC0_IRQn);
}
