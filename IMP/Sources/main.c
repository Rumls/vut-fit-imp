#include "MK60D10.h"

#include <stdio.h>
#include <ctype.h>
#include <string.h>

/* Macros for bit-level registers manipulation */
#define GPIO_PIN_MASK 0x1Fu
#define GPIO_PIN(x) (((1)<<(x & GPIO_PIN_MASK)))

/* digits from left to right */
#define C1LED	~0x1000 // digit 1
#define C2LED 	~0x4000	// digit 2
#define C3LED 	~0x8000	// digit 3
#define C4LED	~0x2000	// digit 4

/* LED segments */
#define A 	0x200 	// PTA9
#define B 	0x4 	// PTA2
#define C 	0x400 	// PTA10
#define D 	0x800 	// PTA11
#define E 	0x80 	// PTA7
#define F 	0x8 	// PTA3
#define G 	0x100 	// PTA8
#define DP 	0x40 	// PTA6

#define LEDSEG 	0x0FCC	// all display segments
#define LEDC	0xF000	// all control bits

float a_h, a_l; 							// coeficients for filters
float y, x, yx, yx_prev = 0, avg_y = 0.0; 	// variables for filters
char result[10] = "0000";					// final result to display
int temp = 0, temp_prev;					// temporary stored ADC value output
int cnt = 0;								// counter
uint32_t currentCNR = 0U;					// for LPTMR counter
int prev_out = 0, peak = 0, out = 0;		// for work with filtered ADC output
int up = 0, beat = 0;						// rising edge detection and peak counting
int bpm;									// final beats per minute

unsigned int compare = 0x2710, index = 0;	// compare for triggering LPTMR IRQhandler and result indexing

/* A delay function */
void delay(long long bound)
{
  long long i;
  for(i=0;i<bound;i++);
}

/* Initialize the MCU - basic clock settings, turning the watchdog off */
void MCUInit(void)  {
    MCG_C4 |= ( MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x01) );
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0x00);
    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}

/* Initialize ports for the LED display */
void PortsInit(void)
{
    /* Turn on all port clocks */
    SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;

    /* Set corresponding PTA pins of LED display segments for GPIO functionality */
    PORTA->PCR[2] = PORT_PCR_MUX(0x01);		// P1-30
    PORTA->PCR[3] = PORT_PCR_MUX(0x01);		// P1-29
    PORTA->PCR[6] = PORT_PCR_MUX(0x01);		// P1-25
    PORTA->PCR[7] = PORT_PCR_MUX(0x01);		// P1-27
    PORTA->PCR[8] = PORT_PCR_MUX(0x01);		// P1-23
    PORTA->PCR[9] = PORT_PCR_MUX(0x01);		// P1-28
    PORTA->PCR[10] = PORT_PCR_MUX(0x01);	// P1-24
    PORTA->PCR[11] = PORT_PCR_MUX(0x01);	// P1-26

    /* Set corresponding PTB pins of LED display control bits for GPIO functionality */
    PORTD->PCR[12] = PORT_PCR_MUX(0x01); 	// P1-19
    PORTD->PCR[13] = PORT_PCR_MUX(0x01); 	// P1-20
    PORTD->PCR[14] = PORT_PCR_MUX(0x01); 	// P1-22
    PORTD->PCR[15] = PORT_PCR_MUX(0x01); 	// P1-21

    /* Setting all display pins and turning them off*/
    PTA->PDDR = GPIO_PDDR_PDD(LEDSEG);
    PTD->PDDR = GPIO_PDDR_PDD(LEDC);
    PTA->PDOR &= GPIO_PDOR_PDO(~LEDSEG);
    PTD->PDOR &= GPIO_PDOR_PDO(~LEDC);
}

/* Initialize ports for the LED display */
void LPTMR0Init(int count)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;	// Enable clock to LPTMR
    LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;		// Turn OFF LPTMR to perform setup
    LPTMR0_PSR = (LPTMR_PSR_PRESCALE(0)		// 0000 is div 2
                 | LPTMR_PSR_PBYP_MASK		// LPO feeds directly to LPT
                 | LPTMR_PSR_PCS(1));		// use the choice of clock
    LPTMR0_CMR = count;						// Set compare value
    LPTMR0_CSR =(LPTMR_CSR_TCF_MASK			// Clear any pending interrupt (now)
                 | LPTMR_CSR_TIE_MASK);		// LPT interrupt enabled

    NVIC_EnableIRQ(LPTMR0_IRQn);         	// enable interrupts from LPTMR0
    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;    	// Turn ON LPTMR0 and start counting
}

/* Initialize analog to digital conversion for the analog output */
void ADC0_Init(void)
{
	NVIC_ClearPendingIRQ(ADC0_IRQn); 	// clearing any pending interupt for ADC0
    NVIC_EnableIRQ(ADC0_IRQn);			// enabling interupt for ADC0

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;	// turning on ADC0 clock

	ADC0_CFG1 = (ADC_CFG1_MODE(3) | ADC_CFG1_ADIV(2) | ADC_CFG1_ADICLK(0));	// ADC0 configuration, 16bit mode and using bus clock
	ADC0_SC1A = ADC_SC1_ADCH(31);		// turning of all channels
}

/* Function for easy setting of result digits to corresponding display digit by indexing control pins*/
void controlNo(int Cno)
{
	switch(Cno) // cases are inverted for leaving most left display digits empty if (result digits < display digits)
	{

		case 3:
			PTD->PDOR = GPIO_PDOR_PDO(C1LED);
			PTD->PDOR &= GPIO_PDOR_PDO(~C2LED | ~C3LED | ~C4LED);
			break;

		case 2:
			PTD->PDOR = GPIO_PDOR_PDO(C2LED);
			PTD->PDOR &= GPIO_PDOR_PDO(~C1LED | ~C3LED | ~C4LED);
			break;

		case 1:
			PTD->PDOR = GPIO_PDOR_PDO(C3LED);
			PTD->PDOR &= GPIO_PDOR_PDO(~C1LED | ~C2LED | ~C4LED);
			break;

		case 0:
			PTD->PDOR = GPIO_PDOR_PDO(C4LED);
			PTD->PDOR &= GPIO_PDOR_PDO(~C1LED | ~C2LED | ~C3LED);
			break;

	}
}

/* function to build numbers from defined LED segments */
void pushNo(int no, int point)
{
	PTA->PDOR &= GPIO_PDOR_PDO(~LEDSEG);	// clearing segments

	switch(no)
	{

		case 0:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(E);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			break;

		case 1:
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			break;

		case 2:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			PTA->PDOR |= GPIO_PDOR_PDO(E);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			break;

		case 3:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;

		case 4:
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;

		case 5:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;

		case 6:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(E);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;

		case 7:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			break;

		case 8:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(E);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;

		case 9:
			PTA->PDOR |= GPIO_PDOR_PDO(A);
			PTA->PDOR |= GPIO_PDOR_PDO(B);
			PTA->PDOR |= GPIO_PDOR_PDO(C);
			PTA->PDOR |= GPIO_PDOR_PDO(D);
			PTA->PDOR |= GPIO_PDOR_PDO(F);
			PTA->PDOR |= GPIO_PDOR_PDO(G);
			break;
	}

	if (point)
	{
		PTA->PDOR |= GPIO_PDOR_PDO(DP);
	}
}

/* writing result digits to display digits */
void writeVal(char *res)
{
	for(index = 0; index < strlen(res); index++) 			// getting digits and control one by one
	{
		if (isdigit(res[strlen(res) - 1 - index]))
		{
			pushNo(res[strlen(res) - 1 - index]-'0', 0);	// selection is inverted so taking result from last digit
			controlNo(index);								// switchnig control for different digit
			delay(2000);									// delay for proper switching to diffrent digit
		}
		else
		{
			PTA->PDOR = GPIO_PDOR_PDO(G);					// if result is not a digit display "-"
		}
	}
}

/* handler for work with analog signal */
void ADC0_IRQHandler(void)
{
    if (ADC0_SC1A & ADC_SC1_COCO_MASK) 						// waits until convrsion is done
    {
		temp = ADC0_RA;										// getting result of conversion from ADC0

		while(temp > 10000)									// rounding for better work and debugging output for display
		{
			temp = temp/10;
		}

		x = (float)temp;									// casting for work with floating point

		/* RC / (RC + dt) time between conversions (dt) simplified to an approximated constant */
		a_h = 0.99375;										// constant coeficient for high pass filter based of minimum beat frequency 0.6Hz
		a_l = 0.9815;										// constant coeficient for low pass filter based of maximum beat frequency 2Hz

		yx = (a_l * x) + ((1 - a_l) * yx_prev);				// low pass filter (fed to a high pass filter)
		y = a_h*y + a_h*(yx - yx_prev);						// high pass filter (with a low pass filter as input creating extremely simple band pass filter)
		yx_prev = yx;

		cnt++;
		avg_y += y;

		/* averaging 24 samples to smooth the the output and getting rid of noise and second pulse peak */
		if(cnt >= 24)
		{
			avg_y = avg_y/cnt;
			while(avg_y > 10000)
			{
				avg_y = avg_y/10;
			}
			out = (int)avg_y;
			cnt = 0;
			avg_y = 0.0;
		}
    }
}

/* every 10 seconds calculating beats per minute */
void LPTMR0_IRQHandler(void)
{
    LPTMR0_CMR = compare;                // writing our time interval (10s) to CMR register
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // writing 1 to TCF tclear the flag

    peak = 0;

    /* casting variables as floating points for math */
    float f_currentCNR = (float)currentCNR;
    float f_beat = (float)beat;

    /* getting bpm from number of beats(f_bpm)
     * and last time the beat was detected in 10 second interval for better bpm accuracy(f_currentCNR) */
    float f_bpm = (f_beat*1000/f_currentCNR)*60;
    bpm = (int)f_bpm;
    sprintf(result, "%d", bpm);			// getting bpm to a final result
    beat = 0;
}

int main(void)
{
    MCUInit();
    PortsInit();
    LPTMR0Init(compare);
    ADC0_Init();

    while (1)
    {

    	ADC0_SC1A = (ADC_SC1_AIEN(1)  | ADC_SC1_ADCH(0));

    	LPTMR0->CNR = 1;

    	if(out >= 10) 											// getting rid of persisting low value noise
    	{
			if(up)												// rising edge of function
			{
				if(prev_out > out)								// this is our peak
				{
					beat++;										// valid beat
					currentCNR = LPTMR0->CNR;					// getting the latest time of the beat
					up = 0;										// now we are on falling edge
				}
				else
				{
					prev_out = out;
				}
			}
			else
			{
				if(prev_out < out)								// this is our valley
				{
					up = 1;										// now we are on rising edge
				}
				else
				{
					prev_out = out;
				}
			}
    	}

    	writeVal(result);										// final result of bpm
    }
    return 0;
}
