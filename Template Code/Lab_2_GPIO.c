#include "MKL25Z4.h"

#define RED_LED 18	 // PORTB Pin 18
#define GREEN_LED 19 // PORTB Pin 19
#define BLUE_LED 1	 // PORTD Pin 1
#define MASK(x) (1 << (x))

unsigned int counter = 0;

void initGPIO(void) {
	// Enable Clock to PortB and PortD
	// SCGC5 - System Clock Gating Control Register 5
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));

	// Configure MUX settings to make all 3 pins GPIO
	// This is done because all the ports have multiple functions, including GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);

	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);

	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);

	// Set Data Direction Registers for PortB and PortD (1 = out, 0 = in)
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

void offRGB(void) {
	// Port SET output register because it is active LOW
	PTB->PSOR = MASK(RED_LED) | MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);
}

void led_control(int color) {
	offRGB();
	switch(color) {
	case RED_LED:
		// Port CLEAR output register
		// Again, active LOW so clear to enable
		PTB->PCOR = MASK(RED_LED);
		break;
	case GREEN_LED:
		PTB->PCOR = MASK(GREEN_LED);
		break;
	case BLUE_LED:
		PTD->PCOR = MASK(BLUE_LED);
		break;
	default:
		offRGB();
		break;
	}
}

static void delay (volatile uint32_t nof) {
	while(nof) {
		__asm("NOP"); // Assembly code - provided by lecture
		nof--;
	}
}

int main(void)
{
	SystemCoreClockUpdate();
	initGPIO();
	while (1)
	{
		counter++;
		if (counter > 0x0F)
			counter = 0;
		led_control(RED_LED);
		delay(0x80000);
		led_control(GREEN_LED);
		delay(0x80000);
		led_control(BLUE_LED);
		delay(0x80000);
	}
}