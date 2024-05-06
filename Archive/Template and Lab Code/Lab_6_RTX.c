/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
 
#define RED_LED 18	 // PORTB Pin 18
#define GREEN_LED 19 // PORTB Pin 19
#define BLUE_LED 1	 // PORTD Pin 1
#define MASK(x) (1 << (x))

typedef enum {
  led_on,
  led_off
}led_status_t;

/*----------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/
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

void led_control(int colour, led_status_t led_status) {
  if (colour == RED_LED ) {
	if (led_status == led_on) {
		PTB->PCOR = MASK(RED_LED);
	} else {
		PTB->PSOR = MASK(RED_LED);
	}
  } else if (colour == GREEN_LED) {
	if (led_status == led_on) {
		PTB->PCOR = MASK(GREEN_LED);
	} else {
		PTB->PSOR = MASK(GREEN_LED);
	}
  } else if (colour == BLUE_LED) {
	if (led_status == led_on) {
		PTD->PCOR = MASK(BLUE_LED);
	} else {
		PTD->PSOR = MASK(BLUE_LED);
	}
  } else {
	offRGB();
  }
}

static void delay (volatile uint32_t nof) {
	while(nof) {
		__asm("NOP"); // Assembly code - provided by lecture
		nof--;
	}
}

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

void led_red_thread (void *argument) {
  for(;;) {
    led_control(RED_LED, led_on);
    delay(0x80000);
    //osDelay(1000);
    led_control(RED_LED, led_off);
    delay(0x80000);
    //osDelay(1000);
  }
}

void led_green_thread (void *argument) {
  for(;;) {
    led_control(GREEN_LED, led_on);
    delay(0x80000);
    //osDelay(1000);
    led_control(GREEN_LED, led_off);
    delay(0x80000);
    //osDelay(1000);
  }
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  initGPIO();
  offRGB();
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(led_red_thread, NULL, NULL);    // Create application main thread
  osThreadNew(led_green_thread, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
