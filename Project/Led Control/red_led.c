#include "MKL25Z4.h"


#define PORTB0_Pin 0    //Only Port B Pin 0 is being used
#define MASK(x) (1 << (x))

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {}
}

/*Delay Function(s)*/

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
			nof--;
  }
}
	
void delay_ms(uint32_t milliseconds) {
    // Assuming 48MHz clock
    // Each loop takes approximately 1 / (48e6 / 5) = 0.10416667 microseconds
    // This function will delay approximately milliseconds milliseconds
    for (volatile uint32_t i = 0; i < milliseconds * 48000; ++i) {
        __NOP(); // No Operation
    }
}


/* Main Red LED Codes */
void initRedLed(){
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
    PORTB->PCR[PORTB0_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PORTB0_Pin] |= PORT_PCR_MUX(1);
    
    PTB->PDDR |= MASK(PORTB0_Pin);
}
	
void toggle500ms (){    // Red LEDs go on for 500ms and off for 500ms
    while(1){
    PTB->PCOR |= MASK(PORTB0_Pin);
    osDelay(500);
    PTB->PSOR |= MASK(PORTB0_Pin);
    osDelay(500);
    }
}
	
void toggle250ms (){    // Red LEDs go on for 250ms and off for 250ms
    while(1){
    PTB->PCOR |= MASK(PORTB0_Pin);
    osDelay(250);
    PTB->PSOR |= MASK(PORTB0_Pin);
    osDelay(250);
    }
}
 
int main (void) {
 
    // System Initialization
    SystemCoreClockUpdate();
    // ...
    initRedLed();
 
    osKernelInitialize();                 // Initialize CMSIS-RTOS
    osThreadNew(toggle250ms, NULL, NULL);    // Create application main thread
    osKernelStart();                      // Start thread execution
    for (;;) {}
}
