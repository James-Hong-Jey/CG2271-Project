#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //PortB Pin 19
#define MASK(x) (1 << (x))

osSemaphoreId_t mySem;

typedef enum {
  led_on,
  led_off
} led_status_t;

const osThreadAttr_t thread_attr = {
    .priority = osPriorityNormal1
};

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

void initLED(void) {
  // Enable CLock to PORTB and PORTD
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  // Configure MUX settings 
  PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; // Clear bit 10 to 8
  PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); // Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
  
  PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; // Clear bit 10 to 8
  PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); // Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
  
  //Set Data Direction Registers for PortB and PortD
  PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
}

void ledControl(int color, led_status_t led_status) {
    if (color == RED_LED) {
        if (led_status == led_on) {
            PTB->PCOR = MASK(RED_LED);
        } else {
            PTB->PSOR = MASK(RED_LED);
        }
    } else { // For green led
        if (led_status == led_on) {
            PTB->PCOR = MASK(GREEN_LED);
        } else {
            PTB->PSOR = MASK(GREEN_LED);
        }
    }
}

void led_red_thread (void *argument) {
    for (;;) {
        osSemaphoreAcquire(mySem, osWaitForever);

        ledControl(RED_LED, led_on);
        osDelay(1000); 
        //delay(0x80000);
        ledControl(RED_LED, led_off);
        osDelay(1000); 
        //delay(0x80000);

        osSemaphoreRelease(mySem); // Remove for part 3
    }
}

void led_green_thread (void *argument) {
    for (;;) {
        osSemaphoreAcquire(mySem, osWaitForever); // Binary semaphore

        ledControl(GREEN_LED, led_on);
        osDelay(1000);
        //delay(0x80000);
        ledControl(GREEN_LED, led_off);
        osDelay(1000); 
        //delay(0x80000);

        osSemaphoreRelease(mySem); // Remove for part 3
    }
}

void offRGB(void) { // Off all LED
  PTB->PSOR = MASK(RED_LED);
  PTB->PSOR = MASK(GREEN_LED);
}

// PART 3
/**
 * 
 * For push button to release semaphore. Green and red dont release.
*/
/*
void PORTD_IRQHandler() {
    // Clear Pending IRQ
    NVIC_ClearPendingIRQ(PORTD_IRQn);

    delay(0x80000);
    myStatus = osSemaphoreRelease(mySem); // Semaphore release must only come from IRQ handler

    // Clear INT Flag
    PORTD->ISFR |= MASK(SW_POS);
}
*/

int main (void) {
    SystemCoreClockUpdate();
    InitGPIO();
    offRGB();

    osKernelInitialize(); // Initialise CMSIS-RTOS2
    mySem = osSemaphoreNew(1, 1, NULL);
    //mySem = osSemaphoreNew(1, 0, NULL); // Part 3
    osThreadNew(led_red_thread, NULL, NULL); // Red run first
    osThreadNew(led_green_thread, NULL, NULL); // Goes to BLOCKED state first
    osKernelStart();

    for (;;) {} // only reached in case of error
}