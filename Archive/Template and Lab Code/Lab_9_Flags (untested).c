#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //PortB Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))

osThreadId_t redLED_Id, greenLED_Id, blueLED_Id, control_Id;
//osEventFlagsId_t led_flag;
//osMessageQueueId_t redMsg, greenMsg, blueMsg;

typedef {
    uint8_t cmd;
    uint8_t data;
}myDataPkt;

typedef enum {
  led_on,
  led_off
} led_status_t;

typedef struct {
    uint8_t cmd;
    uint8_t data;
}myDataPkt;

void initLED(void) {
  // Enable CLock to PORTB and PORTD
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  // Configure MUX settings 
  PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
  
  PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);

  PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
  
  //Set Data Direction Registers for PortB and PortD
  PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
  PTD->PDDR |= MASK(BLUE_LED);
}

void ledControl(int color, led_status_t led_status) {
    if (color == RED_LED) {
        if (led_status == led_on) {
            PTB->PCOR = MASK(RED_LED);
        } else {
            PTB->PSOR = MASK(RED_LED);
        }
    } else if (color == GREEN_LED) { // For green led
        if (led_status == led_on) {
            PTB->PCOR = MASK(GREEN_LED);
        } else {
            PTB->PSOR = MASK(GREEN_LED);
        }
    } else {
        if (led_status == led_on) {
            PTB->PCOR = MASK(BLUE_LED);
        } else {
            PTB->PSOR = MASK(BLUE_LED);
        }
    }
}

void led_red_thread(void *argument) {
    
    //myDataPkt myRxData;

    for (;;) {
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        //osMessageQueueGet(redMsg, &myRxData, NULL, osWaitForever);
        //if (myRxData.cmd == 0x01 && myRxData.data == 0x01) {}
        ledControl(RED_LED, led_on);
        osDelay(1000);
        ledControl(RED_LED, led_off);
        osDelay(1000);
    }
}

void led_green_thread(void *argument) {

    //myDataPkt myRxData;

    for (;;) {
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        //osMessageQueueGet(greenMsg, &myRxData, NULL, osWaitForever);
        //if (myRxData.cmd == 0x01 && myRxData.data == 0x01) {}
        ledControl(GREEN_LED, led_on);
        osDelay(1000);
        ledControl(GREEN_LED, led_off);
        osDelay(1000);
    }
}

void led_blue_thread(void *argument) {

    //myDataPkt myRxData;
    
    for (;;) {
        osThreadFlagsWait(0x00000001, osFlagsWaitAny, osWaitForever);
        //osMessageQueueGet(blueMsg, &myRxData, NULL, osWaitForever);
        //if (myRxData.cmd == 0x01 && myRxData.data == 0x01) {}
        ledControl(BLUE_LED, led_on);
        osDelay(1000);
        ledControl(BLUE_LED, led_off);
        osDelay(1000);
    }
}

void control_thread (void *argument) {
    for (;;) {
        osThreadFlagsSet(redLED_Id, 0x0000001);
        osDelay(1000);
        osThreadFlagsSet(greenLED_Id, 0x0000001);
        osDelay(1000)
        osThreadFlagsSet(blueLED_Id, 0x0000001);
        osDelay(1000);
    }
}

// PART 5
/*
void control_thread (void *argument) {
    
    myDataPkt myData;
    myData.cmd = 0x01;
    myData.data = 0x01;

    for (;;) {
        osMessageQueuePut(redMsg, &myData, NULL, 0);
        osDelay(2000);
        osMessageQueuePut(greenMsg, &myData, NULL, 0);
        osDelay(2000);
        osMessageQueuePut(blueMsg, &myData, NULL, 0);
        osDelay(2000);
    }
}
*/

void offRGB(void) { // Off all LED
  PTB->PSOR = MASK(RED_LED);
  PTB->PSOR = MASK(GREEN_LED);
  PTD->PSOR = MASK(BLUE_LED);
}

int main (void) {
    SystemCoreClockUpdate();
    InitGPIO();
    offRGB();

    osKernelInitialize(); // Initialise CMSIS-RTOS2
    redLED_Id = osThreadNew(led_red_thread, NULL, NULL);
    greenLED_Id = osThreadNew(led_green_thread, NULL, NULL); 
    blueLED_Id = osThreadNew(led_blue_thread, NULL, NULL);
    control_Id = osThreadNew(control_thread, NULL, NULL);
    //redMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
    //greenMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
    //blueMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
    osKernelStart();

    for (;;) {}
}