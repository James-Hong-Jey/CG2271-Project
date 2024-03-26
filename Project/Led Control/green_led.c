#include "MKL25Z4.h"

#define GREEN_LED_1 2 //PortE Pin 2
#define GREEN_LED_2 3 //PortE Pin 3
#define GREEN_LED_3 4 //PortE Pin 4
#define GREEN_LED_4 5 //PortE Pin 5
#define GREEN_LED_5 20 //PortE Pin 20
#define GREEN_LED_6 21 //PortE Pin 21
#define GREEN_LED_7 22 //PortE Pin 22
#define GREEN_LED_8 23 //PortE Pin 23
#define GREEN_LED_9 29 //PortE Pin 29
#define GREEN_LED_10 30 //PortE Pin 30
#define MASK(x) (1 << (x))



// 0110 0000 1111 0000 0000 0000 0011 1100

unsigned int counter = 0;

void  InitGPIO(void) {
  //Enable CLock to PORTB and PORTD
  SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK); 
  
  //Configure MUX settings 
  PORTE->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1); //Assign output
  
	PORTE->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1); //Assign output 
	
  PORTE->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1); //Assign output 
	
  PORTE->PCR[GREEN_LED_9] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_9] |= PORT_PCR_MUX(1); //Assign output 
	 
  PORTE->PCR[GREEN_LED_10] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTE->PCR[GREEN_LED_10] |= PORT_PCR_MUX(1); //Assign output 
	 

    
  //Set Data Direction Registers for PortB and PortD
  PTE->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | 
                MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
  
}

/*Delay Function*/

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
			nof--;
  }
}

void led_toggler(int colour_current) { //, int colour_previou
	PTE->PCOR = (MASK(2)|MASK(3)|MASK(4)|MASK(5)|MASK(20)|MASK(21)|MASK(22)|MASK(23)|MASK(29)|MASK(30));
    //PTE->PCOR = MASK(colour_previous);
    PTE->PSOR = MASK(colour_current);
    delay(0xf0f0);
}

void ledAllOn() { //, int colour_previou
	PTE->PSOR = (MASK(2)|MASK(3)|MASK(4)|MASK(5)|MASK(20)|MASK(21)|MASK(22)|MASK(23)|MASK(29)|MASK(30));
    //delay();
}

void ledToggleFromLeftToRight(){
    while (1){
        led_toggler(GREEN_LED_1);
        led_toggler(GREEN_LED_2);
        led_toggler(GREEN_LED_3);
        led_toggler(GREEN_LED_4);
        led_toggler(GREEN_LED_5);
        led_toggler(GREEN_LED_6);
        led_toggler(GREEN_LED_7);
        led_toggler(GREEN_LED_8);
        led_toggler(GREEN_LED_9);
        led_toggler(GREEN_LED_10); // final state
        led_toggler(GREEN_LED_10);
        led_toggler(GREEN_LED_9);
        led_toggler(GREEN_LED_8);
        led_toggler(GREEN_LED_7);
        led_toggler(GREEN_LED_6);
        led_toggler(GREEN_LED_5);
        led_toggler(GREEN_LED_4);
        led_toggler(GREEN_LED_3);
        led_toggler(GREEN_LED_2);
        led_toggler(GREEN_LED_1);
    }
}

int main(void){
    SystemCoreClockUpdate();
    InitGPIO();
	//ledToggleFromLeftToRight();     // Use when you want the LEDs to toggle 
	//ledAllOn();                     // Use when you want the LEDs to all light up
    
}
