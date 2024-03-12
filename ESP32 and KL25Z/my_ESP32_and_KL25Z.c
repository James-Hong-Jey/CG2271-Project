#include "MKL25Z4.h"

#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //PortB Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define Q_SIZE 32

typedef enum {
  led_on,
  led_off
} led_status_t;



typedef struct { 
	unsigned char DATA[Q_SIZE];
	// Done halfway; copy the rest from lect 8 page 18
	unsigned int HEAD;
	unsigned int TAIL;
	unsigned int SIZE;
} Q_T;

Q_T tx_q, rx_q;
volatile uint8_t rx_IRQ_data = 0;

void Q_Init(Q_T *q) {
	unsigned int i;
	for(i=0; i < Q_SIZE; i++) q->DATA[i] = 0; // Initialise to 0
	q->HEAD = 0;
	q->TAIL = 0;
	q->SIZE = 0;
}

int Q_Empty(Q_T *q) {
	return q->SIZE = 0;
}

int Q_Full(Q_T *q) {
	return q->SIZE == Q_SIZE;
}

int Q_Enqueue(Q_T *q, unsigned char d) {
	if(Q_Full(q)) return 0; // Queue full - Failure
	q->DATA[q->TAIL++] = d;
	q->TAIL %= Q_SIZE; // This makes the list circular
	q->SIZE++;
	return 1; // Success
}

unsigned char Q_Dequeue(Q_T *q) {
	if(Q_Empty(q)) return 0; // Nothing to dequeue
	unsigned char t = q->DATA[q->HEAD];
	q->DATA[q->HEAD++] = 0;
	q->HEAD %= Q_SIZE;
	q->SIZE--;
	return t;
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);

	// Transmitter ready
	if(UART2->S1 & UART_S1_TDRE_MASK) {
		if(Q_Empty(&tx_q)) {
            // Queue Empty so disable interrupts
            UART2->C2 &= ~UART_C2_TIE_MASK; 
        } else {
            UART2->D = Q_Dequeue(&tx_q);
        }
	}

	// If receiver is full
	if(UART2->S1 & UART_S1_RDRF_MASK) {
        if(Q_Full(&rx_q)) {
            while(1); // TODO: Handle error
        } else {
            rx_IRQ_data = UART2->D;
        }
	}

	// Error checking
	if(UART2->S1 & (UART_S1_OR_MASK | 
                    UART_S1_NF_MASK | 
                    UART_S1_FE_MASK | 
                    UART_S1_PF_MASK)) {
		// TODO: Handle error
		// TODO: Clear Flag
		return;
	}
}

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

void initLED(void) {
	//Enable CLock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	//Configure MUX settings 
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK; //clear bit 10 to 8
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1); //Select GPIO, from datasheet it is alternative 1, bits 10-8 are 001 so we use 1
	
	//Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

void offAllLed(void) { //off all LED
	PTB->PSOR = MASK(RED_LED);
	PTB->PSOR = MASK(GREEN_LED);
	PTD->PSOR = MASK(BLUE_LED);
}

void initUART2(uint32_t baud_rate) {

    uint32_t divisor, bus_clock;
	
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK; //enable clock to UART2 module
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //enable clock to PORT E module

    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK; //clear PORTE_PCR22
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4); //enable pin mux alternative 4, page 163, which enables UART2_TX

    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK; //clear PORTE_PCR23
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4); //enable UART2_RX

    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //disable tx and rx before configuration

    bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
    divisor = bus_clock / (baud_rate * 16); //oversampling of UART 2, 16*baud rate is most commonly used, each serial bit sampled 16 times
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);

		// No parity, 8 bits, two stop bits, other settings
    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;

    // Queue
    //NCIV_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // Enable TX and RX Interrupts
    // UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;

    // Initialise both tx_q and rx_q
    Q_Init(&tx_q);
    Q_Init(&rx_q);

    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //Enable TX and RX
}

/* UART2 Transmit Poll */
void UART2_Transmit_Poll(uint8_t data) {
    while(!(UART2->S1 & UART_S1_TDRE_MASK)); // wait until transmit data register is empty
    UART2->D = data; // put into D register
    // in transmit mode , waiting for data to be set
}

uint8_t UART2_Receive_Poll(void) {
    while(!(UART2->S1 & UART_S1_RDRF_MASK)); // wait until receive data register is full
    return (UART2->D);
}

void ledControl(int colour, led_status_t led_status) {
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
	} 
}

int main(void) {
	uint8_t rx_data = 0x01; //placeholder value just to initialize rx_data
	SystemCoreClockUpdate();
	initLED();
	initUART2(BAUD_RATE);
	offAllLed();

	while (1) {
		offAllLed(); // With this here it will just blink for 0x80000 
		rx_data = UART2_Receive_Poll();
		if (rx_data == 0x30) { // OFF Red LED
			ledControl(RED_LED, led_off);
			delay(0x80000);
		} else if (rx_data == 0x31) { // ON Red LED
			ledControl(RED_LED, led_on);
			delay(0x80000);
		} else if (rx_data == 0x32) { // OFF Green LED
			ledControl(GREEN_LED, led_off);
			delay(0x80000);
		} else if (rx_data == 0x33) { // ON Green LED
			ledControl(GREEN_LED, led_on);
			delay(0x80000);
		} else {
		// 	if (rx_IRQ_data == 0x30) { // OFF Red LED
		// 		ledControl(RED_LED, led_off);
		// 		delay(0x80000);
		// 	} else if (rx_IRQ_data == 0x31) { // ON Red LED
		// 		ledControl(RED_LED, led_on);
		// 		delay(0x80000);
		// 	} else if (rx_IRQ_data == 0x32) { // OFF Green LED
		// 		ledControl(GREEN_LED, led_on);
		// 		delay(0x80000);
		// 	} else if (rx_IRQ_data == 0x33) { // ON Green LED
		// 		ledControl(GREEN_LED, led_on);
		// 		delay(0x80000);
		// 	} else {
				ledControl(BLUE_LED, led_on);
				delay(0x80000);
			}
		// }

	}
}
