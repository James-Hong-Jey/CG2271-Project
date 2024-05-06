#include "MKL25Z4.h"

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22  // Page 162
#define UART_RX_PORTE23 23   // Page 162
#define UART2_INT_PRIO 128

// Create a queue 
#define Q_SIZE 32

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
            // Queue Empty so disable interrupts (??)
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

void initUART2(uint32_t baud_rate) {

    uint32_t divisor, bus_clock;

    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Page 162 - PTE22 Alt 4 = UART2 TX
    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

    // Page 162 - PTE23 Alt 4 = UART2 RX
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

    // UART2->C2 - UART Control Register 2 (Page 753)
    // This turns them off 
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

    // Default system clock - 48MHz
    bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
    // 16 because clock is oversampled for noise control - unique to this controller
    divisor = bus_clock / (baud_rate * 16); 
    UART2->BDH = UART_BDH_SBR(divisor >> 8); // Baud High
    UART2->BDL = UART_BDL_SBR(divisor); // Baud Low

    // Page 751 - These are just to disable parity errors
    UART2->C1 = 0;  // Sets last bit PT (Parity) to 0 (no parity)
    UART2->S2 = 0;  // RAF Receiver Active Flag set to 0 - receiver waiting for start bit
    UART2->C3 = 0;  // Parity Error Interrupt set to 0 - disabled

    // Queue
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // Enable TX and RX Interrupts
    UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;

    // Initialise both tx_q and rx_q
    Q_Init(&tx_q);
    Q_Init(&rx_q);

    // Enables Transmit & Receive
    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
}

/* UART2 Transmit Poll */
void UART2_Transmit_Poll(uint8_t data) {
    // S1 = Status Register 1, TDRE = Transmit Data Register Empty
    while(!(UART2->S1 & UART_S1_TDRE_MASK)); // Poll until Data register is empty
    UART2->D = data; // put into D register
    // in transmit mode , waiting for data to be set
}

uint8_t UART2_Receive_Poll(void) {
    // S1 = Status Register 1, RDRF = Receive Data Register Full Flag
    while(!(UART2->S1 & UART_S1_RDRF_MASK)); // Poll until Full
    return (UART2->D); // Return data
}

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

int main(void) {

    uint8_t rx_data = 0x69;

    SystemCoreClockUpdate();
    initUART2(BAUD_RATE);

    while(1) {
        UART2_Transmit_Poll(0x69);
        delay(0x80000);
        rx_data++;
    }

}
