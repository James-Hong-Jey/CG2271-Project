#include "MKL25Z4.h"

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22  // Page 162
#define UART_RX_PORT23 23   // Page 162
#define UART2_INT_PRIO 128

void initUART2(uint32_t baud_rate) {

    uint32_t divisor, bus_clock;

    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Page 162 - PTE22 Alt 4 = UART2 TX
    PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

    // Page 162 - PTE23 Alt 4 = UART2 RX
    PORTE->PCR[UART_TX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX_PORTE23] |= PORT_PCR_MUX(4);

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