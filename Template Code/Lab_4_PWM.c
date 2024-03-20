#include "MKL25Z4.h"

#define PTB0_Pin 0
#define PTB1_Pin 1
#define MASK(x) (1 << (x))

unsigned int frequency = 262;
unsigned int dutyCycle = 50; // percentage

void initPWM(void) {

    //Enable Clock Gating - Give power to the clock
    // SIM - System Integration Module
    // SCGC - System Clock Gating Control
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Configure Mode 3 for the particular pin
    // Based on page 163 of the manual,
    // ALT3 on PTB0 is TPM1_CH0 (the PWM mode)
    PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3); // ALT3

    // ALT3 on PTB1 is TPM1_CH1
    PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3); // ALT3

    // Enable Clock Gating for Timer1
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    // Select Clock for TPM Module
    // Referencing page 196 to get MCGFLLCLK or MCGPLLCLK/2
    // Multipurpose Clock Generator - Frequency / Phase Locked Loop - Clock
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    // Set Modulo (when clock resets)
    // clk_frequency = 48000000 / 128 (prescalar) = 375000 
    // clk_frequency / frequency (desired) = modulo_required
    // i.e. 375000 / 50 Hz = 7500
    TPM1->MOD = 375000 / frequency;

    /* Edge-Aligned PWM */
    // Update SnC register: CMOD = 01, Prescaler PS = 111 (128)
    // Referencing page 552 for Status and Control (SnC) register
    // Clock Mode Selection (CMOD) 01 for LPTPM counter++ on every clock
    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
    TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

    // Enable Edge-Aligned PWM (High True Pulses) (Clean output on match, set output on reload)
    // On page 555, means MS0B = 1, ELS0B = 1
    TPM1_C0SC &= ~( (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C0SC |= ( TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1) );
    TPM1_C1SC &= ~( (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C1SC |= ( TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1) );
}

int main(void)
{
	SystemCoreClockUpdate();
	initPWM();

    char i = 0;
    // In init PWM the period is set at 7500
    // TPM1_C0V = 0xEA6; // 3750 -> 7500 / 2 -> 50% Duty Cycle
    // TPM1_C1V = 0x753; // 1875 -> 7500 / 4 -> 25% Duty Cycle on channel 2

    TPM1_C0V = 375000 / frequency / (100 / dutyCycle);
    TPM1_C1V = 375000 / frequency / (100 / dutyCycle);

    // Now generating hardware interrupts and waveform at the port
}
