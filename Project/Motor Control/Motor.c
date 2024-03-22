#include "MKL25Z4.h"

#define LEFT_FORWARD 0 // PTB0 TPM1_CH0 (M1 & M3)
#define LEFT_BACK 1 // PTB1 TPM1_CH1 (M1 & M3)
#define RIGHT_FORWARD 2 // PTB2 TPM2_CH0 (M2 & M4)
#define RIGHT_BACK 3 //PTB3 TPM2_CH1 (M2 & M4)
#define DUTY_CYCLE 0x1D4C // 7500 (50hz)

volatile int speed = 2; // Control turning speed to be slower

void initPWM(void) {
    // enable clock gating for PORTB
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // configure mode 3 for the PWM pin operation
    // For left motors
    PORTB->PCR[LEFT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[LEFT_FORWARD] |= PORT_PCR_MUX(3);
    PORTB->PCR[LEFT_BACK] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[LEFT_BACK] |= PORT_PCR_MUX(3);

    // For right motors
    PORTB->PCR[RIGHT_FORWARD] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[RIGHT_FORWARD] |= PORT_PCR_MUX(3);
    PORTB->PCR[RIGHT_BACK] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[RIGHT_BACK] |= PORT_PCR_MUX(3);

    // enable clock gating for Timer 1
    SIM->SCGC6 = (SIM_SCGC6_TPM1_MASK) | (SIM_SCGC6_TPM2_MASK);

    // select clock for TPM module
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK clock or MCGPLLCLK/2 clock

    // Set Mod value 48000000 / 120 = 375000 / 7500 = 50hz
    TPM1->MOD = 7500;
    TPM2->MOD = 7500;

    // Edged aligned PWM
    // upate snc register: CMOD = 01, PS = 111 (128)
    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7)); // LPTPM counter increments on every LPTPM counter clock
    TPM1->SC &= ~(TPM_SC_CPWMS_MASK); // up counting mode

    // Same config as TPM1
    TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
    TPM2->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7)); 
    TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

    // Enable PWM on TPM1 Channel 0 -> PTB0
    TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM1 Channel 1 -> PTB1
    TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM2 Channel 0 -> PTB3
    TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

    // Enable PWM on TPM2 Channel 1 -> PTB4
    TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

void stopMotor() {
    TPM1->MOD = 0;
    // Captured LPTPM counter value of the input modes
    TPM1_C0V = 0; // Stop Left motors Fw
    TPM1_C1V = 0; // Stop Left motors Back

    TPM2->MOD = 0;
    TPM2_C0V = 0; // Stop Right motors fw
    TPM2_C1V = 0; // Stop Right motors back
}

void forwardMotor() {
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE; // Right motors forward
}

void reverseMotor() {
    TPM1->MOD = 7500;
    TPM1_C1V = DUTY_CYCLE; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C1V = DUTY_CYCLE; // Right motors reverse
}

void leftTurn() {
    TPM1->MOD = 7500;
    TPM1_C1V = DUTY_CYCLE / speed; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE / speed; // Right motors forward
}

void rightTurn() {
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE / speed; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C1V = DUTY_CYCLE / speed; // Right motors reverse
}

int main (void) {
    initPWM();
    stopMotor();
    
    while(1) {
        forwardMotor();
        delay(0x80000);
        stopMotor();
        reverseMotor();
        delay(0x80000);
        stopMotor();
        rightTurn();
        delay(0x80000);
        leftTurn();
        stopMotor();
        delay(0x80000);
    }
}