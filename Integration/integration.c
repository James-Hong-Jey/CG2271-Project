/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
#include "MKL25Z4.h" 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h" 
 

 
 
 
 
 
// ###################### START OF MOTOR AND URAT #########################


// has to be declared early here as motor uses it
osSemaphoreId_t ledStationarySem;
osSemaphoreId_t ledMovingSem; 
osSemaphoreId_t audioRaceOngoingSem;
osSemaphoreId_t audioRaceFinishedSem;

#define RED_LED 18 //PortB Pin 18
#define GREEN_LED 19 //PortB Pin 19
#define BLUE_LED 1 //PortD Pin 1
#define MASK(x) (1 << (x))

#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define Q_SIZE 32

#define FORWARD 1
#define BACKWARDS 0
#define LEFT 2
#define RIGHT 3

#define LEFT_FORWARD 0 // PTB0 TPM1_CH0 (M1 & M3)
#define LEFT_BACK 1 // PTB1 TPM1_CH1 (M1 & M3)
#define RIGHT_FORWARD 2 // PTB2 TPM2_CH0 (M2 & M4)
#define RIGHT_BACK 3 //PTB3 TPM2_CH1 (M2 & M4)
#define DUTY_CYCLE 0x1D4C // 7500 (50hz)
 

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
volatile uint8_t rx_IRQ_data = 0x00;
// volatile uint32_t rx_IRQ_data = 0;

void Q_Init(Q_T *q) {
  unsigned int i;
  for(i=0; i < Q_SIZE; i++) q->DATA[i] = 0; // Initialise to 0
  q->HEAD = 0;
  q->TAIL = 0;
  q->SIZE = 0;
}

int Q_Empty(Q_T *q) {
  return q->SIZE == 0;
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
    NVIC_SetPriority(UART2_IRQn, 128);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // // Enable TX and RX Interrupts
    UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;

    // Initialise both tx_q and rx_q
    Q_Init(&tx_q);
    Q_Init(&rx_q);

    UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); //Enable TX and RX
}

// /* UART2 Transmit Poll */
// void UART2_Transmit_Poll(uint8_t data) {
//     while(!(UART2->S1 & UART_S1_TDRE_MASK)); // wait until transmit data register is empty
//     UART2->D = data; // put into D register
//     // in transmit mode , waiting for data to be set
// }

// /* UART2 Receive Poll */
// uint8_t UART2_Receive_Poll(void) {
//     while(!(UART2->S1 & UART_S1_RDRF_MASK)); // wait until receive data register is full
//     return (UART2->D);
// }

void initMotorPWM(void) {
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

 

void stopMotor() {
    TPM1->MOD = 0;
    // Captured LPTPM counter value of the input modes
    TPM1_C0V = 0; // Stop Left motors Fw
    TPM1_C1V = 0; // Stop Left motors Back

    TPM2->MOD = 0;
    TPM2_C0V = 0; // Stop Right motors fw
    TPM2_C1V = 0; // Stop Right motors back
}

void rotateRight() {
    TPM1->MOD = 7500;
    TPM1_C0V = 3750; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C0V = 3750; // Right motors forward
}

void rotateLeft() {
    TPM1->MOD = 7500;
    TPM1_C1V = 3750; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C1V = 3750; // Right motors reverse
}

void leftTurn() {
    TPM1->MOD = 7500;
    TPM1_C0V = 0x1964; // Left motors slower

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE; // Right motors faster
}

void rightTurn() {
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE; // Left motors faster

    TPM2->MOD = 7500;
    TPM2_C0V = 1000; // Right motors slower
}

void forwardMotor() {
    TPM1->MOD = 7500;
    TPM1_C1V = DUTY_CYCLE; // Left motors reverse

    TPM2->MOD = 7500;
    TPM2_C0V = DUTY_CYCLE; // Right motors forward
}

void reverseMotor() {
    TPM1->MOD = 7500;
    TPM1_C0V = DUTY_CYCLE; // Left motors forward

    TPM2->MOD = 7500;
    TPM2_C1V = DUTY_CYCLE; // Right motors reverse
}


static void delay(volatile uint32_t nof) {
    while(nof != 0) {
        __asm("NOP");
        nof--;
    }
}

 // ###################### END OF MOTOR AND URAT  ########################
 
 

// ####################### START OF AUDIO TASK ###########################

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  261
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS 455
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 830
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST 0

#define MOD(x) (37500/x) // Audio Lab uses 375000 with an extra zero

void delay_ms(uint32_t delay)
{
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Count down from this value
    SysTick->VAL = 0; // Clear current value register
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick

    for(uint32_t i = 0; i < delay; i++) {
        // Wait until the COUNTFLAG is set
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }

    SysTick->CTRL = 0; // Disable SysTick
}

#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTE30_Pin 30 // TPM0_CH3
			

void initAudioPWM(void) {
	
	//Enables the clock gate for Port E
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	
	PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK; //Clear bit 10 to 8
	PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3); //We want to enable the Timer/PWM module so we use 3 to select Alternative 3 to enable GPIO, datasheet pg 163
	

	
	//Enables the clock gate for TPM0 module, datasheet page 208
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; 
	
	
	// POTENTIAL BUG: already done in initPWM for motor 
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; //Clear bit 25 to 24, datasheet page 195
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // Set 01 to bit 25 to 24, MCGFLLCLK clock or MCGPLLCLK/2 is used as clock source for TPM counter clock
	
	TPM0->MOD = 7500; //Set Modulo value = 4 800 000 / 128 = 375 000 / 7500 = 50 Hz
	
	//See datasheet page 553, LPTPM means low power timer/pulse width modulator module
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK)); //Clears bit 4 to 0, 2 to 0 for PS, 4 to 3 for CMOD
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7)); // LPTPM counter clock mode is selected as 01 (LPTPM counter increments on every LPTPM counter clock), Prescale Factor Selection of 7 is 0b111 which is divide by 128 
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK); //Clears CPWMS (Centre-aligned PWM select). Aka mode = 0 which means LPTPM counter operates in up counting mode.
	
	//See datasheet page 555
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); //Clears bit 5 to 2, disabling channel mode 
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); // CPWMS = 0, ELSnB:ELSnA = 0b10, MSnB:MSnA = 0b10, this means Mode = Edge-aligned PWM, Config = High-true pulses (clear Output on match, setOutput on reload)
}


int once_upon_a_time_notes[] = 
{
	// Bar 1
	NOTE_E4, NOTE_G4, NOTE_C4,

	// Bar 2 
	NOTE_E4, NOTE_G4, NOTE_C4,
	
	// Bar 3
	NOTE_E4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_C4,
	
	// Bar 4
	NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_G4,
	
	
	// Bar 5
	NOTE_E4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C4,
	
	// Bar 6
	NOTE_E4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_C4,
	
	// Bar 7
	
	NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_E4,
	
	
	// Last
	REST
};

int once_upon_a_time_notes_one_octave_up[] = 
{
	// Bar 1
	NOTE_E5, NOTE_G5, NOTE_C5,

	// Bar 2 
	NOTE_E5, NOTE_G5, NOTE_C5,
	
	// Bar 3
	NOTE_E5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
	
	// Bar 4
	NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6, NOTE_G5,
	
	
	// Bar 5
	NOTE_E5, NOTE_G5, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C5,
	
	// Bar 6
	NOTE_E5, NOTE_F5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_C5,
	
	// Bar 7
	
	NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6, NOTE_E5,
	
	
	// Last
	REST
};

/*
*	1  = Whole Note
* 2  = Half Note
* 4  = Quarter Note
* 8  = Eighth Note
* 16 = Sixteenth Note
*	32 = Thirty_second Note
* 64 = Sixty-Fourth Note
* So actually the greater the number, the less duration the note plays for 
* unless I am miss understanding this...
*/



int once_upon_a_time_notes_duration[] = 
{
	// Bar 1
	4,4,2,
	
	// Bar 2
	4,4,2,
	
	// Bar 3
	8,8,4,4,4,4,
	
	// Bar 4
	8,8,4,4,4,
	
	// Bar 5
	8,8,4,4,4,4,
	
	// Bar 6
	8,8,4,4,4,4,
	
	// Bar 7
	8,8,4,4,4,
	
	
	
	// LAST
	4
	
};




int river_flows_in_you_notes[] = 
{
	// Bar 1
	NOTE_A4,NOTE_E4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,

	// Bar 2 
	NOTE_A4,NOTE_E4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,
	
	// Bar 3
	NOTE_B4,NOTE_FS4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_B4,NOTE_FS4,NOTE_E4,
	
	
	// Bar 4
	NOTE_E4,NOTE_E4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,
	
	// Bar 5
	NOTE_A4,NOTE_E4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,
	
	// Bar 6
	NOTE_A4,NOTE_CS4,NOTE_B4,NOTE_A4,NOTE_GS4,NOTE_FS4,NOTE_GS4,NOTE_A4,
	
	// Bar 7
	NOTE_B4,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,
	
	// Bar 8
	NOTE_E4,NOTE_E4,NOTE_E4,NOTE_FS4,NOTE_GS4,NOTE_A4,NOTE_E4,NOTE_E4,
	
	// Last
	REST
};

/*
*	1  = Whole Note
* 2  = Half Note
* 4  = Quarter Note
* 8  = Eighth Note
* 16 = Sixteenth Note
*	32 = Thirty_second Note
* 64 = Sixty-Fourth Note
* So actually the greater the number, the less duration the note plays for 
* unless I am miss understanding this...
*/



int river_flows_in_you_duration[] = 
{
	// Bar 1
	4,8,8,8,8,8,8,8,
	
	// Bar 2
	4,8,8,8,8,8,8,8,
	
	// Bar 3
	4,8,8,8,8,8,8,8,
	
	// Bar 4
	4,8,8,8,8,8,8,8,
	
	// Bar 5
	4,8,8,8,8,8,8,8,
	
	// Bar 6
	4,8,8,8,8,16,16,8,
	
	// Bar 7
	4,8,8,8,8,8,8,8,
	
	// Bar 8
	4,8,8,8,8,8,8,8,
	
	// LAST
	4
	
};



void play_once_upon_a_time_one_octave_up()
{
	int notes_num = sizeof(once_upon_a_time_notes_one_octave_up)/ sizeof(once_upon_a_time_notes_one_octave_up[0]);
	int beats_per_min = 120;
	
	int one_beat = 60000 / beats_per_min; // 60000 bcs its 60000 ms = 60 seconds in one min
	// one beat is the duration for one quarter note.
	// To compute duration, we will do, (1/note_duration) * 4 * one_beat
	// This is because the way we defined the notes earlier. Whole note should be 4 beats. Half note 2 beats.
	// quarter_note should be 1 beat. eigtht_note is 1/2 beat.
	// I think this formula works.. Might be wrong
	
	for(int i = 0; i < notes_num; i++)
	{
		int curr_musical_note = once_upon_a_time_notes_one_octave_up[i];
		float curr_note_duration = (1/once_upon_a_time_notes_duration[i]) * 4 * one_beat; // See reasoning above
		
		int period = MOD(curr_musical_note);
		
		TPM0->MOD = period;
		TPM0_C3V = period / 6; 
		
		osDelay(curr_note_duration);
		//delay_ms(curr_note_duration);

	}
};

#define MOD_river(x) (75000/x)
void play_river_flows_in_you()
{
	int notes_num = sizeof(river_flows_in_you_notes)/ sizeof(river_flows_in_you_notes[0]);
	int beats_per_min = 120;
	
	int one_beat = 60000 / beats_per_min; // 60000 bcs its 60000 ms = 60 seconds in one min
	// one beat is the duration for one quarter note.
	// To compute duration, we will do, (1/note_duration) * 4 * one_beat
	// This is because the way we defined the notes earlier. Whole note should be 4 beats. Half note 2 beats.
	// quarter_note should be 1 beat. eigtht_note is 1/2 beat.
	// I think this formula works.. Might be wrong
	
	for(int i = 0; i < notes_num; i++)
	{
		int curr_musical_note = river_flows_in_you_notes[i];
		float curr_note_duration = (1/river_flows_in_you_duration[i]) * 4 * one_beat; // See reasoning above
		
		int period = MOD_river(curr_musical_note);
		
		TPM0->MOD = period;
		TPM0_C3V = period / 6; 
		
		osDelay(curr_note_duration);
		//delay_ms(curr_note_duration);

	}
};



// ####################### END OF AUDIO TASK ###############################





// ####################### START OF LED TASK ###########################

// The numbering here looks weird but follows a line on the left-side of the board
#define RED_LED_1 8 		// 	PortB Pin 8 
#define GREEN_LED_1 7 //	PortC Pin 7
#define GREEN_LED_2 0 //	PortC Pin 0
#define GREEN_LED_3 3 //	PortC Pin 3
#define GREEN_LED_4 4 //	PortC Pin 4
#define GREEN_LED_5 5 //	PortC Pin 5
#define GREEN_LED_6 6 //	PortC Pin 6
#define GREEN_LED_7 10 //	PortC Pin 10
#define GREEN_LED_8 11 //	PortC Pin 11
#define GREEN_LED_9 12 //	PortC Pin 12
#define GREEN_LED_10 13//	PortC Pin 13
#define MASK(x) (1 << (x))


void InitLEDGPIO(void) {
	
	// Green LED
  //Enable CLock to PORTC
  SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK); 
  
  //Configure MUX settings 
  PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1); //Assign output
  
  PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1); //Assign output 
  
  PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1); //Assign output 
  
  PORTC->PCR[GREEN_LED_9] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_9] |= PORT_PCR_MUX(1); //Assign output 
   
  PORTC->PCR[GREEN_LED_10] &= ~PORT_PCR_MUX_MASK; //clear 
  PORTC->PCR[GREEN_LED_10] |= PORT_PCR_MUX(1); //Assign output 
  
  //Set Data Direction Registers for PortB and PortD
  PTC->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | 
                MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
								
								
								
								
	// RED LED 
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
	PORTB->PCR[RED_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_1] |= PORT_PCR_MUX(1);
	
	PTB->PDDR |= MASK(RED_LED_1);
};




void led_toggler(int colour_current) { //, int colour_previou
  PTC->PCOR = (MASK(GREEN_LED_1)|MASK(GREEN_LED_2)|MASK(GREEN_LED_3)|MASK(GREEN_LED_4)|MASK(GREEN_LED_5)|MASK(GREEN_LED_6)|MASK(GREEN_LED_7)|MASK(GREEN_LED_8)|MASK(GREEN_LED_9)|MASK(GREEN_LED_10));
  //PTE->PCOR = MASK(colour_previous);
  PTC->PSOR |= MASK(colour_current);
  //delay(0xf0f0); // POTENTIAL BUG -> could cause a lot of lag time and stall processer
	osDelay(61680); // TODO: Fine-Tune this. I converted 0xf0f0 directly to decimal
}
void green_led_left_to_right(){
  
	/*
	while (1){ // TODO: The code can get struck here.
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
	*/
	
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



void green_led_remain() { //, int colour_previou
  PTC->PSOR = (MASK(GREEN_LED_1)|MASK(GREEN_LED_2)|MASK(GREEN_LED_3)|MASK(GREEN_LED_4)|MASK(GREEN_LED_5)|MASK(GREEN_LED_6)|MASK(GREEN_LED_7)|MASK(GREEN_LED_8)|MASK(GREEN_LED_9)|MASK(GREEN_LED_10));
  //delay();
}



void toggleRedLED500ms (){    // Red LEDs go on for 500ms and off for 500ms
    
    PTB->PCOR |= MASK(RED_LED_1);
    osDelay(500);
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(500);
    
}
	
void toggleREDLED250ms (){    // Red LEDs go on for 250ms and off for 250ms
    
    PTB->PCOR |= MASK(RED_LED_1);
		osDelay(100); // TODO: CHANGE BACK
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(100);
    
}

/*
void toggleRedLED500ms (){    // Red LEDs go on for 500ms and off for 500ms
    while(1){
    PTB->PCOR |= MASK(RED_LED_1);
    osDelay(500);
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(500);
    }
}
	
void toggleREDLED250ms (){    // Red LEDs go on for 250ms and off for 250ms
    while(1){
    PTB->PCOR |= MASK(RED_LED_1);
    osDelay(250);
    PTB->PSOR |= MASK(RED_LED_1);
    osDelay(250);
    }
}
*/



// ####################### END OF LED TASK ###############################




// ##################### START OF UART IRQ ############################

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
// ####################### END OF UART IRQ #####################


// ####################### START OF THREADS ###############################

void motor_thread(void *argument) {
	for(;;) {
		//offAllLed();
    //stopMotor();
    // left and right are both 4 bit numbers from 0 (UP) to 15 (DOWN)
    // 7-8 is the idle position
    uint8_t leftStick = (rx_IRQ_data >> 4);
    uint8_t rightStick = rx_IRQ_data & 0x0F;

    // Direction (FORWARD == 1) 
    uint8_t leftDir = leftStick - 8 > 0 ? FORWARD : BACKWARDS;
    // Magnitude from 0 to 7
    uint8_t leftMag = leftStick - 8 > 0 ? leftStick - 8 : 8 - leftStick;

    // Direction (LEFT == 2) 
    uint8_t rightDir = rightStick - 8 > 0 ? LEFT : RIGHT;
    // Magnitude from 0 to 7
    uint8_t rightMag = rightStick - 8 > 0 ? rightStick - 8 : 8 - rightStick;

		if (rx_IRQ_data == 0x01) {
				forwardMotor(); // R1 full speed
		} else if (rx_IRQ_data == 0x02) { 
				rightTurn(); // L1 full speed
		} else if (rx_IRQ_data == 0x06) {
				forwardMotor(); // triangle
		} else if (rx_IRQ_data == 0x03) {
				reverseMotor(); // cross half speed
		} else if (rx_IRQ_data == 0x04) {
				rotateLeft(); // square
		} else if (rx_IRQ_data == 0x05) {
				rotateRight(); // circle
		} else {
				stopMotor();
		}
		
		// TODO: POTENTIAL BUG 
		// I expect that there will be an additional button that will play the unique sound
		// And this would need to be changed. 
		// If we assume that once we finish the challenge run, the robot will be stationary then this does not need to be changed
		// Moving
		if (rx_IRQ_data == 0x01 || rx_IRQ_data == 0x02 || rx_IRQ_data == 0x03 ||
				rx_IRQ_data == 0x04 || rx_IRQ_data == 0x05 || rx_IRQ_data == 0x06) {
			osSemaphoreRelease(ledMovingSem);
		}
		// Stationary
		else { 
			osSemaphoreRelease(ledStationarySem);
		}
		
		// TODO: Add a button that plays the alternative song for when the race ends
		if (rx_IRQ_data == 0x10) {
			osSemaphoreRelease(audioRaceFinishedSem);
		}
		else { 
			osSemaphoreRelease(audioRaceOngoingSem);
		}
		
		
		
		// FOR TESTING
    // if (rx_IRQ_data == 0x31) { // ON Red LED
    //    PTB->PCOR = MASK(RED_LED); // TEST
    // } else if(left > 8) { // LEFT DOWN
    //   PTB->PCOR = MASK(GREEN_LED); // JUST TO TEST
    //  } else if (right < 7) { // RIGHT UP
    //    PTD->PCOR = MASK(BLUE_LED); // JUST TO TEST
    // } else { 
    //   offAllLed();
    // }	
	}
	
}


// POTENTIAL BUG: will this run multiple times? I dun think so right 
void audio_race_ongoing_thread(void *argument) {
	for(;;)
	{
		osSemaphoreAcquire(audioRaceOngoingSem, osWaitForever);
		play_once_upon_a_time_one_octave_up();
		osSemaphoreRelease(audioRaceOngoingSem);
	}
}

void audio_race_finished_thread(void *argument) {
	for(;;)
	{
		osSemaphoreAcquire(audioRaceFinishedSem, osWaitForever);
		play_river_flows_in_you();
		osSemaphoreRelease(audioRaceFinishedSem);
	}

}





void led_stationary_thread(void *argument) 
{
	for(;;)
	{
		osSemaphoreAcquire(ledStationarySem, osWaitForever);
		// Function for Front 8-10 Green LED to be in ALL LIGHTED UP
		green_led_remain();
		
		// Function for Rear 8-10 Red LED to be Flashing at 250 ms ON ,250 ms OFF
		toggleREDLED250ms(); // delay is already built in
		
		
	}
};

void led_moving_thread(void *argument)
{
	
	for(;;)
	{
		osSemaphoreAcquire(ledMovingSem, osWaitForever);
		// Function for Front 8-10 Green LED to be in RUNNING MODE
		green_led_left_to_right();
		
		// Function for Rear 8-10 Red LED to be Flashing at 500 ms ON ,500 ms OFF
		toggleRedLED500ms(); // delay is already built in
		

	}
	
	
}


/*
void led_thread(void *argument) 
{
	for(;;)
	{
		
		if (rx_IRQ_data == 0x01 || rx_IRQ_data == 0x02 || rx_IRQ_data == 0x03 ||
				rx_IRQ_data == 0x04 || rx_IRQ_data == 0x05 || rx_IRQ_data == 0x06) {
			green_led_left_to_right();
		
			// Function for Rear 8-10 Red LED to be Flashing at 500 ms ON ,500 ms OFF
			toggleRedLED500ms(); // delay is already built in

					
			
		} else
		{
			// Function for Front 8-10 Green LED to be in ALL LIGHTED UP
			green_led_remain();
			
			// Function for Rear 8-10 Red LED to be Flashing at 250 ms ON ,250 ms OFF
			toggleREDLED250ms(); // delay is already built in
		}	
	}
};

*/



// ######################## END OF THREADS  ################################






/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void app_main (void *argument) {
 
  // ...
  for (;;) {
		;
	
	
	}
}


int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	
	
	// AUDIO 
	initAudioPWM();
	
	
  
	// MOTOR 
  // initLED();
  initUART2(BAUD_RATE);
  initMotorPWM();
  offAllLed();
  stopMotor();
	
	// LED
	InitLEDGPIO();
	
	

	
	
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	audioRaceOngoingSem = osSemaphoreNew(1,0,NULL);
	audioRaceFinishedSem = osSemaphoreNew(1,0,NULL);
	ledStationarySem = osSemaphoreNew(1,0,NULL);
	ledMovingSem = osSemaphoreNew(1,0,NULL);
	
	osThreadNew(motor_thread, NULL, NULL);
	osThreadNew(led_moving_thread, NULL,NULL);
	osThreadNew(led_stationary_thread, NULL, NULL);
	//osThreadNew(audio_race_ongoing_thread, NULL, NULL);
	//osThreadNew(audio_race_finished_thread, NULL, NULL);
	
	//osThreadNew(led_thread, NULL, NULL);
	
	
	
  //osThreadNew(app_main, NULL, NULL);    // Create application main thread
	
	
	
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
