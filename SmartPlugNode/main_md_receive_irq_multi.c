#define NUM_SAMPLES 20
#define F_CPU   32000000UL     //!< System clock is 32 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "serialF0.h"
#include "clock.h"
#include "nrf24spiXM2.h"
#include "nrf24L01.h"

char  Rx2_packet[NRF_MAX_PAYLOAD_SIZE+1];

char R;
volatile int flag = 0;

uint8_t pipe2[5] = {0x47, 0x52, 0x44, 0x32, 0x32}; // GRD22 

uint8_t a;

double BerekenRMS(volatile int16_t *samples, uint8_t numSamples) {
	
	uint8_t i;
	double som = 0, gemiddelde, somKwadraten = 0, gemiddeldeKwadraten;
	for(i = 0; i < numSamples; i++)
	{
		som += samples[i];
	}
	gemiddelde = som / numSamples;
	for(i = 0; i < numSamples; i++)
	{
		somKwadraten += (samples[i] - gemiddelde) * (samples[i] - gemiddelde);
	}
	gemiddeldeKwadraten = somKwadraten / numSamples;
	return sqrt(gemiddeldeKwadraten);
}

static int16_t sample[NUM_SAMPLES];
static uint8_t i;

ISR(PORTF_INT0_vect)
{
	uint8_t  packet_length;
	uint8_t tx_ds, max_rt, rx_dr;
	nrfWhatHappened(&tx_ds, &max_rt, &rx_dr);
	
	if (rx_dr) {
		packet_length =  nrfGetDynamicPayloadSize();
		nrfRead( Rx2_packet, packet_length );
		Rx2_packet[packet_length] = '\0';
	}
	
	a = atoi(Rx2_packet);
	printf("%d\n" , a);
}

ISR(ADCA_CH0_vect)
{
	if(i < NUM_SAMPLES) {
		sample[i] = ADCA.CH0.RES;
		i++;
	}
}

void init_adc(void){
	PORTA.DIRCLR     = PIN2_bm|PIN3_bm;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc | ADC_CH_MUXNEG_PIN3_gc;
	ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_DIFF_gc;
	ADCA.CH0.INTCTRL = ADC_CH_INTLVL_LO_gc;
	ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;
	ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
	ADCA.PRESCALER   = ADC_PRESCALER_DIV16_gc;
	ADCA.CTRLA       = ADC_ENABLE_bm;
	ADCA.EVCTRL      = ADC_SWEEP_0_gc |         // sweep CH0
	ADC_EVSEL_0123_gc |      // select event CH0,1,2,3
	ADC_EVACT_CH0_gc;        // event triggers ADC CH0
	EVSYS.CH0MUX     = EVSYS_CHMUX_TCE0_OVF_gc; // event overflow timer E0
}

void init_timer(void)
{
	TCE0.PER      = 499;     // Tper =  64 * (499+1) / 32M = 0,001
	TCE0.CTRLA    = TC_CLKSEL_DIV64_gc;          // Prescaling 64
	TCE0.CTRLB    = TC_WGMODE_NORMAL_gc;        // Normal mode
	TCE0.INTCTRLA = TC_OVFINTLVL_OFF_gc;        // Interrupt overflow off
}

void Init_Nrf(void)
{
	nrfspiInit();
	nrfBegin();

	nrfSetRetries(NRF_SETUP_ARD_1500US_gc, NRF_SETUP_ARC_15RETRANSMIT_gc);
	nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);
	nrfSetDataRate(NRF_RF_SETUP_RF_DR_250K_gc);
	nrfSetCRCLength(NRF_CONFIG_CRC_16_gc);
	nrfSetChannel(125);
	nrfSetAutoAck(1);
	nrfEnableDynamicPayloads();

	nrfClearInterruptBits();
	nrfFlushRx();
	nrfFlushTx();

	PORTF.INT0MASK |= PIN6_bm;
	PORTF.PIN6CTRL  = PORT_ISC_FALLING_gc;
	PORTF.INTCTRL   = (PORTF.INTCTRL & ~PORT_INT0LVL_gm) |
	PORT_INT0LVL_LO_gc ;

	PORTF.OUTSET = PIN0_bm;
	nrfOpenReadingPipe(0,pipe2);
	nrfStartListening();

}



int main(void)
{
	Config32MHzClock_Ext16M();
	Init_Nrf();
	init_stream(F_CPU);
	init_adc();
	init_timer();                               // initialize timer
	init_clock();
	
	PORTD.DIRSET = PIN1_bm;
	double stroom;
	
	PMIC.CTRL |= PMIC_LOLVLEN_bm;           // Low level interrupt
	sei();

	clear_screen();
	
	while (1) {
		while (a == 1) {
				if (i == NUM_SAMPLES)
				{
					double rmsWaarde = (double) BerekenRMS(sample, NUM_SAMPLES);
					stroom = rmsWaarde*0.0085;
					printf("rms =%f stroom =%f A\n", rmsWaarde, stroom);
					i = 0;
				}
				if (stroom <= 0.01)
				{
					PORTD.OUTSET = PIN1_bm;
				}
				if (stroom >= 0.01)
				{
					PORTD.OUTCLR = PIN1_bm;
				}
		}
	}
}