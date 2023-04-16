
#define F_CPU 32000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "nrf24spiXM2.h"
#include "nrf24L01.h"
#include "clock.h"
#include "serialF0.h"
#include <ctype.h>

#define MAX_VALUE   2047 // FOR ADC XMEGA256A3U
#define VCC         3.30 // XMEGA256A3U Voltage
#define VREF        (((double) VCC) / 1.6) // formule voor het omzetten van rauwe waardes naar volt.

char  Rx_packet[NRF_MAX_PAYLOAD_SIZE+1];
char  command[NRF_MAX_PAYLOAD_SIZE+1];

uint8_t  pipe[5] = {0x47, 0x52, 0x44, 0x32, 0x33}; // GRD23
	
char afgedrukt[50];
int16_t rauwe_waarde;
double   vinp;

ISR(TCE0_OVF_vect)
{
	rauwe_waarde = lees_adc();
	vinp = (double) rauwe_waarde * VREF / (MAX_VALUE + 1);
	
	printf("spanning: %5.3lf V \n", vinp);
	
	
	sprintf(afgedrukt, "%lf", vinp);
	nrfWrite( (uint8_t *) afgedrukt, strlen(afgedrukt) );
}

void nrfInit(void)
{
  nrfspiInit();
  nrfBegin();

  nrfSetRetries(NRF_SETUP_ARD_1000US_gc, NRF_SETUP_ARC_8RETRANSMIT_gc);
  nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);
  nrfSetDataRate(NRF_RF_SETUP_RF_DR_250K_gc);
  nrfSetCRCLength(NRF_CONFIG_CRC_16_gc);
  nrfSetChannel(125);
  nrfSetAutoAck(1);
  nrfEnableDynamicPayloads();

  nrfClearInterruptBits();
  nrfFlushRx();
  nrfFlushTx();

  
  // Opening pipes
  nrfOpenWritingPipe(pipe);
  nrfOpenReadingPipe(0,pipe);
 
}

void init_adc(void)
{
	PORTA.DIRCLR     = PIN2_bm;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc |ADC_CH_MUXNEG_GND_MODE3_gc;
	ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_DIFF_gc;
	ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;
	ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc |ADC_CONMODE_bm;
	ADCA.PRESCALER   = ADC_PRESCALER_DIV256_gc;
	ADCA.CTRLA       = ADC_ENABLE_bm;
}

int16_t lees_adc(void)
{
	int16_t res;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	while ( !(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm) ) ;
	ADCA.CH0.INTFLAGS |= ADC_CH_CHIF_bm;

	return ADCA.CH0.RES;
}

void Init_Timer(void)
{
	TCD0.CTRLB     = TC0_CCCEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCD0.CTRLA     = TC_CLKSEL_DIV4_gc;
	TCD0.PER       = 9999;
	TCE0.CTRLB     = TC_WGMODE_NORMAL_gc;  // Normal mode
	TCE0.CTRLA     = TC_CLKSEL_DIV1024_gc; // prescaling 21024
	TCE0.INTCTRLA  = TC_OVFINTLVL_LO_gc;   // enable overflow interrupt low level
	TCE0.PER       = 39062;                // t = N*(PER+1)/FCPU = 1024*39063/32000000 = 1.250016 sec
}

int main(void)
{	
	PORTD.DIRSET = PIN4_bm; // LED up
	PORTD.DIRSET = PIN5_bm; // top switch
	PORTD.DIRSET = PIN6_bm; // LED down
	PORTD.DIRSET = PIN7_bm; // bottom switch
	
	Config32MHzClock_Ext16M();

	nrfInit();
	init_adc();
	Init_Timer();
	init_stream(F_CPU);
	sei();

	clear_screen();
	
	
	

	while (1) {
		
		if (vinp >=2 && ! (PORTD.IN & PIN5_bm)) {
			PORTD.OUTSET = PIN4_bm;
			
		}else if (vinp <=0.5 && ! (PORTD.IN & PIN7_bm)) {
			PORTD.OUTSET = PIN6_bm;
			
		}else{
			PORTD.OUTCLR = PIN4_bm;
			PORTD.OUTCLR = PIN6_bm;
		}
	}
}

