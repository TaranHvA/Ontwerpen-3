#define F_CPU 32000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf24spiXM2.h"
#include "nrf24L01.h"
#include "clock.h"
#include "serialF0.h"
#include <ctype.h>

char  Rx2_packet[NRF_MAX_PAYLOAD_SIZE+1];

char R;
volatile int flag = 0;

uint8_t  pipe[5] = {0x47, 0x52, 0x44, 0x32, 0x33}; // GRD23

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
	
	flag = 1;
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
	nrfOpenReadingPipe(0,pipe);
	nrfStartListening();

}



int main(void)
{
	int a;
	
	Config32MHzClock_Ext16M();
	Init_Nrf();
	init_stream(F_CPU);

	PMIC.CTRL |= PMIC_LOLVLEN_bm;           // Low level interrupt
	PORTC.DIRSET   = PIN0_bm;
	sei();

	clear_screen();
	
	while (1) {
		
		if (flag == 1) {
			a = atoi(Rx2_packet);
			printf("%d\n" , a);
			flag = 0;
		}
		
		if (a == 1) {
			PORTC.OUTSET = PIN0_bm;
		}else{
			PORTC.OUTCLR = PIN0_bm;
		}
		}
}







