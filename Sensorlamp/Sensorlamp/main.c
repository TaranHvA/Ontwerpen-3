// Bestandsnaam     : Sensorlamp.c
// Auteur           : Taran
// Datum            : 04/03/2023
// Doel programma   : Light and motion control by sending data by using NRF communication.

#define F_CPU  2000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "SerialF0.h"
#include "nrf24spiXM2.h"
#include "nrf24L01.h"

#define MAX_VALUE   2047 // MAX ADC Value ATXMEGA256A3U
#define VCC         3.30 // XMEGA256A3U VCC PIN Voltage
#define VREF        (((double) VCC) / 1.6) // This Is The Calculation To Make The ADC Value A Voltage.

char  Rx_packet[NRF_MAX_PAYLOAD_SIZE+1];


char R;
volatile int flag = 0;

char  Rx2_packet[NRF_MAX_PAYLOAD_SIZE+1];
char  command[NRF_MAX_PAYLOAD_SIZE+1];

uint8_t Sen_Int  = 0;		// Sensor interrupt, Counter to Interrupts program every 20 minutes to see if people are still in the room
uint8_t Sen_Time = 0;		// Sensor time, Counter that oversees the 30 second time period of motion before it's turning the program off
uint8_t Sen_Prog = 0;		// Sensor program state, if the value is turned into an one the program work's when the state is changed to a zero the program is turned off
uint8_t Dim_Time = 15;		// Timer to readjust light level

uint8_t pipe[5] = {0x47, 0x52, 0x44, 0x32, 0x33}; // GRD23
uint8_t pipe2[5] = {0x47, 0x52, 0x44, 0x32, 0x32}; // GRD22

char Print[50];
int16_t Raw_Value;
double   Vinp;

void init_adc(void)
{
	PORTA.DIRCLR     = PIN2_bm;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc |
	ADC_CH_MUXNEG_GND_MODE3_gc;
	ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_DIFF_gc;
	ADCA.REFCTRL     = ADC_REFSEL_INTVCC_gc;
	ADCA.CTRLB       = ADC_RESOLUTION_12BIT_gc |
	ADC_CONMODE_bm;
	ADCA.PRESCALER   = ADC_PRESCALER_DIV256_gc;
	ADCA.CTRLA       = ADC_ENABLE_bm;
}

int16_t read_adc(void)
{
	int16_t res;
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	while ( !(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm) ) ;
	ADCA.CH0.INTFLAGS |= ADC_CH_CHIF_bm;

	return ADCA.CH0.RES;
}

ISR(TCE0_OVF_vect)
{
	if ( (PORTD.IN & PIN3_bm) && (Sen_Int == 0)){
		Sen_Prog = 1;
		Sen_Int = 1;
		printf("Turn on Light\n");
		sprintf(Print, "%d", 1);
		nrfWrite( (uint8_t *) Print, strlen(Print) );
		nrfStartListening();								 // Starts To Listen To The NRF Read And Write Channels
	}
	
	if ( (Sen_Int != 0) && (Sen_Int != 12) ){
		++Sen_Int;
		printf("Counting till Sensor Time==12\n");
	}
	
	if (Sen_Int >= 12){
		++Sen_Time;
		printf("Scanning current situation\n");
		
		if (PORTD.IN & PIN3_bm){
			Sen_Time = 0;
			Sen_Int = 1;
			printf("Light Stays on\n");
		}
		
		if ( (Sen_Time >= 6) && ( ! (PORTD.IN & PIN3_bm)) ){
			Sen_Prog = 0;
			Sen_Time = 0;
			Sen_Int = 0;
			printf("Light's turned off\n");
			nrfStopListening();									 // stops with Listenong To The NRF Read And Write Channels
			sprintf(Print, "%d", 0);
			nrfWrite( (uint8_t *) Print, strlen(Print) );
			nrfStartListening();								 // Starts To Listen To The NRF Read And Write Channels
		}
	}
	if(Sen_Prog == 1) {
		printf("Voltage: %5.3lf V \n", Vinp);
	}
	++Dim_Time;
}

ISR(PORTF_INT0_vect)
{
	uint8_t  packet_length;
	uint8_t tx_ds, max_rt, rx_dr;
	nrfWhatHappened(&tx_ds, &max_rt, &rx_dr);
	
	if (rx_dr) {
		packet_length =  nrfGetDynamicPayloadSize();
		nrfRead( Rx_packet, packet_length );
		Rx_packet[packet_length] = '\0';
	}
	
	flag = 1;
}

void init_timer(void)
{
	TCE1.PER      = 31249;     // Tper =  8 * (31249 +1) / 2M = 0.125 s
	TCE1.CTRLA    = TC_CLKSEL_DIV8_gc;          // Prescaling 8
	TCE1.CTRLB    = TC_WGMODE_NORMAL_gc;        // Normal mode
	TCE1.INTCTRLA = TC_OVFINTLVL_OFF_gc;        // Interrupt overflow off
}

void Init_Counter_Timer(void)
{
	TCD0.CTRLB     = TC0_CCCEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCD0.CTRLA     = TC_CLKSEL_DIV4_gc;
	TCD0.PER       = 9999;
	TCE0.CTRLB     = TC_WGMODE_NORMAL_gc;  // Normal mode
	TCE0.CTRLA     = TC_CLKSEL_DIV64_gc;   // prescaling 64
	TCE0.INTCTRLA  = TC_OVFINTLVL_LO_gc;   // enable overflow interrupt low level
	TCE0.PER       = 31249;                // t = N*(PER+1)/FCPU = 64*31250/2000000 = 1 sec
}

void init_nrf(void)
{
	nrfspiInit();                                        // Starts SPI
	nrfBegin();                                          // Starts radio module

	nrfSetRetries(NRF_SETUP_ARD_1500US_gc,               // Delay between connection retries: 1500 us
	NRF_SETUP_ARC_15RETRANSMIT_gc);						 // Number of connection retries: 15 retries
	nrfSetPALevel(NRF_RF_SETUP_PWR_6DBM_gc);             // Power Level: -6 dBm
	nrfSetDataRate(NRF_RF_SETUP_RF_DR_250K_gc);          // NRF Data Send Speed 250 Kbps
	nrfSetCRCLength(NRF_CONFIG_CRC_16_gc);               // CRC Length
	nrfSetChannel(125);                                  // Communication Channel: 125
	nrfSetAutoAck(1);                                    // Auto Acknowledge 1 == on
	nrfEnableDynamicPayloads();                          // Enable Dynamic Payloads

	nrfClearInterruptBits();                             // Clears interrupt bits
	nrfFlushRx();                                        // Flush Receive
	nrfFlushTx();                                        // Flush Transmission

	// Interrupt section for NRF
	PORTF.INT0MASK |= PIN6_bm;
	PORTF.PIN6CTRL  = PORT_ISC_FALLING_gc;
	PORTF.INTCTRL   = (PORTF.INTCTRL & ~PORT_INT0LVL_gm) |
	PORT_INT0LVL_LO_gc ;
	
	
	nrfOpenReadingPipe(0, pipe);                         // Pipe for Receiving
	nrfOpenWritingPipe(pipe2);                           // Pipe for Sending
}

int main(void)
{
	double  Rx_Value;
	uint8_t Light_LVL;
	uint8_t  Out_LVL;
	uint8_t  Ins_LVL;
	uint8_t Ins_Per;
	
	PORTD.DIRSET   = PIN2_bm;			   // input pin ADC
	PORTD.DIRCLR   = PIN3_bm;              // input pin PIR Motion Sensor
	
	init_adc();								// initialize adc function
	init_timer();                           // initialize timer function
	Init_Counter_Timer();                   // initialize timer of the sensor counter function
	init_nrf();								// initialize nrf function
	init_stream(F_CPU);

	PMIC.CTRL     |= PMIC_LOLVLEN_bm;      // set low level interrupts
	sei();                                 // enable interrupts
	
	clear_screen();
	
	while(1){
		
		while (Sen_Prog == 1) {
			
			if (0 > read_adc()){
				Raw_Value = 0;
			}
			
			if (read_adc() >= 0){
				Raw_Value = read_adc();
			}
			
			Vinp = (double) Raw_Value * VREF / (MAX_VALUE + 1);
			
			if (flag == 1) {
				Rx_Value = atof(Rx_packet);
				printf("%5.3lf Remote Value\n" , Rx_Value);
				flag = 0;
			}
			
			if(Dim_Time >= 15){				// Changes Light Value every 15 sec
				if (2.061 <= Rx_Value){
					TCD0.CCC = 0;
				}
				
				if ((2.061 >= Rx_Value) && (0.01 <= Rx_Value)){
					Out_LVL=100-((100/2.061)*Rx_Value);
					
					printf("%d Out Level\n" , Out_LVL);
					printf("%d Ins Level\n" , Ins_LVL);
					if(Vinp>0.240){
						Ins_LVL=100;
					}
					
					if(0.240>=Vinp){
						Ins_LVL=(100/0.240)*Vinp;
					}
					
					Ins_Per=Out_LVL+Ins_LVL;
					
					if (Out_LVL>Ins_LVL && Ins_Per>=100 ){
						Light_LVL=99.99*(Ins_LVL-(Ins_Per-100));
						TCD0.CCC = Light_LVL;
						printf("%d Light Level\n" , Light_LVL);
					}
					
					if (Out_LVL>Ins_LVL && Ins_Per<=100 ){
						Light_LVL=99.99*(Ins_LVL+(100+Ins_Per));
						TCD0.CCC = Light_LVL;
						printf("%d Light Level\n" , Light_LVL);
					}
					
					if (Out_LVL<Ins_LVL && Ins_Per>100 ){
						Light_LVL=99.99*(Ins_LVL-(Ins_Per-100));
						TCD0.CCC = Light_LVL;
						printf("%d Light Level\n" , Light_LVL);
					}
					
					if (Out_LVL<Ins_LVL && Ins_Per<100 ){
						Light_LVL=99.99*(Ins_LVL+(100+Ins_Per));
						TCD0.CCC = Light_LVL;
						printf("%d Light Level\n" , Light_LVL);
					}
				}
				
				if (0.02 >= Rx_Value){
					TCD0.CCC = 9999;
				}
				Dim_Time = 0;
			}
		}
		if (Sen_Prog == 0){
			TCD0.CCC = 0;
		}
	}
}