#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef cbi
#define cbi(register,bit)       register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)       register |= (_BV(bit))
#endif

#define STATE_WAITING 0
#define STATE_DATA_ADDRESS 1
#define STATE_DATA 2
#define STATE_BRIGHTNESS_ADDRESS 3
#define STATE_BRIGHTNESS 4
#define STATE_SKIP 5

// global brightness change
#define STG 0x99
// start of transmission
#define STX 0xAA
// individual brightness change
#define STI 0xBB

#define defaultBrightness 0;
#define LOADPIN 4
#define LOADPORT PORTB
#define CLOCKPIN 7
#define CLOCKPORT PORTB
#define DATPIN 5
#define DATPORT PORTB

unsigned char serialState = STATE_WAITING;
unsigned char receivedByte = 0;
unsigned char remappedByte = 0;
unsigned char myAddress = 0;

void shiftOutByte(unsigned int shiftByte);

int main(void) {
	// set up ports
	// DO (PB6) and USCK(PB7) pins are output
	// PB2 is pwm out
	// UART is input - PD0 and PD1
	// PD2 is direction control - output, default low
	// PD3-6 are address, thus input
	// PB4 is LOAD, and is thus output
	DDRB = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
	DDRD = (1 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (0 << 1) | (0 << 0);
	PORTB = 0x00;
	PORTD = 0x00;
	
	// set up USART for 9600 asynchronous 8n1
	UBRRH = 0;
	UBRRL = 47;
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UCSRC = (0 << USBS) | (3 << UCSZ0);
	
	// set up PWM
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00);
	OCR0A = defaultBrightness;
	
	serialState = STATE_WAITING;
	
	while(1) {	
		// check for received data
		while ( !(UCSRA & (1<<RXC)) );
		receivedByte = UDR;
		
		// refresh my address
		myAddress = PIND & 0x78;
		myAddress >>= 3;
		
		cbi(LOADPORT, LOADPIN);
	
		switch(serialState) {
			case STATE_WAITING:
				cbi(PORTB, 3);
				cbi(PORTB, 1);
				cbi(PORTB, 0);
				if(receivedByte == STG) {
					serialState = STATE_BRIGHTNESS;
				} else if(receivedByte == STX) {
					serialState = STATE_DATA_ADDRESS;
				} else if(receivedByte == STI) {
					serialState = STATE_BRIGHTNESS_ADDRESS;
				} else {
					serialState = STATE_WAITING;
				}
				break;
			case STATE_DATA_ADDRESS:
				// compare to address
				sbi(PORTB, 3);
				if(receivedByte == myAddress) {
					serialState = STATE_DATA;
				} else {
					serialState = STATE_SKIP;
				}
				break;
			case STATE_DATA:
				// process the received byte and write to SPI
				// first do the necessary remapping
				sbi(PORTB, 1);
				remappedByte = 0;
				if(receivedByte & 1) remappedByte |= (1 << 3);
				if(receivedByte & 2) remappedByte |= (1 << 4);
				if(receivedByte & 4) remappedByte |= (1 << 5);
				if(receivedByte & 8) remappedByte |= (1 << 7);
				if(receivedByte & 16) remappedByte |= (1 << 0);
				if(receivedByte & 32) remappedByte |= (1 << 1);
				if(receivedByte & 64) remappedByte |= (1 << 2);
				if(receivedByte & 128) remappedByte |= (1 << 6);
				// now send it
				shiftOutByte(remappedByte);
				serialState = STATE_WAITING;
				break;
			case STATE_BRIGHTNESS_ADDRESS:
				// compare to address
				sbi(PORTB, 3);
				if(receivedByte == myAddress) {
					serialState = STATE_BRIGHTNESS;
				} else {
					serialState = STATE_SKIP;
				}
				break;
			case STATE_BRIGHTNESS:
				sbi(PORTB, 0);
				// process the received byte and write to PWM
				OCR0A = receivedByte;
				serialState = STATE_WAITING;
				break;
			case STATE_SKIP:
			default:
				// skip a bit, brother
				serialState = STATE_WAITING;
				break;
		}
	}
}

void shiftOutByte(unsigned int shiftByte) {
	unsigned char cnt = 0;
	cbi(LOADPORT, LOADPIN);
	cbi(CLOCKPORT, CLOCKPIN);
	cbi(DATPORT, DATPIN);
	
	for(cnt=8; cnt; --cnt, shiftByte <<= 1) {
		if(shiftByte & 0x80) {
			sbi(DATPORT, DATPIN);
		} else {
			cbi(DATPORT, DATPIN);
		}
		__builtin_avr_delay_cycles(5);
		sbi(CLOCKPORT, CLOCKPIN);
		__builtin_avr_delay_cycles(5);
		cbi(CLOCKPORT, CLOCKPIN);
	}
	sbi(LOADPORT, LOADPIN);
	__builtin_avr_delay_cycles(5);
	cbi(LOADPORT, LOADPIN);
	cbi(CLOCKPORT, CLOCKPIN);
	cbi(DATPORT, DATPIN);
}