/* Operate a motorized potentiometer based on HDMI CEC volume commands. */

/*#define DEBUG 1*/

#define F_CPU 16384000UL

#ifdef DEBUG
#include <stdio.h>
#endif
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/* The HDMI address for this device.  Respond to CEC messages sent to this
 * address. */
#define ADDRESS 0x05

/* The duration to turn the motor for a single volume step.  On my setup this
 * value, which I found through trial and error, produces a small but audible
 * change in volume, but it depends heavily on the characteristics of the
 * receiver, volume potentiometer, and listener. */
#define STEP_TICKS 12000

#ifdef DEBUG
static int uart_put(unsigned char, FILE *);

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_put, NULL,
                                            _FDEV_SETUP_WRITE);

static void
uart_init(void)
{
	/* Baud rate 500000 */
	UBRRL = 1;

	UCSRB = _BV(RXEN) | _BV(TXEN);
	UCSRC = _BV(UCSZ1) | _BV(UCSZ0);
}

static int
uart_put(unsigned char c, FILE *stream)
{
	if (c == '\n')
		uart_put('\r', stream);
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
	return 0;
}
#endif /* DEBUG */

static void
send_ack(void)
{
	/* Send a follower-initiated ACK.  This must be called immediately
	 * after a falling edge has occured. */
	uint16_t ticks_start;
	uint16_t ticks;

	ticks_start = TCNT1;

	/* Pull the CEC line low. */
	DDRD = 0x40;

	for (;;) {
		ticks = TCNT1;

		/* optimal 1.5 ms */
		if ((ticks - ticks_start) >= 96) {
			/* Set the CEC line back to high-Z. */
			DDRD = 0x00;
			break;
		}
	}
}

enum edge { FALLING, RISING };

static uint16_t
wait_edge(enum edge e)
{
	uint16_t ticks;
	uint8_t last, cec;

	last = cec = (PIND & 0x40) >> 6;

	for (;;) {
		ticks = TCNT1;

		last = cec;
		cec = (PIND & 0x40) >> 6;

		if (e == RISING) {
			if ((last == 0) && (cec == 1)) {
				return ticks;
			}
		}
		else {
			if ((last == 1) && (cec == 0)) {
				return ticks;
			}
		}
	}
}

static uint16_t
wait_falling_edge(void)
{
	return wait_edge(FALLING);
}

static uint16_t
wait_rising_edge(void)
{
	return wait_edge(RISING);
}

static uint8_t
recv_data_bit(void)
{
	/* Sample a bit, must be called immediately after a falling edge
	 * occurs. */
	uint16_t ticks_start;
	uint16_t ticks;

	ticks_start = TCNT1;

	for (;;) {
		ticks = TCNT1;

		/* optimal 1.05 ms */
		if ((ticks - ticks_start) >= 67) {
			return (PIND & 0x40) >> 6;
		}
	}
}

static int8_t
wait_start_bit(void)
{
	uint16_t ticks_start;
	uint16_t ticks;

	/* A start bit consists of a falling edge followed by a rising edge
	 * between 3.5 ms and 3.9 ms after the falling edge, and a second
	 * falling edge between 4.3 and 4.7 ms after the first falling edge.
	 * Wait until those conditions are met, and start over at the next
	 * falling edge if any threshold is exceeded. */
	for (;;) {
		ticks_start = wait_falling_edge();
		ticks = wait_rising_edge();
		/* Rising edge took longer than 3.9 ms, start over */
		if ((ticks - ticks_start) >= 250) {
			continue;
		}
		/* Rising edge occured between 3.5 ms and 3.9 ms */
		else if ((ticks - ticks_start) >= 224) {
			ticks = wait_falling_edge();
			/* Falling edge took longer than 4.7 ms, start over */
			if ((ticks - ticks_start) >= 301) {
				continue;
			}
			/* Falling edge between 4.3 ms and 4.7 ms means that
			 * this has been a start bit! */
			else if ((ticks - ticks_start) >= 276) {
				return 0;
			}
			/* The falling edge came too early, start over */
			else {
				continue;
			}
		}
		/* The rising edge came sooner than 3.5 ms, start over */
		else {
			continue;
		}
	}
}

static int8_t
recv_frame(uint8_t *pld, uint8_t address)
{
	uint16_t ticks_start;
	uint16_t ticks;
	uint8_t bit_count;
	uint8_t pldcnt;
	uint8_t eom;

	wait_start_bit();

	bit_count = 9;
	pldcnt = 0;
	pld[pldcnt] = 0;

	/* Read blocks into pld until the EOM bit signals that the message is
	 * complete.  Each block is 10 bits consisting of information bits 7-0,
	 * an EOM bit, and an ACK bit.  The initiator sends the information
	 * bits and the EOM bit and expects the follower to send a '0' during
	 * the ACK bit to acknowledge receipt of the block. */
	for (;;) {
		/* At this point in the loop, a falling edge has just occured,
		 * either in wait_start_bit() above or wait_falling_edge() at
		 * the end of the loop, so it is time to sample a bit. */
		ticks_start = TCNT1;

		/* Only store and return the information bits. */
		if (bit_count > 1) {
			pld[pldcnt] <<= 1;
			pld[pldcnt] |= recv_data_bit();
		}
		else {
			eom = recv_data_bit();
		}
		bit_count--;

		/* Wait for the starting falling edge of the next bit. */
		ticks = wait_falling_edge();
		/* 2.05 ms */
		if ((ticks - ticks_start) < 131) {
#ifdef DEBUG
			printf_P(PSTR("min\n"));
#endif /* DEBUG */
			return -1;
		}
		ticks_start = ticks;
		/* If that was the EOM bit, it's time to send an ACK and either
		 * return the data (if EOM was indicated by the initiator) or
		 * prepare to read another block. */
		if (bit_count == 0) {
			/* Only ACK messages addressed to us (indicated by the
			 * header block). */
			if ((pld[0] & 0x0f) == address) {
				send_ack();
			}
			if (eom) {
				/* Don't consume the falling edge in this case
				 * because it could be the start of the next
				 * start bit! */
				return pldcnt + 1;
			}
			else {
				/* Wait for the starting falling edge of the
				 * next bit. */
				ticks = wait_falling_edge();
				/* 2.75 ms */
				if ((ticks - ticks_start) >= 176) {
#ifdef DEBUG
					printf_P(PSTR("max\n"));
#endif /* DEBUG */
					return -1;
				}
			}

			bit_count = 9;
			pldcnt++;
			pld[pldcnt] = 0;
		}
	}
}

static void
send_start_bit(void)
{
	/* Pull the line low for 3.7 ms and then high again until the 4.5 ms
	 * mark.  This function doesn't produce the final falling edge of the
	 * start bit - that is left to send_data_bit(). */
	uint16_t ticks;
	uint16_t ticks_start;

	ticks_start = TCNT1;

	/* Pull the CEC line low. */
	DDRD = 0x40;

	for (;;) {
		ticks = TCNT1;
		/* 3.7 ms */
		if ((ticks - ticks_start) >= 237) {
			break;
		}
	}

	/* Set the CEC line back to high-Z. */
	DDRD = 0x00;

	for (;;) {
		ticks = TCNT1;
		/* 4.5 ms */
		if ((ticks - ticks_start) >= 288) {
			break;
		}
	}
}

static void
send_data_bit(int8_t bit)
{
	/* A data bit consists of a falling edge at T=0ms, a rising edge, and
	 * another falling edge at T=2.4ms.  The timing of the rising edge
	 * determines the bit value.  The rising edge for an optimal logical 1
	 * occurs at T=0.6ms.  The rising edge for an optimal logical 0 occurs
	 * at T=1.5ms. */
	uint16_t ticks;
	uint16_t ticks_start;

	ticks_start = TCNT1;

	/* Pull the CEC line low. */
	DDRD = 0x40;

	for (;;) {
		ticks = TCNT1;
		if (bit) {
			/* 0.6 ms */
			if ((ticks - ticks_start) >= 39) {
				break;
			}
		}
		else {
			/* 1.5 ms */
			if ((ticks - ticks_start) >= 96) {
				break;
			}
		}
	}

	/* Set the CEC line back to high-Z. */
	DDRD = 0x00;

	for (;;) {
		ticks = TCNT1;
		/* 2.4 ms */
		if ((ticks - ticks_start) >= 154) {
			break;
		}
	}
}

static void
send_frame(uint8_t pldcnt, uint8_t *pld)
{
	uint8_t bit_count;
	uint8_t i;

	send_start_bit();

	for (i = 0; i < pldcnt; i++) {
		bit_count = 7;
		/* Information bits. */
		do {
			send_data_bit((pld[i] >> bit_count) & 0x01);
		} while (bit_count--);
		/* EOM bit. */
		send_data_bit(i == (pldcnt - 1));
		/* ACK bit (we will assume the block was received). */
		send_data_bit(1);
	}
}

static void
device_vendor_id(uint8_t initiator, uint8_t destination, uint32_t vendor_id)
{
	uint8_t pld[5];

	pld[0] = (initiator << 4) | destination;
	pld[1] = 0x87;
	pld[2] = (vendor_id >> 16) & 0x0ff;
	pld[3] = (vendor_id >> 8) & 0x0ff;
	pld[4] = (vendor_id >> 0) & 0x0ff;

	send_frame(5, pld);
}

static void
report_power_status(uint8_t initiator, uint8_t destination, uint8_t power_status)
{
	uint8_t pld[3];

	pld[0] = (initiator << 4) | destination;
	pld[1] = 0x90;
	pld[2] = power_status;

	send_frame(3, pld);
}

static void
set_system_audio_mode(uint8_t initiator, uint8_t destination, uint8_t system_audio_mode)
{
	uint8_t pld[3];

	pld[0] = (initiator << 4) | destination;
	pld[1] = 0x72;
	pld[2] = system_audio_mode;

	send_frame(3, pld);
}

static void
set_osd_name(uint8_t initiator, uint8_t destination)
{
	uint8_t pld[15] = {
	    0, 0x47,
	    'P', 'i', 'o', 'n', 'e', 'e', 'r', 'S', 'X', '-', '9', '5', '0' };

	pld[0] = (initiator << 4) | destination;

	send_frame(15, pld);
}

static void
report_physical_address(uint8_t initiator, uint8_t destination, uint16_t physical_address, uint8_t device_type)
{
	uint8_t pld[5];

	pld[0] = (initiator << 4) | destination;
	pld[1] = 0x84;
	pld[2] = (physical_address >> 8) & 0x0ff;
	pld[3] = (physical_address >> 0) & 0x0ff;
	pld[4] = device_type;

	send_frame(5, pld);
}

ISR (TIMER1_COMPA_vect)
{
#ifdef DEBUG
	printf_P(PSTR("<int>\n"));
#endif
	PORTB = 0;
	wdt_reset();
}

int
main(void)
{
	/* Operate a motorized potentiometer based on HDMI CEC volume commands.
	 * Pins 0 and 1 of port B are connected to the A inputs of an L293DNE
	 * half-H driver.  The HDMI CEC line is connected to pin 6 of port D.
	 * The AVR is clocked at 16.384 MHz. */
	uint8_t pld[16];
	int8_t pldcnt;
	uint8_t initiator, destination;
	uint8_t repeat = 0;
#ifdef DEBUG
	int i;
#endif /* DEBUG */

	/* This needs to be executed first to make sure the motor is stopped,
	 * just in case we end up here by way of the watchdog timer. */
	PORTB = 0;
	/* The CEC line is an open-collector bus, so we leave PORTD permanently
	 * set to 0.  This allows us to toggle its pins between low impedance
	 * and high impedance by setting and clearing bits in DDRD. */
	PORTD = 0;

	wdt_enable(WDTO_2S);

#ifdef DEBUG
	uart_init();
        stdout = &uart_stdout;

	printf_P(PSTR("s\n"));
#endif /* DEBUG */

	/* Set up timer 1 to count up at CLK / 256.  This comes out to 64000
	 * ticks per second.  Timing will be achieved by sampling TCNT1 at
	 * different times and comparing the difference. */
	TCCR1A = 0;
	TCCR1B = _BV(CS12); /* CLK / 256 */
	TCNT1 = 0;
	/* Also enable interrupts for timer 1.  For momentary volume changes,
	 * the main loop will turn on the motor and set OCR1A to STEP_TICKS in
	 * the future.  The interrupt handler will turn off the motor.  The
	 * interrupt is also used during for long presses as an added safety
	 * measure. */
	TIMSK = _BV(OCIE1A);
	sei();

	DDRB = 0x03;

	for (;;) {
		pldcnt = recv_frame(pld, ADDRESS);
		if (pldcnt < 0) {
#ifdef DEBUG
			printf_P(PSTR("error %i\n"), pldcnt);
#endif /* DEBUG */
			continue;
		}
		initiator = (pld[0] & 0xf0) >> 4;
		destination = pld[0] & 0x0f;
#ifdef DEBUG
		for (i = 0; i < pldcnt - 1; i++) {
			printf_P(PSTR("%02x:"), pld[i]);
		}
		printf_P(PSTR("%02x\n"), pld[i]);
#endif /* DEBUG */
		if ((destination == ADDRESS) && (pldcnt > 0)) {
			switch (pld[1]) {
				case 0x8c:
#ifdef DEBUG
					/* Give Device Vendor ID */
					printf_P(PSTR("<vendor id>\n"));
#endif /* DEBUG */
					_delay_ms(13);
					device_vendor_id(ADDRESS, 0x0f, 0x000045);
					break;
				case 0x8f:
#ifdef DEBUG
					/* Give Device Power Status */
					printf_P(PSTR("<power status>\n"));
#endif /* DEBUG */
					_delay_ms(13);
					/* On */
					report_power_status(ADDRESS, initiator, 0x00);
					break;
				case 0x46:
#ifdef DEBUG
					/* Give OSD Name */
					printf_P(PSTR("<osd name>\n"));
#endif /* DEBUG */
					_delay_ms(13);
					set_osd_name(ADDRESS, initiator);
					break;
				case 0x83:
#ifdef DEBUG
					/* Give Physical Address */
					printf_P(PSTR("<phys address>\n"));
#endif /* DEBUG */
					_delay_ms(13);
					report_physical_address(ADDRESS, 0x0f, 0x0005, 0x05);
					break;
				case 0x70:
#ifdef DEBUG
					/* System Audio Mode Request */
					printf_P(PSTR("<sys audio>\n"));
#endif /* DEBUG */
					_delay_ms(13);
					set_system_audio_mode(ADDRESS, 0x0f, 1);
					break;
				case 0x44:
					if (pld[2] == 0x41) {
						/* Volume Up Pressed */
#ifdef DEBUG
						printf_P(PSTR("<vol up>\n"));
#endif /* DEBUG */
						repeat = ((PORTB & 0x01) != 0);
						OCR1A = TCNT1 + STEP_TICKS;
						TIFR = _BV(OCF1A);
						TIMSK = _BV(OCIE1A);
						PORTB = 0x01;
					}
					else if (pld[2] == 0x42) {
						/* Volume Down Pressed */
#ifdef DEBUG
						printf_P(PSTR("<vol down>\n"));
#endif /* DEBUG */
						repeat = ((PORTB & 0x02) != 0);
						OCR1A = TCNT1 + STEP_TICKS;
						TIFR = _BV(OCF1A);
						TIMSK = _BV(OCIE1A);
						PORTB = 0x02;
					}
					break;
				case 0x45:
#ifdef DEBUG
					printf_P(PSTR("<released>\n"));
#endif /* DEBUG */
					if (repeat)
						PORTB = 0;
					break;
				default:
					break;
			}
		}
	}

	return 0;
}
