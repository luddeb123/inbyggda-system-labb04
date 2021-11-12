#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "serial.h"
#include "timer.h"

char input[] = "Ludvig";
char output[7];

void main(void)
{

	i2c_init();
	uart_init();

	sei();

	// eeprom_write_byte(EEPROM_ADDR, 'X'); // I used this to write only one char.
	for (int i = 0; i < sizeof(input); i++)
	{
		eeprom_write_byte(EEPROM_ADDR + i, input[i]);
	}

	while (1)
	{
		// printf_P(PSTR("%c\n"), eeprom_read_byte(EEPROM_ADDR)); // I used this to check only one char.
		for (int j = 0; j < sizeof(output); j++)
		{
			output[j] = eeprom_read_byte(EEPROM_ADDR + j);
		}

		printf_P(PSTR("%s\n"), output);
	}
}
