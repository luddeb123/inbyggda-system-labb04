#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <string.h>

#include "i2c.h"

void i2c_init(void)
{
	uint32_t sclFrequency = 100000; // 100 000 Hz
	uint8_t prescaler = 0;
	// sclFrequency = cpuFrequency / (16 + (2 * TWBR * prescalerVal))
	// twbr = ((cpuFrequency / sclFrequency) - 16)/(2*prescaler)
	uint8_t twbrVal = ((F_CPU / sclFrequency) - 16) / (2 * prescaler);

	TWCR = (1 << TWEN);
	TWSR = prescaler;

	// twbr = ((16 000 000 / 100 000) - 16)/(2*1)
	// twbr = ((160)-16)/(2*1)
	// twbr = (144)/(2*1)
	// twbr = 144/2
	// twbr = 72
	TWBR = twbrVal;
}

void i2c_meaningful_status(uint8_t status)
{
	switch (status)
	{
	case 0x08: // START transmitted, proceed to load SLA+W/R
		printf_P(PSTR("START\n"));
		break;
	case 0x10: // repeated START transmitted, proceed to load SLA+W/R
		printf_P(PSTR("RESTART\n"));
		break;
	case 0x38: // NAK or DATA ARBITRATION LOST
		printf_P(PSTR("NOARB/NAK\n"));
		break;
	// MASTER TRANSMIT
	case 0x18: // SLA+W transmitted, ACK received
		printf_P(PSTR("MT SLA+W, ACK\n"));
		break;
	case 0x20: // SLA+W transmitted, NAK received
		printf_P(PSTR("MT SLA+W, NAK\n"));
		break;
	case 0x28: // DATA transmitted, ACK received
		printf_P(PSTR("MT DATA+W, ACK\n"));
		break;
	case 0x30: // DATA transmitted, NAK received
		printf_P(PSTR("MT DATA+W, NAK\n"));
		break;
	// MASTER RECEIVE
	case 0x40: // SLA+R transmitted, ACK received
		printf_P(PSTR("MR SLA+R, ACK\n"));
		break;
	case 0x48: // SLA+R transmitted, NAK received
		printf_P(PSTR("MR SLA+R, NAK\n"));
		break;
	case 0x50: // DATA received, ACK sent
		printf_P(PSTR("MR DATA+R, ACK\n"));
		break;
	case 0x58: // DATA received, NAK sent
		printf_P(PSTR("MR DATA+R, NAK\n"));
		break;
	default:
		printf_P(PSTR("N/A %02X\n"), status);
		break;
	}
}

// Most of this is taken from p.225 in the atmega-datasheet including most of the comments.
inline void i2c_start()
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Send START condition
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the START condition has been transmitted
}

inline void i2c_stop()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Transmit STOP condition
	while (TWCR & (1 << TWSTO)); // TWSTO clears when STOP condition is executed
}

inline uint8_t i2c_get_status(void)
{
	uint8_t status = TWSR & 0xF8; // The status of the TWI logic and the 2-wire serial bus
	return status;
}

inline void i2c_xmit_addr(uint8_t address, uint8_t rw)
{
	TWDR = (address & 0xfe) | (rw & 0x01); // Write address and read/write
	TWCR = (1 << TWINT) | (1 << TWEN); // Start the twi operation
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
}

inline void i2c_xmit_byte(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN); // start the twi operation
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
}

inline uint8_t i2c_read_ACK()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); 
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received
	return TWDR;
}

inline uint8_t i2c_read_NAK()
{
	TWCR = (1 << TWINT) | (1 << TWEN); 
	while (!(TWCR & (1 << TWINT))); // Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received
	return TWDR;
}

inline void eeprom_wait_until_write_complete()
{
	while (i2c_get_status() != 0x18)
	{
		i2c_start();
		i2c_xmit_addr(ADDR, I2C_W);
	}
}

uint8_t eeprom_read_byte(uint8_t addr)
{
	uint8_t readByte;

	i2c_start(); // Start comunication
	i2c_xmit_addr(ADDR, I2C_W); // Transmit eeprom address and write
	i2c_xmit_byte(addr); // Transmit the address of the data 
	i2c_start(); // Restart
	i2c_xmit_addr(ADDR, I2C_R); // Transmit eeprom address and read
	// readByte = i2c_read_ACK();	// Receive data
	readByte = i2c_read_NAK();	// Receive data
	i2c_stop(); // Stop
	return readByte;
}

void eeprom_write_byte(uint8_t addr, uint8_t data)
{
	i2c_start(); // Start
	i2c_xmit_addr(ADDR, I2C_W); // Transmit eeprom addres and write
	i2c_xmit_byte(addr); // Move pointer to this address
	i2c_xmit_byte(data); // Transmit data
	i2c_stop(); // Stop
	eeprom_wait_until_write_complete(); 
}

void eeprom_write_page(uint8_t addr, uint8_t *data)
{
	// ... (VG)
}

void eeprom_sequential_read(uint8_t *buf, uint8_t start_addr, uint8_t len)
{
	// ... (VG)
}
