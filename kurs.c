#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <string.h>
#include <avr/pgmspace.h>

// EEPROM macros
#define PAGE_SIZE 32 // External EEPROM page size

// SPI macros
#define SPI_PORT PORTB
#define SPI_DDR DDRB
#define SS PB4
#define MOSI PB5
#define MISO PB6
#define SCK PB7
#define SPI_ENABLE PORTB &= ~(1 << SS)
#define SPI_DISABLE PORTB |= (1 << SS)
#define SPI_WREN 0b00000110	 // Set the write enable latch (enable write operations)
#define SPI_WRITE 0b00000010 // Write data to memory array beginning at selected address

// LCD macros
#define DATA 1
#define COMMAND 0
#define lcd_on PORTC |= 0x80	   // PC7 to E (enable) of LCD = 1
#define lcd_off PORTC &= ~0x80	   // PC7 to E (enable) of LCD = 0
#define lcd_data PORTC |= 0x40	   // PC6 to RS = 1
#define lcd_command PORTC &= ~0x40 // PC6 to RS = 0

#define DELAY 1
#define LCD_STROBEDELAY 5

volatile uint8_t buf_count = 0;
volatile uint8_t buf[32];
uint16_t buf_address = 0;

volatile uint8_t channel_number = 9;

void ports_init(void)
{
	DDRC = 0b11000000; // Through PORTD data from button matrix
	PORTC = 0b11000000;
	DDRD = 0b11111100; // External interrupt
	PORTD = 0b00000000;
	PORTB = 0b00000000;
	DDRB = 0b10001111;
}

void interrupt_init(void)
{
	GICR |= ((1 << INT0) | (1 << INT1));
	GIFR = 0xC0;
	OFF(&GIFR, 0xC0);
	MCUCR = 0x0A;
}

void OFF(volatile uint8_t *byte, uint8_t mask)
{
	*byte &= ~mask; // set values to zero by mask
}

// ************************************ADC************************************

void adc_init()
{
	ADMUX = (0 << REFS0); // Vref=AVcc
	// ADSC=1 ADC Enable
	// ADPS[2:0]=10, prescaler=128
	// ADIE=1, ADC interrupt Enable
	// ADATE=1, ADC Auto Triggering Enable
	ADCSRA = (1 << ADEN) | (3 << ADPS0) | (1 << ADSC) | (1 << ADIE) | (1 << ADATE);

	ADMUX = (ADMUX & 0xF8) | channel_number;
}

// ************************************END************************************

// ************************************SPI************************************
void spi_init(void)
{
	SPI_DDR |= ((1 << SS) | (1 << MOSI) | (1 << SCK));
	SPI_DDR &= ~(1 << MISO);
	SPI_PORT |= ((1 << SS) | (1 << MOSI) | (1 << MISO) | (1 << SCK));
	SPCR = (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (0 << SPR0);
	SPSR = (0 << SPI2X);
}

void eeprom_wait(void)
{
	_delay_ms(100);
}

void spi_transmit_byte(uint8_t _data)
{
	SPI_ENABLE;
	SPDR = _data;
	while (!(SPSR & (1 << SPIF)))
		;
	SPI_DISABLE;
}

void spi_write_page(uint16_t address, uint8_t *data, uint16_t n)
{
	spi_transmit_byte(SPI_WREN);
	SPI_ENABLE;
	SPDR = SPI_WRITE;
	while (!(SPSR & (1 << SPIF)))
		;
	SPDR = address >> 8;
	while (!(SPSR & (1 << SPIF)))
		;
	SPDR = address;
	while (!(SPSR & (1 << SPIF)))
		;
	short num = n;
	while (num--)
	{
		SPDR = *data++;
		while (!(SPSR & (1 << SPIF)))
			;
	}
	SPI_DISABLE;
}

void spi_clear(void)
{
	uint8_t i = 0;
	const uint8_t zeros[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	while (i < PAGE_SIZE * 2)
	{
		eeprom_wait();
		spi_write_page(i * PAGE_SIZE, zeros, PAGE_SIZE);
		i++;
	}
	eeprom_wait();
}
// ************************************END************************************

// ************************************UART************************************

void uart_init(void)
{
	UBRRH = 0;
	UBRRL = 104;
	UCSRA = 0x00;
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void usart_transmit_char(char _data)
{
	while (!(UCSRA & (1 << UDRE)))
		;
	UDR = _data;
}

void usart_transmit_string(PGM_P _data)
{
	uint8_t i = 0;
	char buf[strlen_P(_data)];
	strcpy_P(buf, _data);
	while (buf[i])
	{
		usart_transmit_char(buf[i++]);
	}
}

void usart_transmit_stringLN(PGM_P _data)
{
	usart_transmit_string(_data);
	usart_transmit_char((char)13);
	usart_transmit_char((char)10);
}

void usart_transmit_buffer()
{
	uint8_t i = 0;
	while (i < PAGE_SIZE)
	{
		usart_transmit_char(buf[i++]);
	}
}

// ************************************END************************************

// ******************************BUTTONS MATRIX*******************************

uint8_t get_matrix_state(void)
{
	// Matrix
	const uint8_t state_ddr[4] = {0xC1, 0xC2, 0xC4, 0xC8};
	const uint8_t keypad[4][2] = {
		{0, 4},
		{1, 5},
		{2, 6},
		{3, 7}};

	uint8_t res = 255;
	uint8_t i = 0;
	while (i < 4)
	{
		DDRC = state_ddr[i];
		PORTC = 0x00;
		if (!(PINC & (1 << 5)))
		{
			res = keypad[i][1];
		}
		if (!(PINC & (1 << 4)))
		{
			res = keypad[i][0];
		}
		i++;
	}

	return res;
}
// ************************************END************************************

// **********************************DISPLAY**********************************

void lcd_send(uint8_t byte, char state)
{
	DDRD = 0b11111100; // External interrupt
	DDRB |= 0b00001111;
	if (state == 0)
		lcd_command;
	else
		lcd_data;
	PORTB &= 0xF0;			// clear first 4 bits
	PORTB |= (byte & 0x0F); // set first 4 bits
	PORTD &= 0x0F;			// clear last 4 bits
	PORTD |= (byte & 0xF0); // set last 4 bits
	lcd_on;
	_delay_ms(7 * DELAY);
	lcd_off;
	_delay_ms(7 * DELAY);
}

void lcd_clear(void)
{
	_delay_ms(DELAY);
	lcd_send(0x01, COMMAND);
}

void lcd_init(void)
{
	lcd_send(0x38, COMMAND); // Function Set: 8-bit, 2 Line
	_delay_ms(DELAY);
	lcd_send(0x01, COMMAND); // Clear Display (also clear DDRAM content)
	_delay_ms(DELAY);
	lcd_send(0x06, COMMAND); // Entry Mode (No shift and auto incremement)
	_delay_ms(DELAY);
	lcd_send(0x0C, COMMAND); // Display on Cursor off
}

void lcd_cursor(char row, char column)
{
	_delay_ms(DELAY);
	lcd_send(0b10000000 | ((0x40 * row) + column), COMMAND);
}

void lcd_display_string(char row, char column, char string[])
{
	lcd_cursor(row, column);
	while (*string)
		lcd_send(*string++, DATA);
}

// ************************************END************************************

// Interrupts

/// @brief START button interrupt
ISR(INT0_vect)
{
	channel_number = get_matrix_state();
}

/// @brief CLEAR button interrupt
ISR(INT1_vect)
{
	spi_clear();
}

/// @brief ADC convert interrupt
ISR(ADC_vect)
{
	if (buf_count < PAGE_SIZE)
	{
		buf[buf_count++] = (uint8_t)(ADC >> 2);
	}
}

/// @brief Init ports, interrupts globally, USART, SPI, LCD
void init()
{
	cli(); // Clear global interrupt flag in SREG (No interrupts)
	ports_init();

	interrupt_init();
	lcd_init();

	uart_init();
	spi_init();

	sei(); // Set global interrupt flag in SREG (Allow interrupts)
}

/// @brief Display legend on LCD
void display_legend_on_lcd()
{
	lcd_clear();
	lcd_display_string(2, 0, "Channel: ");
	lcd_display_string(1, 0, "Value: ");
}

/// @brief Send data to extrenal EEPROM, LCD, Terminal
void send_buf()
{
	char channel_number_str[2] = "";
	char current_voltage_str[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	// Send data buffer with SPI to external EEPROM
	eeprom_wait();
	spi_write_page(buf_address, buf, PAGE_SIZE);
	buf_address = buf_address + PAGE_SIZE;
	buf_count = 0;

	// Send data buffer with USART to terminal
	usart_transmit_buffer();

	// Convert channel number and send to LCD
	channel_number_str[0] = (channel_number + 1) + '0';
	channel_number_str[1] = 0;
	lcd_display_string(2, 9, channel_number_str);

	// Convert last value from buffer and send to LCD
	itoa(buf[31], current_voltage_str, 10);
	lcd_display_string(1, 9, "   ");
	lcd_display_string(1, 9, current_voltage_str);
}

int main()
{
	init();

	display_legend_on_lcd();

	// Wait for answer (channel number) from button matrix
	while (channel_number == 9)
		;

	// Start ADC convertion
	adc_init();

	while (1)
	{
		if (buf_count == PAGE_SIZE && buf_address < 2000)
		{
			cli(); // Clear global interrupt flag in SREG (No interrupts), because ADC interrupt prohibits sending data

			send_buf();

			// Check button matrix to observe channel number change
			if (get_matrix_state() != 255)
			{
				channel_number = get_matrix_state();
			}

			adc_init();
			sei(); // Allow global interrupts to continue converting data with ADC
		}
	};

	return 0;
}
