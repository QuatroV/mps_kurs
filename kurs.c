#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <inttypes.h>
#include <string.h>
#include <avr/pgmspace.h>

#define D_NUM 128

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
#define SPI_READ 0b00000011	 // Read data from memory array beginning at selected address
#define SPI_RDSR 0b00000101	 // Read Status Register
#define SPI_DUMMY 0x00		 // Dummy byte

#define PAGE_SIZE 32 // External EEPROM page size

#define PORTD_RS 0x20
#define DATA 1
#define COMMAND 0
#define lcd_on PORTC |= 0x80	   // PC7 to E (enable) of LCD = 1
#define lcd_off PORTC &= ~0x80	   // PC7 to E (enable) of LCD = 0
#define lcd_data PORTC |= 0x40	   // PC6 to RS = 1
#define lcd_command PORTC &= ~0x40 // PC6 to RS = 0

#define DELAY 1
#define LCD_STROBEDELAY 50000

uint16_t debug2 = 0;

volatile uint8_t buf_count = 0;
volatile uint8_t buf[32];
uint16_t buf_address = 0;
uint8_t values[D_NUM];

uint16_t addr = 0x0000;

volatile uint16_t tot_overflow;

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

void acc

	// ************************************END************************************

	// ************************************SPI************************************
	void
	spi_init(void)
{
	SPI_DDR |= ((1 << SS) | (1 << MOSI) | (1 << SCK));
	SPI_DDR &= ~(1 << MISO);
	SPI_PORT |= ((1 << SS) | (1 << MOSI) | (1 << MISO) | (1 << SCK));
	SPCR = (1 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (0 << SPR0);
	SPSR = (0 << SPI2X);
}

void EEPROM_Wait(void)
{
	_delay_ms(100);
}

void SPITransmitByte(uint8_t _data)
{
	SPI_ENABLE;
	SPDR = _data;
	while (!(SPSR & (1 << SPIF)))
		;
	SPI_DISABLE;
}

void SPIWriteByte(uint16_t address, uint8_t _data)
{
	SPITransmitByte(SPI_WREN);
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

	SPDR = _data;
	while (!(SPSR & (1 << SPIF)))
		;
	SPI_DISABLE;
}

uint8_t SPIReadByte(uint16_t address)
{
	EEPROM_Wait();
	uint8_t result = 0;
	SPI_ENABLE;
	SPDR = SPI_READ;
	while (!(SPSR & (1 << SPIF)))
		;

	SPDR = address >> 8;
	while (!(SPSR & (1 << SPIF)))
		;
	SPDR = address;
	while (!(SPSR & (1 << SPIF)))
		;

	SPDR = 0;
	while (!(SPSR & (1 << SPIF)))
		;
	result = SPDR;
	SPI_DISABLE;
	return result;
}

void SPIWritePage(uint16_t address, uint8_t *data, uint16_t n)
{
	SPITransmitByte(SPI_WREN);
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

void SPIBufferAppend(uint8_t data)
{
	buf[buf_count++] = data;
	if (buf_count == PAGE_SIZE)
	{
		EEPROM_Wait();
		SPIWritePage(buf_address, buf, PAGE_SIZE);
		buf_address = buf_address + PAGE_SIZE;
		buf_count = 0;
	}
}

void SPIBufferTransmit(void)
{
	EEPROM_Wait();
	SPIWritePage(buf_address, buf, buf_count);
	buf_address = buf_address + buf_count;
	buf_count = 0;
}

void SPISaveValues(void)
{
	register uint8_t i = 0;
	while (i < PAGE_SIZE)
	{
		SPIBufferAppend(values[i]);
	}
	SPIBufferTransmit();
}

void SPIClear(void)
{
	uint8_t i = 0;
	const uint8_t zeros[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	while (i < PAGE_SIZE * 2)
	{
		EEPROM_Wait();
		SPIWritePage(i * PAGE_SIZE, zeros, PAGE_SIZE);
		i++;
	}
	EEPROM_Wait();
}
// ************************************END************************************

// ************************************UART************************************

void uart_init(void)
{
	UBRRH = 0;
	UBRRL = 103;
	UCSRA = 0x00;
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void USARTTransmitChar(char _data)
{
	while (!(UCSRA & (1 << UDRE)))
		;
	UDR = _data;
}

void USARTTransmitString(PGM_P _data)
{
	uint8_t i = 0;
	char buf[strlen_P(_data)];
	strcpy_P(buf, _data);
	while (buf[i])
	{
		USARTTransmitChar(buf[i++]);
	}
}

void USARTTransmitStringLN(PGM_P _data)
{
	USARTTransmitString(_data);
	USARTTransmitChar((char)13);
	USARTTransmitChar((char)10);
}

void USARTTransmitBuffer()
{
	uint8_t i = 0;
	while (i < PAGE_SIZE)
	{
		USARTTransmitChar(buf[i++]);
	}
}

unsigned char USARTReceiveChar(void)
{
	while (!(UCSRA & (1 << RXC)))
		;
	return UDR;
}

// ************************************END************************************

// ******************************BUTTONS MATRIX*******************************
uint8_t key_state(void)
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

uint8_t debug3 = 0;

void LCDSend(uint8_t byte, char state)
{
	DDRD = 0b11111100; // External interrupt
	DDRB |= 0b00001111;
	tot_overflow++;
	if (state == 0)
		lcd_command;
	else
		lcd_data;
	PORTB &= 0xF0;			// clear first 4 bits
	PORTB |= (byte & 0x0F); // set first 4 bits
	PORTD &= 0x0F;			// clear last 4 bits
	PORTD |= (byte & 0xF0); // set last 4 bits
	lcd_on;
	_delay_us(LCD_STROBEDELAY);
	lcd_off;
	_delay_us(LCD_STROBEDELAY);
}

void LCDClear(void)
{
	_delay_ms(DELAY);
	LCDSend(0x01, COMMAND);
}

void LCDInit(void)
{
	LCDSend(0x38, COMMAND); // Function Set: 8-bit, 2 Line
	_delay_ms(DELAY);
	LCDSend(0x01, COMMAND); // Clear Display (also clear DDRAM content)
	_delay_ms(DELAY);
	LCDSend(0x06, COMMAND); // Entry Mode (No shift and auto incremement)
	_delay_ms(DELAY);
	LCDSend(0x0C, COMMAND); // Display on Cursor off
}

void LCDCursor(char row, char column)
{
	_delay_ms(DELAY);
	LCDSend(0b10000000 | ((0x40 * row) + column), COMMAND);
}

void LCDDisplayString(char row, char column, PGM_P string)
{
	LCDCursor(row, column);
	char buf[strlen_P(string)];
	strcpy_P(buf, string);
	uint8_t i = 0;
	while (buf[i])
		LCDSend(buf[i++], DATA);
}

void LCDDisplayStringT(char row, char column, char string[])
{
	LCDCursor(row, column);
	while (*string)
		LCDSend(*string++, DATA);
}

// ************************************END************************************

// Interrupts

// START button interrupt
ISR(INT0_vect)
{
	channel_number = key_state();
}

// CLEAR button interrupt
ISR(INT1_vect)
{
	SPIClear();
}

// ADC convert interrupt
ISR(ADC_vect)
{
	if (buf_count < PAGE_SIZE)
	{
		debug2 = ADC;
		buf[buf_count++] = (uint8_t)(ADC >> 2);
	}
	else
	{
		cli();
	}
}

uint8_t current_index = 0;
uint16_t current_voltage = 0;

char channel_number_str[2] = "";
char current_voltage_str[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/// @brief Init ports, interrupts globally, USART, SPI, LCD
void init()
{
	cli(); // Clear global interrupt flag in SREG (No interrupts)
	ports_init();

	interrupt_init();
	LCDInit();

	uart_init();
	spi_init();

	sei(); // Set global interrupt flag in SREG (Allow interrupts)
}

/// @brief Display legend on LCD
void display_legend_on_lcd()
{
	LCDClear();
	LCDDisplayString(2, 0, PSTR("Channel: "));
	LCDDisplayString(1, 0, PSTR("Value: "));
}

/// @brief Send data to extrenal EEPROM, LCD, Terminal
void send_buf()
{
	// Send data buffer with SPI to external EEPROM
	EEPROM_Wait();
	SPIWritePage(buf_address, buf, PAGE_SIZE);
	buf_address = buf_address + PAGE_SIZE;
	buf_count = 0;

	// Send data buffer with USART to terminal
	USARTTransmitBuffer();

	// Convert channel number and send to LCD
	channel_number_str[0] = (channel_number + 1) + '0';
	channel_number_str[1] = 0;
	LCDDisplayStringT(2, 9, channel_number_str);

	// Convert last value from buffer and send to LCD
	itoa(buf[31], current_voltage_str, 10);
	LCDDisplayStringT(1, 9, "   ");
	LCDDisplayStringT(1, 9, current_voltage_str);
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
			if (key_state() != 255)
			{
				channel_number = key_state();
			}

			adc_init();
			sei(); // Allow global interrupts to continue converting data with ADC
		}
	};

	return 0;
}
