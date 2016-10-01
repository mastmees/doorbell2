/* The MIT License (MIT)
 
  Copyright (c) 2016 Madis Kaal <mast@nomad.ee>
 
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>

#define dac_ldac_low() PORTC &= (~_BV(PC2))
#define dac_ldac_high() PORTC |= _BV(PC2)
#define dac_cs_low() PORTC &= (~_BV(PC1))
#define dac_cs_high() PORTC |= _BV(PC1)
#define dac_clk_low() PORTC &= (~_BV(PC4))
#define dac_clk_high() PORTC |= _BV(PC4)
#define dac_sdi_low() PORTC &= (~_BV(PC3))
#define dac_sdi_high() PORTC |= _BV(PC3)

#define spi_clock_low() PORTB &= (~_BV(PB5))
#define spi_clock_high() PORTB |= _BV(PB5)
#define spi_mosi_high() PORTB |= _BV(PB3)
#define spi_mosi_low() PORTB &= (~_BV(PB3))
#define spi_miso_bit() ((PINB & _BV(PB4))?1:0)
#define eeprom_cs_low() PORTC &= (~_BV(PC0))
#define eeprom_cs_high() PORTC |= _BV(PC0)
// PC5 low is used to indicate timer interrupt handler
// processing time, to see how much cpu cycles are available
// between playing audio samples (with 20MHz clock less than
// 50% is currently used).
#define busy_low() PORTC &= (~_BV(PC5))
#define busy_high() PORTC |= _BV(PC5)

#define disable_playback() { TIMSK1=0; eeprom_cs_high(); }
#define enable_playback() TIMSK1=1; // enable TIMER1 overflow interrupts
#define disable_button() EIMSK=0;
#define enable_button() { EIFR=3; EIMSK=1; } // enable INT0 interrupts

void start_playback();

volatile uint8_t sample;
uint32_t sample_count,sample_len;

// send command byte and 24-bit address to EEPROM chip
// /CS stays low
void ee_set_addr(uint32_t cmd_addr)
{
  eeprom_cs_low();
  uint8_t c=32;
  spi_clock_low();
  while (c--) {
    if (cmd_addr&0x80000000)
      spi_mosi_high();
    else
      spi_mosi_low();
    spi_clock_high();
    cmd_addr<<=1;
    spi_clock_low();
  }
}

// clock next byte out of eeprom
uint8_t ee_read_byte()
{
uint8_t c=8,d=0;
  while (c--) {
    d<<=1;
    spi_clock_high();
    d|=spi_miso_bit();
    spi_clock_low();
  }
  return d;
}

// find last byte with non-FF value, that determines the number of
// samples in the audio
void find_end(void)
{
  ee_set_addr(0x03000000);
  sample_len=0;
  for (uint32_t i=0;i<128L*1024L;i++) { // for 25LC1024
    if (ee_read_byte()!=0xff)
      sample_len=i;
  }
  eeprom_cs_high();
}

void init_read(void)
{
  ee_set_addr(0x03000000);
  // we are now ready to start reading data from beginning of EEPROM
  // while the /CS is held low, bytes can be clocked out
  // continously
  sample_count=sample_len;
}


void start_playback()
{
  disable_playback();
  // configure timer1 for periodic interrupts
  TCCR1A=0;
  TCCR1B=1;
  TCNT1H=0xff;
  TCNT1L=0x00;
  init_read();
  if (sample_count) {
    sample=ee_read_byte();
    enable_playback();
    disable_button();
  }
}


ISR(TIMER1_OVF_vect)
{
  // reset timer for next interrupt
  #define X 869 // 22050 Hz sampling rate with 20MHz clock
  TCNT1H=(65535-X)/256;
  TCNT1L=(65535-X)%256;
  busy_low();
  register uint8_t c=8;
  // latch previously sent sample to dac output
  dac_ldac_low();
  dac_ldac_high();
  // send next sample to DAC registers
  // the sample is already prepared
  // unbuffered VREF, x1 amplification, DAC active
  dac_cs_low();
  dac_clk_low();
  dac_sdi_low();
  dac_clk_high(); // send 0
  dac_clk_low();  
  dac_clk_high(); // send 0
  dac_clk_low();
  dac_sdi_high();
  dac_clk_high(); // send 1
  dac_clk_low();
  dac_clk_high(); // send 1
  dac_clk_low();
  // clock out 8 data bits of next sample to DAC
  // and at a same time read next 8 bits from EEPROM 
  while (c--) {
    if (sample&0x80)
      dac_sdi_high();
    else
      dac_sdi_low();
    dac_clk_high();
    sample<<=1;
    dac_clk_low();
    spi_clock_high();
    sample|=spi_miso_bit();
    spi_clock_low();
  }
  dac_sdi_low();
  dac_clk_high(); // send 0
  dac_clk_low();
  dac_clk_high(); // send 0
  dac_clk_low();
  dac_clk_high(); // send 0
  dac_clk_low();
  dac_clk_high(); // send 0
  dac_clk_low();  
  dac_cs_high();
  if (!--sample_count) {
    disable_playback();
    enable_button();
  }
  busy_high();
}

ISR(WDT_vect)
{
}

ISR(INT0_vect)
{
  start_playback();
}

/*
I/O configuration
-----------------
I/O pin                               direction    DDR  PORT
PC0 EEPROM /CS                        output       1    1
PC1 DAC /CS                           output       1    1
PC2 DAC /LDAC                         output       1    1
PC3 DAC SDI                           output       1    1
PC4 DAC SCK                           output       1    1
PC5 unused                            output       1    1

PD0 unused                            output       1    1
PD1 unused                            output       1    1
PD2 button input                      input, p-up  0    1
PD3 unused                            output       1    1
PD4 unused                            output       1    1
PD5 unused                            output       1    1
PD6 unused                            output       1    1
PD7 unused                            output       1    1

PB0 unused                            output       1    1
PB1 unused                            output       1    1
PB2 unused                            output       1    1
PB3 SPI MOSI                          output       1    1
PB4 SPI MISO                          input, p-up  0    1
PB5 SPI SCK                           output       1    1
*/
int main(void)
{
  MCUSR=0;
  MCUCR=0;
  // I/O directions
  DDRC=0x3f;
  DDRD=0xfb;
  DDRB=0x2f;
  // initial state
  PORTC=0x3f;
  PORTD=0xff;
  PORTB=0x3f;
  //
  EICRA=2; // falling edge on INT0 interrupts
  enable_button(); // enable INT0 interrupts
  //
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  // configure watchdog to interrupt&reset, 4 sec timeout
  WDTCSR|=0x18;
  WDTCSR=0xe8;
  find_end();
  sei();
  while (1) {
    sleep_cpu(); // any interrupt, including watchdog, wakes up
    wdt_reset();
    WDTCSR|=0x40;
  }
}
