#define F_CPU 16000000ul

#include <stdint.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>

#define FIFO_SIZE 64

#define NUM_FRAMES 2

#define NUM_PIXELS (8*8*3)

volatile uint8_t framebuffer[16 * 16 * NUM_FRAMES];

static uint8_t write_frame;
static volatile uint8_t *write_framebuffer;

static uint8_t display_frame;
static volatile uint8_t *display_framebuffer;

volatile uint8_t next_anode;

volatile uint8_t ticks;

#define FIFO_MASK (FIFO_SIZE - 1)
#if (FIFO_SIZE & FIFO_MASK) != 0
#error
#endif
volatile uint8_t fifo[FIFO_SIZE];

volatile uint8_t fifo_head;

#define SET_XLAT() PORTD |= _BV(5)
#define CLEAR_XLAT() PORTD &= ~_BV(5)

#define SET_BLANK() PORTD |= _BV(7)
#define CLEAR_BLANK() PORTD &= ~_BV(7)

#define SET_VPRG() PORTD |= _BV(3)
#define CLEAR_VPRG() PORTD &= ~_BV(3)

#define SET_DCPRG() PORTB |= _BV(0);
#define CLEAR_DCPRG() PORTB &= ~_BV(0);

// This table translates from an 8x8x3 grid to the
// 16x16 anode/cathode matrix used by the framebuffer
const uint8_t pixel_map[NUM_PIXELS] PROGMEM = {
#define PIXEL(a, k) ((15 - (k)) | ((a) << 4))
#include "board.h"
};

// enable pullups on unused pins
static void
init_unused(void)
{
  // Not connected
  PORTB |= _BV(1);
  PORTD |= _BV(2);
}

static void
init_i2c(void)
{
  // We do not currently use I2C.
  // However we do use the SDA pin for address programming
  // Enable pullup and SDA and SCL
  PORTC |= _BV(4) | _BV(5);
}

static void
init_spi_slave(void)
{
  // Mode3, slave mode, MSB first, enable interrupt
  SPCR = _BV(SPIE) | _BV(SPE) | _BV(CPOL) | _BV(CPHA);

  // Enable pullups on SS, MOSI, MISO, SCK
  PORTB |= _BV(2) | _BV(3) | _BV(4) | _BV(5);
}

static void
init_spi_master(void)
{
  // Configure UART in SPI master mode, clk/2, mode0, MSB first
  UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);
  UBRR0 = 0;
  UCSR0B = _BV(TXEN0);

  // Enable clock and data pins
  DDRD |= _BV(1) | _BV(4);
  // Enable pullup on input
  PORTD |= _BV(0);
}

// 2kHz refresh timer
static void
init_timer(void)
{
  // CTC mode
  TCNT1 = 0;
  OCR1A = 8000; // 8000 cycles = 500us = 2Khz
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  // Enable OCA interrupt
  TIMSK1 = _BV(OCIE1A);
}

static void
init_gsclk(void)
{
  // CTC mode, toggle OC0A
  TCCR0A = _BV(COM0A0) | _BV(WGM01);
  TCCR0B = 0;
  TCNT0 = 0;
  // 570kHz output clock.  Combined with a 2kHz row mux this gives a maximum
  // PWM value of 0xff, plus a bit of extra delay for latching in new data.
  OCR0A = 14;
  // Enable OC0A output
  DDRD |= _BV(6);
}

static inline void
enable_gsclk(void)
{
  TCCR0B = _BV(CS00);
}

static inline void
disable_gsclk(void)
{
  TCCR0B = 0;
  TCNT0 = 0;
  // Make sure output line is low
  _NOP();
  if (PINB & _BV(6)) {
      TCCR0B |= _BV(FOC0A);
  }
}

static uint8_t eeprom_address EEMEM;

static uint8_t my_address;

static inline void
set_pixel(uint8_t n, uint8_t val)
{
  uint8_t pos;
  pos = pgm_read_byte(&pixel_map[n]);
  write_framebuffer[pos] = val;
}

static uint8_t
get_pixel(uint8_t n)
{
  uint8_t pos;
  pos = pgm_read_byte(&pixel_map[n]);
  return write_framebuffer[pos];
}

static volatile uint8_t fb_offset;
static volatile bool sending_frame;
static volatile uint8_t dc_bytes;
static volatile bool dc_changed;

static uint8_t dc[12];

bool done_tick;

static void
rotate_pixel(uint8_t n)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  r = get_pixel(n);
  g = get_pixel(n + 1);
  b = get_pixel(n + 2);
  if (r == 0 && g != 0) {
      set_pixel(n + 1, g - 1);
      set_pixel(n + 2, b + 1);
  } else if (g == 0 && b != 0) {
      set_pixel(n + 2, b - 1);
      set_pixel(n, r + 1);
  } else {
      set_pixel(n, r - 1);
      set_pixel(n + 1, g + 1);
  }
}

static void
plasma_pixel_init(uint8_t x, uint8_t y, int n)
{
  uint8_t pos;
  pos = (x + y * 8) * 3;
  set_pixel(pos, 0xff);
  while (n) {
      rotate_pixel(pos);
      n--;
  }
}

static void
plasma_init(void)
{
  uint8_t x;
  uint8_t y;
  int n;
  for (x = 0; x < 4; x++) {
      for (y = 0; y < 4; y++) {
#if 0
	  n = sqrt(x*x + y * y) * 100.0;
#else
	  if (x > y)
	    n = x * 2 + y;
	  else
	    n = y * 2 + x;
	  n *= 50;
#endif
	  n = 700-n;
	  plasma_pixel_init(4 + x, 4 + y, n);
	  plasma_pixel_init(3 - x, 4 + y, n);
	  plasma_pixel_init(4 + x, 3 - y, n);
	  plasma_pixel_init(3 - x, 3 - y, n);
      }
  }
}

static void
demo_tick(void)
{
  static bool done_init;
  int i;

  if (!done_init) {
      plasma_init();
      done_init = true;
  }
  for (i = 0; i < 8*8*3; i += 3) {
      rotate_pixel(i);
  }
}

#if 0
static void
demo_tick(void)
{
  static int n;

  //do_pixel(4*24+0*3+1, 0x20);
#if 1
  set_pixel(n, 0);
  n += 3;
  if (n >= 8*8*3) {
      n = (n - 8*8*3) + 1;
      if (n == 3)
	n = 0;
  }
  set_pixel(n, 0xff);
#endif
}
#endif

static void
send_data(void)
{
  uint8_t tmp;

  if ((UCSR0A & _BV(UDRE0)) == 0)
    return;

  //  All data is send MSB first
  if (dc_bytes) {
      // Dot correction data
      dc_bytes--;
      UDR0 = dc[dc_bytes];
      if (dc_bytes == 0) {
	  // Wait for the transmit to complete
	  _delay_us(20);
	  // Latch data into register
	  SET_XLAT();
	  CLEAR_XLAT();
	  SET_DCPRG();
	  CLEAR_VPRG();
      }
  } else if (sending_frame) {
      // PWM data
      // The count registers are 12 bits, so a pair of values expand to 
      // 3 bytes of data.  We can not submit all this immediately, but the USART
      // double buffering means we probably only stall for 16 cycles and it is
      // not worth trying to be clever.
      tmp = display_framebuffer[fb_offset++];
      UDR0 = tmp >> 4;
      tmp <<= 4;
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = tmp;
      tmp = display_framebuffer[fb_offset++];
      if ((fb_offset & 0xf) == 0) {
	  _delay_us(2);
	  sending_frame = false;
	  display_framebuffer = &framebuffer[(uint16_t)display_frame * 16 * 16];
      }
      while ((UCSR0A & _BV(UDRE0)) == 0)
	/* no-op */;
      UDR0 = tmp;
  } else {
      if ((ticks & 0x0f) == 0) {
	  if (!done_tick) {
	      demo_tick();
	      done_tick = true;
	  }
      } else {
	done_tick = false;
      }
  }
}

static void
set_address(uint8_t address)
{
  if (address == 0xff)
    return;
  my_address = address;
  eeprom_update_byte(&eeprom_address, address);
}

static void
set_dc(uint8_t red, uint8_t green, uint8_t blue)
{
  /* Dot correction data is only 6 bits.  */
  red >>= 2;
  green >>= 2;
  blue >>= 2;
  /* Pack data ready for transmission.  */
  dc[0] = red | (green << 6);
  dc[1] = (green >> 2) | (blue << 4);
  dc[2] = (blue >> 4) | (red << 2);
  dc[3] = green | (blue << 6);
  dc[4] = (blue >> 2) | (red << 4);
  dc[5] = (red >> 4) | (green << 2);
  dc[6] = blue | (red << 6);
  dc[7] = (red >> 2) | (green << 4);
  dc[8] = (green >> 4) | (blue << 2);
  dc[9] = red | (green << 6);
  dc[10] = (green >> 2) | (blue << 4);
  dc[11] = (blue >> 4); // last channel unused
  dc_changed = true;
}

static void
do_data(void)
{
  uint8_t n;
  uint8_t cmd;
  uint8_t d0;
  uint8_t d1;
  uint8_t d2;
  uint8_t new_address;
  uint8_t latched_address;
  bool active;
  uint8_t fifo_tail;
  bool selected;

  fifo_tail = 0;
  active = false;
  while (true) {
      n = (fifo_head - fifo_tail) & FIFO_MASK;
      if (n < 4) {
	  send_data();
	  continue;
      }
      if (!active) {
	selected = false;
	latched_address = 0xff;
      }
      cmd = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      if (cmd == 0xff) {
	  active = false;
	  continue;
      }
      d0 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      d1 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      d2 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & FIFO_MASK;
      if (cmd == 0xe0) {
	  // Initialize
	  if (d0 == 0xf0 && d1 == 0xf1 && d2 == 0xf2) {
	      latched_address = new_address;
	      active = true;
	  } else {
	      active = false;
	  }
	  continue;
      }
      new_address = 0xff;
      if (!active) {
	  continue;
      }
      if (cmd < NUM_PIXELS) {
	  if (selected) {
	      n = cmd * 3;
	      set_pixel(n, d0);
	      set_pixel(n + 1, d1);
	      set_pixel(n + 2, d2);
	  }
      } else if (cmd == 0x80) {
	  /* Board select */
	  if (d0 == my_address || d0 == 0xff)
	    selected = true;
	  else
	    selected = false;
      } else if (cmd == 0x81) {
	  /* Set framebuffer (page flip).  */
	  if (d0 == my_address || d0 == 0xff) {
	      display_frame = d1 % NUM_FRAMES;
	      write_frame = d2 % NUM_FRAMES;
	      write_framebuffer = &framebuffer[(uint16_t)write_frame * 16 * 16];
	  }
      } else if (cmd == 0x82) {
	  /* Set brightness */
	  if (selected) {
	      set_dc(d0, d1, d2);
	  }
      } else if (cmd == 0xe1) {
	  /* Set address.  */
	  if ((d0 == 0xf5 || d0 == 0xfa) && d1 >= 0xf0 && d2 >= 0xf0) {
	      new_address = (d1 << 4) | (d2 & 0x0f);
	      if (cmd == 0xfa && ~new_address == latched_address) {
		  set_address(latched_address);
		  new_address = 0xff;
	      }
	  }
	  active = false;
      } else {
	  /* Unknown command.  */
	  active = false;
      }
  }
}

ISR(SPI_STC_vect)
{
  fifo[fifo_head] = SPDR;
  fifo_head = (fifo_head + 1) & FIFO_MASK;
}

ISR(TIMER1_COMPA_vect)
{
  static uint8_t overload;
  ticks++;
  // Enable interrupts so we are not blocking the SPI slave interrupt.
  // The clock period is long enough that we don't have to worry about
  // nested timer interrupts
  sei();
  if (sending_frame)
    overload = 0xff;

  disable_gsclk();
  SET_BLANK();
  if (overload) {
      overload--;
      return;
  }
  if (next_anode != 0xff && !overload) {
      // Latch data into the register
      SET_XLAT();
      CLEAR_XLAT();
      // Select the next anode
      // The 8 outputs of each decoder are connected in reverse order
      PORTC = (PORTC & ~0xf) | (next_anode ^ 7);
      // Delay a few us for everything to settle.
      _delay_us(5);
  }
  next_anode = (next_anode + 1) & 0xf;
  // Shift in the next set of anode data
  fb_offset = next_anode * 16;
  sending_frame = true;
  // And dot correction data if needed
  if (dc_changed) {
      dc_bytes = 12;
      SET_VPRG();
      dc_changed = false;
  }
  // Triggering the greyscale clock is timing critical, so disable interrupts
  cli();
  // Trigger the output pulse
  CLEAR_BLANK();
  enable_gsclk();
}

static void
init_5940(void)
{
  init_gsclk();
  init_spi_slave();
  // Setup VPRG, XLAT, BLANK
  DDRD |= _BV(3) | _BV(5) | _BV(7);
  SET_BLANK();
  CLEAR_XLAT();
  CLEAR_VPRG();
  // Setup DCPRG
  DDRB |= _BV(0);
  CLEAR_DCPRG();
}

static void
init_eeprom()
{
  my_address = eeprom_read_byte(&eeprom_address);
  if (my_address == 0xff)
    my_address = 0;
}

static void
init_anodes(void)
{
  /* Anode select pins.  */
  DDRC |= 0xf;
  PORTC |= 0xf;
}

int
main()
{
  /* Do this early to minimize amount of time pins are left floating.  */
  SET_BLANK();
  init_anodes();

  fifo_head = 0;
  next_anode = 0xff;
  sending_frame = false;
  write_framebuffer = &framebuffer[0];
  display_framebuffer = &framebuffer[0];

  set_dc(0x40,0x80,0x60);

  init_eeprom();
  init_unused();
  init_i2c();
  init_5940();
  init_spi_master();
  init_timer();

  sei();
  do_data();
  return 0;
}
