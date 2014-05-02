//#define F_CPU 128000
#include <stdint.h>
register uint8_t r_fifo_head asm("r6");
register uint8_t r_map_base asm("r7");
register uint16_t r_cathode_mask asm("r10");
register uint8_t r_pos asm("r12");
register uint8_t r_skip asm("r13");
register uint8_t r_tmp1 asm("r9");
register uint8_t r_tmp2 asm("r8");
register uint8_t r_tmp3 asm("r16");
register uint8_t r_tmp4 asm("r17");

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#define NUM_FRAMES 2

#define NUM_PIXELS (8*8*3)

// The framebuffer stores raw (4-bit) brightness values, so we can handle SPI data quickly
// The map array stores actual pin values, to quickly update the drive pins
// We populate this from the framebuffer when we have nothing better to do
volatile uint16_t map[16 * 16 * NUM_FRAMES] __attribute__((aligned(256))) __attribute__((section(".data")));

static uint8_t framebuffer[16*16 * NUM_FRAMES];

static uint8_t read_frame;
static uint8_t *read_framebuffer;
static volatile uint16_t *expand_map_base;
static uint8_t write_frame;
static uint8_t *write_framebuffer;

static uint8_t display_frame;
volatile uint8_t display_map_base;

volatile uint8_t fifo[128] __attribute__((aligned(128))) __attribute__((section(".data.fifo")));


#define PIXEL(a, k) ((a) | ((k) << 4))

// This table translates from an 8x8 RGB matrix to the
// 16x16 anode/cathode matrix used by the framebuffer
const uint8_t pixel_map[NUM_PIXELS] PROGMEM = {
#include "board.h"
};

#define PD_SHIFT 0
#define PD_MASK 0xff
#define PD_DDR	0x00
#define PD_PORT	0x00

#define PC_SHIFT 10
#define PC_MASK 0x3f
#define PC_DDR	0x00
#define PC_PORT	0x00

#define PB_SHIFT 8
#define PB_MASK 0x03
#define PB_DDR	0x00
#define PB_PORT	0x3c

static void
init_spi(void)
{
  // Slave mode
  SPCR = _BV(SPE) | _BV(CPOL) | _BV(CPHA);
}

static void
init_timer(void)
{
  // CTC mode at system clock
  TCCR0A = _BV(WGM01);
  TCCR0B = _BV(CS00);
  // 100 cycle period.  Worst-case ISR time is 67 cycles.
  // This also means we can reliably receive SPI data at
  // fosc/16
  OCR0A = 100;
  TIMSK0 = _BV(OCIE0A);
}

static const uint16_t mask_lookup[16] = 
{
  0x0001, 0x0002, 0x0004, 0x0008,
  0x0010, 0x0020, 0x0040, 0x0080,
  0x0100, 0x0200, 0x0400, 0x0800,
  0x1000, 0x2000, 0x4000, 0x8000,
};

static bool active;
static bool changed[NUM_FRAMES];

static void
expand_pixel(void)
{
  static uint8_t pos;
  uint16_t mask;
  uint8_t i;
  volatile uint16_t *p;
  uint8_t val;

  if (pos == 0) {
      if (!changed[display_frame]) {
	  display_map_base = ((uint16_t)&map[0] >> 8) + display_frame * 2;
      }
      read_frame = 1 - read_frame;
      if (!changed[read_frame]) {
	  return;
      }
      changed[read_frame] = false;
      read_framebuffer = &framebuffer[(uint16_t)read_frame * 16 * 16];
      expand_map_base = &map[(uint16_t)read_frame * 16 * 16];
  }
  val = read_framebuffer[pos];
  mask = mask_lookup[pos & 0xf];
  p = &expand_map_base[pos & 0xf0u];
  for (i = 0; i < val; i++) {
      *(p++) |= mask;
  }
  for (; i < 0x10; i++) {
      *(p++) &= ~mask;
  }
  pos++;
}

static uint8_t my_address;

static uint8_t current_pixel;

static void
do_pixel(uint8_t val)
{
  uint8_t pos;
  pos = pgm_read_byte(&pixel_map[current_pixel]);
  current_pixel++;
  write_framebuffer[pos] = val;
  changed[write_frame] = true;
}

static void
do_data(void)
{
  uint8_t data1;
  bool selected;
  uint8_t data2;
  uint8_t fifo_tail;

  fifo_tail = 0;
  active = false;
  selected = false;
  while (true) {
      while (fifo_tail == r_fifo_head)
       	expand_pixel();
      data1 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & 0x3f;
again:
      while (fifo_tail == r_fifo_head)
       	expand_pixel();
      data2 = fifo[fifo_tail];
      fifo_tail = (fifo_tail + 1) & 0x3f;
      if (data1 == 0xff) {
	  active = false;
	  if (data2 == 0xff) {
	      goto again;
	  }
	  if (data2 == 0xf5) {
	      active = true;
	  }
	  continue;
      }
      if (!active)
	continue;
      if ((data1 >> 6) == 0) {
	  /* Pixel data.  */
	  if (selected) {
	     if (current_pixel < NUM_PIXELS) {
		  do_pixel(data1 & 0xf);
		  do_pixel(data2 >> 4);
		  do_pixel(data2 & 0xf);
	      } else {
		  selected = false;
		  active = false;
	      }
	  }
      } else if ((data1 >> 6) == 1) {
	  /* Set cursor position.  */
	  selected = (data2 == my_address);
	  current_pixel = (data1 & 0x3f) * 3;
      } else if (data1 == 0x80) {
	  /* Set framebuffer (page flip).  */
	  if (selected || (data2 & 0x80)) {
	      display_frame = data2 & 1;
	      write_frame = (data2 >> 1) & 1;
	      write_framebuffer = &framebuffer[(uint16_t)write_frame * 16 * 16];
	      selected = false;
	  }
#if 0
      } else if (data1 >> 6 == 3) {
	  FIXME set address
	  FIME read address from eeprom
#endif
      } else {
	  /* Unknown command.  */
	  active = false;
	  selected = false;
      }
  }
}

ISR(TIMER0_COMPA_vect, ISR_NAKED)
{
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  asm(
"\n	mov r9,r30"
"\n	mov r17,r31"
"\n	in r8,__SREG__"
"\n	mov r30,r12"
"\n	andi r30,lo8(15)"
"\n	breq .Lstart_frame" // 7
"\n.Lout_line:" // 14/6
"\n	mov r30,r12"
"\n	clr r31"
"\n	lsl r30"
"\n	rol r31"
"\n	add r31, r7"
"\n	ld r16,Z+"
"\n	ld r31,Z" // 23/15
"\n	out 0xb,r16" // PORTD
"\n	inc r12"
"\n	mov r30,r31"
"\n	andi r30,lo8(3)"
"\n	ori r30,lo8(60)"
"\n	out 0x5,r30" // PORTB
"\n	mov r30,r31"
"\n	lsr r30"
"\n	lsr r30"
"\n	out 0x8,r30" // PORTC
"\n	or r16,r10" // 34/26
"\n	or r31,r11"
"\n	out 0xa,r16" //DDRD
"\n	mov r16,r31"
"\n	andi r16,lo8(3)"
"\n	out 0x4,r16" // DDRB
"\n	lsr r31"
"\n	lsr r31"
"\n	out 0x7,r31" // DDRC
"\n.Ldone:"// 42/34 (12)
"\n	in r16,0x2d" // SPSR
"\n	tst r16"
"\n	brpl .Lfifo_empty"
"\n	in r16,0x2e" // SPDR
"\n	ldi r30,lo8(fifo)"
"\n	ldi r31,hi8(fifo)"
"\n	or r30,r6"
"\n	st Z+,r16"
"\n	andi r30, 0x3f"
"\n	mov r6, r30"
"\n.Lfifo_empty:"// 53/45 (23/16)
"\n	out __SREG__,r8"
"\n	mov r31,r17"
"\n	mov r30,r9"
"\n	reti" // 60/52 + 4 + 3 = 67/59 (30/23 + 7 = 37/30)

"\n.Lstart_frame:" // 7
"\n	tst r13"
"\n	breq .Lfirst_line"
"\n	dec r13"
"\n	brne .Ldone"
"\n	lsl r10" // 12
"\n	rol r11"
"\n	brcc .Lframe_wrap"
"\n	inc r10"
"\n.Lframe_wrap:"
// Tristate everything to avoid glitches
"\n	clr r30"
"\n	out 0x4,r30" // DDRB
"\n	out 0x7,r30" // DDRC
"\n	out 0xa,r30" // DDRD
// Update base pointer
"\n	lds r7, display_map_base"
"\n	rjmp .Ldone" // 24
"\n.Lfirst_line:" // 10
"\n	ldi r30,lo8(16)"
"\n	mov r13,r30"
"\n	rjmp .Lout_line" // 14
  );
#else
#error Unsupported CPU
#endif
}

int
main()
{
  r_cathode_mask = 1;
  r_pos = 0;
  r_skip = 0;
  r_fifo_head = 0;
  display_map_base = ((uint16_t)&map[0]) >> 8;
  r_map_base = display_map_base;
  write_framebuffer = &framebuffer[0];
  /* read_framebuffer and expand_map_base set by expand_pixel.  */

  init_spi();
  init_timer();

  sei();
  do_data();
  return 0;
}
