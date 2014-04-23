//#define F_CPU 128000
#include <stdint.h>
register uint16_t r_cathode_mask asm("r10");
register uint8_t r_pos asm("r12");
register uint8_t r_skip asm("r13");
register uint8_t r_tmp1 asm("r9");
register uint8_t r_tmp2 asm("r8");

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#define NUM_PIXELS (8*8*3)

// The framebuffer stores raw (4-bit) brightness values, so we can handle SPI data quickly
static uint8_t framebuffer[16*16];
// The map array stores actual pin values, to quickly update the drive pins
// We populate this from the framebuffer when we have nothing better to do
static volatile uint16_t map[16 * 16] __attribute__((aligned(256)));

#define PIXEL(a, k) ((a) | ((k) << 4))

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
  TCCR0A = 0;
  // Prescale /256 
  //TCCR0B = _BV(CS02);
  // Prescale /64
  TCCR0B = _BV(CS01) | _BV(CS00);
}

static const uint16_t mask_lookup[16] = 
{
  0x0001, 0x0002, 0x0004, 0x0008,
  0x0010, 0x0020, 0x0040, 0x0080,
  0x0100, 0x0200, 0x0400, 0x0800,
  0x1000, 0x2000, 0x4000, 0x8000,
};

static bool active;

static void
expand_pixel(void)
{
  static uint8_t pos;
  static uint8_t n1;
  static uint8_t n0;
  static uint16_t mask;
  static volatile uint16_t *p;
  uint8_t val;

  if (n1) {
      *(p++) |= mask;
      n1--;
      return;
  }
  if (n0) {
      *(p++) &= ~mask;
      n0--;
      return;
  }
  val = framebuffer[pos];
  mask = mask_lookup[pos & 0xf];
  p = &map[pos & ~0xf];
  pos++;
  n1 = val;
  n0 = 16 - val;
}

static void
update_display(void)
{
  static uint8_t pos;
  static uint16_t cathode_mask;
  static uint8_t skip;
  uint16_t val;

  if ((pos & 0xf) == 0) {
      if (skip > 0) {
	  skip--;
	  return;
      }
      skip = 16;
      cathode_mask <<= 1;
      if (cathode_mask == 0) {
	  cathode_mask = 1;
	  pos = 0;
      }
      // Last row should always be off.
      // Tristate eveything except the cathode.
      DDRB = ((cathode_mask >> PB_SHIFT) & PB_MASK) | PB_DDR;
      DDRC = ((cathode_mask >> PC_SHIFT) & PC_MASK) | PC_DDR;
      DDRD = ((cathode_mask >> PD_SHIFT) & PD_MASK) | PD_DDR;
  }
  // Drive selected anodes high
  val = map[pos];
  pos++;
  PORTB = ((val >> PB_SHIFT) & PB_MASK) | PB_PORT;
  PORTC = ((val >> PC_SHIFT) & PC_MASK) | PC_PORT;
  PORTD = ((val >> PD_SHIFT) & PD_MASK) | PD_PORT;
  val |= cathode_mask;
  DDRB = ((val >> PB_SHIFT) & PB_MASK) | PB_DDR;
  DDRC = ((val >> PC_SHIFT) & PC_MASK) | PC_DDR;
  DDRD = ((val >> PD_SHIFT) & PD_MASK) | PD_DDR;
}

// Two most significant bits only
static uint8_t my_address;

static uint8_t current_pixel;

static void
do_pixel(uint8_t val)
{
  uint8_t pos;
  pos = pgm_read_byte(&pixel_map[current_pixel]);
  current_pixel++;
  framebuffer[pos] = val;
}

static void
do_data(void)
{
  static uint8_t data1;
  static bool selected;
  static bool more;
  uint8_t data2;

  if (!more) {
      data1 = SPDR;
      more = true;
      return;
  }
  more = false;
  data2 = SPDR;
  if (data1 == 0xff) {
      active = false;
      if (data2 == 0xff) {
	  more = true;
      } else if (data2 == 0xa5) {
	  active = true;
      }
      return;
  }
  if (!active)
    return;
  if ((data1 >> 6) == 0) {
      /* Pixel data.  */
      if (selected && current_pixel < NUM_PIXELS) {
	  do_pixel(data1 & 0xf);
	  do_pixel(data2 >> 4);
	  do_pixel(data2 & 0xf);
      }
  } else if ((data1 >> 6) == 1) {
      selected = ((data1 & 0x3f) == my_address);
      /* Set position.  */
      current_pixel = (int)data2 * 3;
  } else {
      /* Unknown command.  */
  }
}

// r10/r11 = cathode_mask
// r12=pos
// r13=skip
ISR(TIMER0_OVF_vect, ISR_NAKED)
{
  asm(
"\n	mov r9,r30"
"\n	in r8,__SREG__"
"\n	push r25"
"\n	push r31"
"\n	mov r30,r12"
"\n	andi r30,lo8(15)"
"\n	breq .La8"
"\n.La3:"
"\n	mov r30,r12"
"\n	ldi r31,hi8(map)"
"\n	lsl r30"
"\n	rol r31"
"\n	ld r25,Z+"
"\n	ld r31,Z"
"\n	out 0xb,r25" // PORTD
"\n	inc r12"
"\n	mov r30,r31"
"\n	andi r30,lo8(3)"
"\n	ori r30,lo8(60)"
"\n	out 0x5,r30" // PORTB
"\n	mov r30,r31"
"\n	lsr r30"
"\n	lsr r30"
"\n	out 0x8,r30" // PORTC
"\n	or r25,r10"
"\n	or r31,r11"
"\n	out 0xa,r25" //DDRC
"\n	mov r25,r31"
"\n	andi r25,lo8(3)"
"\n	out 0x4,r25" // DDRB
"\n	lsr r31"
"\n	lsr r31"
"\n	out 0x7,r31" // DDRC
"\n.La2:"
"\n/* epilogue start */"
"\n	pop r31"
"\n	pop r25"
"\n	out __SREG__,r8"
"\n	mov r30,r9"
"\n	reti"
// Next cathode
"\n.La8:"
"\n	tst r13"
"\n	breq .La4"
"\n	dec r13"
"\n	rjmp .La2"
"\n.La4:"
"\n	ldi r30,lo8(16)"
"\n	mov r13,r30"
"\n	lsl r10"
"\n	rol r11"
"\n	brcc .La7"
"\n	inc r10"
"\n	clr r12"
"\n.La7:"
// Tristate evetryhing to avoid glitches
"\n	clr r25"
"\n	out 0x4,r25" // DDRB
"\n	out 0x7,r25" // DDRC
"\n	out 0xa,r25" // DDRD
"\n	rjmp .La3"
);
}

int
main()
{
  uint8_t t;

  r_cathode_mask = 0x8000;
  r_pos = 0;
  r_skip = 0;

  init_spi();
  init_timer();

  t = TCNT0;
  while (true) {
      while ((int8_t)(t - TCNT0) > 0) {
	if (SPSR & _BV(SPIF)) {
	  do_data();
	} else {
	  expand_pixel();
	}
      }
      t += 2;
      while ((int8_t)(t - TCNT0) > 0);
      update_display();
      t += 2;
  }
  return 0;
}
