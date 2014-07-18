/*
 */
 
#include <SPI.h>

static const int led = SS;
static const uint8_t cs_pin = 10;

#define BOARD_X 1
#define BOARD_Y 1

#define CYCLE_SEC 5

#define STEP_MS ((CYCLE_SEC) * 1000 / 256)

void setup()
{
  Serial.begin(9600);
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, 1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();
}

static void
send_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t d2)
{
  digitalWrite(cs_pin, 0);
  SPI.transfer(cmd);
  SPI.transfer(d0);
  SPI.transfer(d1);
  SPI.transfer(d2);
  digitalWrite(cs_pin, 1);
}

uint8_t array[64 * BOARD_X * BOARD_Y];

static void
do_pixel(uint16_t pos)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t val;

  val = array[pos];
  array[pos] = val + 1;
  if (val < 85) {
      r = val * 3;
      g = 255 - r;
      b = 0;
  } else if (val < 170) {
      b = (val - 85) * 3;
      r = 255 - b;
      g = 0;
  } else {
      g = (val - 170) * 3;
      b = 255 - g;
      r = 0;
  }
  send_cmd(pos & 0x3f, r, g, b);
}

static uint8_t
plasma_pixel_init(uint8_t x, uint8_t y)
{
  float xf = ((x + 0.0f) * (M_PI / 8.0f)) * (8/6.0f);
  float yf = ((y + 0.0f) * (M_PI / 8.0f)) * (8/6.0f);
  float u;
  float v;

  u = cos(xf) + 1.0f;
  v = cos(yf) + 1.0f;
  return (uint8_t)((u + v) * (255.9f/4.0f));
}

static void
plasma_init(void)
{
  uint8_t x;
  uint8_t y;
  uint8_t bx;
  uint8_t by;
  uint16_t pos;
  pos = 0;
  for (by = 0; by < BOARD_Y; by++) {
      for (bx = 0; bx < BOARD_X; bx++) {
	  for (y = 0; y < 8; y++) {
	      for (x = 0; x < 8; x++) {
		  array[pos] = plasma_pixel_init(x + bx * 8, y + by * 8);
		  pos++;
	      }
	  }
      }
  }
}

static void
do_plasma(void)
{
  uint16_t pos;
  uint8_t board;
  uint8_t i;
  long last;
  long now;
  plasma_init();
  last = millis();
  while (true) {
      pos = 0;
      for (board = 0; board < BOARD_X * BOARD_Y; board++) {
	  send_cmd(0xe1, board, 0x00, 0x00);
	  //send_cmd(0xe1, 0xff, 0x00, 0x00);
	  for (i = 0; i < 64; i ++) {
	      do_pixel(pos);
	      pos++;
	  }
      }
      do {
	  now = millis();
      } while (now - last < STEP_MS);
      last = now;
  }
}

void loop()
{
  send_cmd(0xff, 0xff, 0xff, 0xff);
  send_cmd(0xe0, 0xf0, 0xf1, 0xf2);
  send_cmd(0xe1, 0xff, 0x00, 0x00);

  do_plasma();
}

