#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (RGB+W NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Example for NeoPixel Shield.  In this application we'd like to use it
// as a 5x8 tall matrix, with the USB port positioned at the top of the
// Arduino.  When held that way, the first pixel is at the top right, and
// lines are arranged in columns, progressive order.  The shield uses
// 800 KHz (v2) pixels that expect GRB color data.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_BOTTOM     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);


void setup()
{
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(50);
  matrix.setTextColor(matrix.Color(255, 0, 0));
}

//Standard colors:
uint16_t red = matrix.Color(255,0,0);
uint16_t orange = matrix.Color(255, 100, 0);
uint16_t green = matrix.Color(0,255,0);
uint16_t blue = matrix.Color(0,0,255);
uint16_t white = matrix.Color(255, 255, 255);
uint16_t off = matrix.Color(0, 0, 0);

//Special colors:
uint16_t DarkGreen = matrix.Color(27, 94, 31);

void loop()
{
  //Hello GMR:
  for(int i = 32; i > -60; i--)
  {
    Print("Hello GMR!", red, i, 0, 0);
    delay(35);
    matrix.clear();
  }

  //Don't mind me:
  for(int i = 32; i > -100; i--)
  {
    Print("Don't mind me.", red, i, 0, 0);
    delay(35);
    matrix.clear();
  }

  //I am a totally normall human:
  for(int i = 32; i > -200; i--)
  {
    Print("I am a totally normal human.", red, i, 0, 0);
    delay(35);
    matrix.clear();
  }

  //There is nothing special about this hat:
  for(int i = 32; i > -250; i--)
  {
    Print("There is nothing special about this hat.", red, i, 0, 0);
    delay(35);
    matrix.clear();
  }  
}

void Print(String text, uint16_t color, int x, int y, int Size)
{
  matrix.setTextSize(Size);
  matrix.setCursor(x, y);
  matrix.setTextColor(color);
  for(int i = 0; i < text.length(); i++)
  {
    matrix.print(text[i]);
  }
  matrix.show();
}

void Creeper(int x, int y)
{
  //Background:
  matrix.fillRect(x, y, 8, 8, DarkGreen);

  //Left eye:
  matrix.fillRect(x + 1, y + 2, 2, 2, off);

  //Right eye:
  matrix.fillRect(x + 5, y + 2, 2, 2, off);

  //Else:
  matrix.fillRect(x + 3, y + 4, 2, 1, off);
  matrix.fillRect(x + 2, y + 5, 4, 2, off);
  matrix.drawPixel(x + 2, y + 7, off);
  matrix.drawPixel(x + 5, y + 7, off);

  matrix.show();
}