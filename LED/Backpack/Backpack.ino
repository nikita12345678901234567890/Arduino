#include <Adafruit_NeoPixel.h>


#define PIN 6
#define N_LEDS 102

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);
const int rightButtonPin = 8;
const int leftButtonPin = 11;
const int hazardButtonPin = 13;
const int brakeButtonPin = 12;
const int threeButtonPin = 9;
const int twoButtonPin = 10;
const int oneButtonPin = 7;

int rightButtonState = 0;
int leftButtonState = 0;
int hazardButtonState = 0;
int brakeButtonState = 0;
int threeButtonState = 0;
int twoButtonState = 0;
int oneButtonState = 0;

int prevState = 0;
bool hazardOn = false;
void setup()
{
  //Button:
  pinMode(rightButtonPin, INPUT);
  pinMode(leftButtonPin, INPUT);
  pinMode(hazardButtonPin, INPUT);
  pinMode(brakeButtonPin, INPUT);
  pinMode(threeButtonPin, INPUT);
  pinMode(twoButtonPin, INPUT);
  pinMode(oneButtonPin, INPUT);
  //Led:
  strip.begin();
  strip.show();
}

void loop()
{
 rightButtonState = digitalRead(rightButtonPin);
 leftButtonState = digitalRead(leftButtonPin);
 hazardButtonState = digitalRead(hazardButtonPin);
 brakeButtonState = digitalRead(brakeButtonPin);
 threeButtonState = digitalRead(threeButtonPin);
 twoButtonState = digitalRead(twoButtonPin);
 oneButtonState = digitalRead(oneButtonPin);
   if(hazardButtonState == HIGH && prevState == LOW)
   {
     prevState = HIGH;
   }
   if(hazardButtonState == LOW && prevState == HIGH)
   {
     prevState = LOW;
     hazardOn = !hazardOn;
   }
   if(hazardOn)
   {
     // Turning off All LEDs
     allLedOff();
     delay(250);
     //setting LEDs to orange
     for(int count = 0; count < N_LEDS; count++)
     {
       strip.setPixelColor(count,255,100,0);
       strip.show();
     }
     delay(250);
   }
  else if(rightButtonState == HIGH)
   {
     //  Turning ON  Right LEDs with orange color
     //strip.show();
     for(int count = N_LEDS / 2; count < N_LEDS; count++)
     {
       strip.setPixelColor(count,255,100,0);
     }
     strip.show();
     delay(250);
     // Turning off All LEDs
     allLedOff();
     delay(250);
   }
   else if(leftButtonState == HIGH)
   {
     //  Turning ON  Left LEDs with orange color
     for(int count = 0; count < N_LEDS / 2; count++)
     {
       strip.setPixelColor(count,255,100,0);
     }
     strip.show();
     delay(250);
     // Turning off All LEDs
     allLedOff();
     delay(250);
   }
   else if(brakeButtonState == HIGH)
   {
     for(int count = 0; count < N_LEDS; count++)
     {
       strip.setPixelColor(count,255,0,0);
       strip.show();
     }
   }
   else if(threeButtonState == HIGH)
   {
     rainbowCycle(5);
   }
   else if(twoButtonState == HIGH)
   {
     allLedOff();
     chase(255, 0, 0);
     chaseBack(0, 255, 0);
     chase(0, 0, 255);
     chaseBack(255, 255, 255);
   }
   else if(oneButtonState == HIGH)
   {
     NewKITT(0xff, 0, 0, 4, 30, 50);
   }
   else
     //  Police Lights
    {
      // Turning off All LEDs
     strip.clear();
     for(int count = 0; count < N_LEDS/2 - 3; count++)
     {
       strip.setPixelColor(count,255,0,0);
       strip.setPixelColor(N_LEDS - count,0,0,0);
       strip.show();
     }
     delay(250);
     for(int count = 0; count < N_LEDS/2 - 3; count++)
     {
       strip.setPixelColor(N_LEDS - count,0,0,255);
       strip.setPixelColor(count,0,0,0);
       strip.show();
     }
     delay(250);
    }
}


void allLedOff()
{
       // Turning off All LEDs
     for(int count = 0; count < N_LEDS; count++)
     {
       strip.setPixelColor(count,0,0,0);
       strip.show();
     }
}

static void chase(int R, int G, int B)
{
  for (int i = 0; i < N_LEDS + 4; i++)
  {
    strip.setPixelColor(i  , R, G, B);
    strip.setPixelColor(i - 4, 0);
    strip.show();
    delay(30);
  }
}


static void chaseBack(int R, int G, int B)
{
  for (int i = N_LEDS; i > 4; i--)
  {
    strip.setPixelColor(i  , R, G, B);
    strip.setPixelColor(i + 4, 0);
    strip.show();
    delay(30);
  }
}






void rainbowCycle(int SpeedDelay) {
  byte *c;
  uint16_t i, j;

  for(j=0; j<256; j++) { // 1 cycle of all colors on wheel
    for(i=0; i< N_LEDS; i++) {
      c=Wheel(((i * 256 / N_LEDS) + j) & 255);
      strip.setPixelColor(i, *c, *(c+1), *(c+2));
    }
    strip.show();
    delay(SpeedDelay);
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}






void NewKITT(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){
  RightToLeft(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  LeftToRight(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  OutsideToCenter(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  CenterToOutside(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  LeftToRight(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  RightToLeft(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  OutsideToCenter(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  CenterToOutside(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
}

void CenterToOutside(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i =((N_LEDS-EyeSize)/2); i>=0; i--) {
    strip.clear();
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    
    strip.setPixelColor(N_LEDS-i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(N_LEDS-i-j, red, green, blue); 
    }
    strip.setPixelColor(N_LEDS-i-EyeSize-1, red/10, green/10, blue/10);
    
    strip.show();
    
    
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void OutsideToCenter(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = 0; i<=((N_LEDS-EyeSize)/2); i++) {
    strip.clear();
    
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    
    strip.setPixelColor(N_LEDS-i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(N_LEDS-i-j, red, green, blue); 
    }
    strip.setPixelColor(N_LEDS-i-EyeSize-1, red/10, green/10, blue/10);
    
    strip.show();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void LeftToRight(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = 0; i < N_LEDS-EyeSize-2; i++) {
    strip.clear();
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void RightToLeft(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = N_LEDS-EyeSize-2; i > 0; i--) {
    strip.clear();
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}
