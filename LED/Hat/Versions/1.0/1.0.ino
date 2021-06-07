//Matrix libraries:
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
//Gesture sensor library:
#include "Melopero_APDS9960.h"
//Clock library
#include "RTClib.h"
//Gyro library:
#include <Wire.h>

//Matrix pin:
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

//Normal matrix:
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_BOTTOM     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);
//Upside down matrix:
Adafruit_NeoMatrix matrix2 = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

//Gesture sensor declaration
Melopero_APDS9960 apds;

//Clock declaration:
RTC_DS3231 rtc;

//Gyro variables:
#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25

int gyroI2CAddr=105;

int gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyroDPS[3];                      // gyro degrees per second, each axis

float heading[3]={0.0f};                // heading[x], heading[y], heading [z]
float quaternion[4]={1.0f,0.0f,0.0f,0.0f};
float euler[3]={0.0f};

int gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.

#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  0         // As recommended

float dpsPerDigit=.00875f;              // for conversion to degrees per second


//Clock variable:
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup()
{
  Serial.begin(9600);
  
  //Matrix stuff
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(50);
  matrix.setTextColor(matrix.Color(255, 0, 0));
  //Same stuff for other matrix
  matrix2.begin();
  matrix2.setTextWrap(false);
  matrix2.setBrightness(50);
  matrix2.setTextColor(matrix.Color(255, 0, 0));

  //Gesture sensor initialization
  int8_t status = NO_ERROR;
  status = apds.init(); // Initialize the comunication library
  if (status != NO_ERROR)
  {
    Serial.println("Error during gesture initialization");
    //while(true);
  }
  status = apds.reset(); // Reset all interrupt settings and power off the device
  if (status != NO_ERROR){
    Serial.println("Error during gesture reset.");
    //while(true);
  }
  else
  {
    Serial.println("Gesture initialized correctly!");
  }

  // Gesture engine settings:
  apds.enableGesturesEngine(); // enable the gesture engine
  apds.setGestureProxEnterThreshold(25); // Enter the gesture engine only when the proximity value 
  // is greater than this value proximity value ranges between 0 and 255 where 0 is far away and 255 is very near.
  apds.setGestureExitThreshold(20); // Exit the gesture engine only when the proximity value is less 
  // than this value.
  apds.setGestureExitPersistence(EXIT_AFTER_1_GESTURE_END); // Exit the gesture engine only when 4
  // consecutive gesture end signals are fired (distance is greater than the threshold)

  apds.wakeUp(); // wake up the device
  

  //Clock initialization:

  if (! rtc.begin())
  {
    Serial.println("Couldn't find RTC");
  }

  if (rtc.lostPower())
  {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

    // When time needs to be re-set on a previously configured device, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //Gyro initialization:
  Wire.begin();
  setupGyro();
  calibrateGyro();
}

//Standard colors:
uint16_t red = matrix.Color(255,0,0);
uint16_t orange = matrix.Color(255, 100, 0);
uint16_t green = matrix.Color(0,255,0);
uint16_t blue = matrix.Color(0,0,255);
uint16_t white = matrix.Color(255, 255, 255);
uint16_t off = matrix.Color(0, 0, 0);

bool flip = true;

//menu stuff:
int menu = 1;
int menuMax = 1;

//String that displays when time is selected:
String Time;

void loop()
{
  matrix.clear();
  matrix2.clear();

  //Clock update:
  DateTime now = rtc.now();

  //Gyro update:
  updateGyroValues();
  updateHeadings();

  Serial.println(quaternion[0]);
  //Flipping display:
  if(quaternion[0] < 0.8)
  {
    flip = false;
  }
  else
  {
    flip = true;
  }

  
  //Gesture sensor update:
  apds.updateGestureStatus();

  if (apds.gestureFifoHasData)
  {
    // Reads the gesture data for the given amount of time and tries to interpret a gesture. 
    // The device tries to detect a gesture by comparing the gesture data values through time. 
    // The device compares the up data with the down data to detect a gesture on the up-down axis and
    // it compares the left data with the right data to detect a gesture on the left right axis.
    //
    // ADVANCED SETTINGS:
    // device.parseGesture(uint parse_millis, uint8_t tolerance = 12, uint8_t der_tolerance = 6, uint8_t confidence = 6);
    //
    // parse_millis: the time in millisecond to read the gesture data and try to interpret a gesture
    //
    // The tolerance parameter determines how much the two values (on the same axis) have to differ to interpret
    // the current dataset as valid for gesture detection (if the values are nearly the same then its not possible to decide the direction 
    // in which the object is moving).
    //
    // The der_tolerance does the same for the derivative of the two curves (the values on one axis through time):
    // this prevents the device from detecting a gesture if the objects surface is not even...
    //
    // The confidence tells us the minimum amount of "detected gesture samples" needed for an axis to tell that a gesture has been detected on that axis:
    // How its used in the source code: if (detected_up_gesture_samples > detected_down_gesture_samples + confidence) gesture_up_down = GESTURE_UP
    apds.parseGesture(300);

    if (apds.parsedLeftRightGesture == LEFT_GESTURE && menu > 0)
    {
        menu--;
    }
    
    else if (apds.parsedLeftRightGesture == RIGHT_GESTURE && menu < menuMax)
    {
        menu++;
    }
  }

  
  switch(menu)
  {
    case 0:
      Time = String("");
      if(now.hour() < 10)
      {
        Time += "0";
      }
      Time += (String)now.hour();
      Time += ":";
      if(now.minute() < 10)
      {
        Time += "0";
      }
      Time += (String)now.minute();
      Print(Time, blue, flip);
      break;
      
    case 1:
      Print("Hello", orange, flip);
      break;
  }
}

void Print(String text, uint16_t color, bool flip)
{
  Print(text, color, 0, 0, 0, flip);
}

void Print(String text, uint16_t color)
{
  Print(text, color, 0, 0, 0, false);
}

void Print(String text, uint16_t color, int x, int y, int Size, bool flip)
{
  if(flip)
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
  else
  {
    matrix2.setTextSize(Size);
    matrix2.setCursor(x, y);
    matrix2.setTextColor(color);
    for(int i = 0; i < text.length(); i++)
    {
      matrix2.print(text[i]);
    }
    matrix2.show();
  }
}

//Gyro functions:

float safe_asin(float v)
{
        if (isnan(v)) {
                return 0;
        }
        if (v >= 1.0) {
                return PI/2;
        }
        if (v <= -1.0) {
                return -PI/2;
        }
        return asin(v);
}

void updateHeadings()
{
   
  float deltaT=getDeltaTMicros();

  double gyroDPSDelta[3];

  for (int j=0;j<3;j++)
    {
      gyroDPSDelta[j]=(gyroDPS[j]*deltaT)/1000000.0;
      heading[j] -= gyroDPSDelta[j];
    }
   
  // get radians per second
  // z is yaw
  // y is pitch
  // x is bank
  double rps[3]={0.0};
  for (int j=0;j<3;j++)
    rps[j]=(gyroDPSDelta[j]/57.2957795);
 
  // convert the radians per second to a quaternion
  double c1 = cos(rps[0]/2);
  double s1 = sin(rps[0]/2);
  double c2 = cos(rps[1]/2);
  double s2 = sin(rps[1]/2);
  double c3 = cos(rps[2]/2);
  double s3 = sin(rps[2]/2);
  double c1c2 = c1*c2;
  double s1s2 = s1*s2;
  double quatDelta[4];
  quatDelta[0] =c1c2*c3 - s1s2*s3;
  quatDelta[1] =c1c2*s3 + s1s2*c3;
  quatDelta[2] =s1*c2*c3 + c1*s2*s3;
  quatDelta[3] =c1*s2*c3 - s1*c2*s3;
 
  // now we just multiply quaternion by quatDelta
  float newQuat[4];
  newQuat[0]=(quaternion[0]*quatDelta[0] - quaternion[1]*quatDelta[1] - quaternion[2]*quatDelta[2] - quaternion[3]*quatDelta[3]);
  newQuat[1]=(quaternion[0]*quatDelta[1] + quaternion[1]*quatDelta[0] + quaternion[2]*quatDelta[3] - quaternion[3]*quatDelta[2]);
  newQuat[2]=(quaternion[0]*quatDelta[2] - quaternion[1]*quatDelta[3] + quaternion[2]*quatDelta[0] + quaternion[3]*quatDelta[1]);
  newQuat[3]=(quaternion[0]*quatDelta[3] + quaternion[1]*quatDelta[2] - quaternion[2]*quatDelta[1] + quaternion[3]*quatDelta[0]);
  for (int j=0;j<4;j++)
    quaternion[j]=newQuat[j];
   
  // Normalize quaternion
  float magnitude=sqrt(quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
  if (magnitude > 50.0)
  {
    Serial.println("Normalizing");
    for (int j=0;j<4;j++)
      quaternion[j]=quaternion[j]/magnitude;
  }
 
  // Convert back to euler angles
  euler[2]=atan2(2.0*(quaternion[0]*quaternion[1]+quaternion[2]*quaternion[3]), 1-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]));
  euler[1]=safe_asin(2.0*(quaternion[0]*quaternion[2] - quaternion[3]*quaternion[1]));
  euler[0]=atan2(2.0*(quaternion[0]*quaternion[3]+quaternion[1]*quaternion[2]), 1-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]));

  // convert back to degrees
  for (int j=0;j<3;j++)
    euler[j]=euler[j] * 57.2957795;
}

// this simply returns the elapsed time since the last call.
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
 
  unsigned long currentTime=micros();
 
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
   
  lastTime=currentTime;
 
  return deltaT;
}

// I called this from the loop function to see what the right values were for the calibration constants.
// If you are trying to reduce the amount of time needed for calibration just try not to go so low that consecutive
// calibration calls give you completely unrelated data.  Some sensors are probably better than others.
void testCalibration()
{
  calibrateGyro();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyroZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyroThreshold[j]);
    Serial.print("  "); 
  }
  Serial.println();
  return;
}

// The settings here will suffice unless you want to use the interrupt feature.
void setupGyro()
{
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setGyroSensitivity500();

  delay(100);
}

// Call this at start up.  It's critical that your device is completely stationary during calibration.
// The sensor needs recalibration after lots of movement, lots of idle time, temperature changes, etc, so try to work that in to your design.
void calibrateGyro()
{
  long int gyroSums[3]={0};
  long int gyroSigma[3]={0};

  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
   
    // Per STM docs, we store the average of the samples for each axis and subtract them when we use the data.
    gyroZeroRate[j]=averageRate;
   
    // Per STM docs, we create a threshold for each axis based on the standard deviation of the samples times 3.
    gyroThreshold[j]=sqrt((double(gyroSigma[j]) / NUM_GYRO_SAMPLES) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE;   
  }
}

void updateGyroValues() {

  while (!(gyroReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
 
  //if (gyroReadI2C(0x27) & B01000000)
  //  Serial.println("Data missed!  Consider using an interrupt");
   
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }

  int deltaGyro[3];
  for (int j=0;j<3;j++)
  {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltaGyro[j]) < gyroThreshold[j])
      deltaGyro[j]=0;
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}

void setGyroSensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyroWriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}

void setGyroSensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}

void setGyroSensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyroWriteI2C(CTRL_REG4,0xA0);
}

int gyroReadI2C (byte regAddr) {
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyroI2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}

int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
