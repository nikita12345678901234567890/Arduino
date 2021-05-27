/*
For black arduino:
D20 - SDA
D21 - SCL
*/

//This code does not work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


//Matrix Libraries:
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
//Gesture Sensor library:
#include "Adafruit_APDS9960.h"
//Clock library:
#include "RTClib.h"
//Gyro library:
#include <Wire.h>

//Matrix pin:
#define PIN 6

//Bunch of gyro variables:
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
//End of gyro variables

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
//Upside down matrix:
Adafruit_NeoMatrix matrix2 = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

//Gesture sensor declaration:
Adafruit_APDS9960 apds;

//Clock declaration:
RTC_DS3231 rtc;

void setup()
{
  Serial.begin(9600);
  
  //Matrix initialization:
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(50);
  matrix.setTextColor(matrix.Color(255, 0, 0));
  //Upside down matrix initialization:
  matrix2.begin();
  matrix2.setTextWrap(false);
  matrix2.setBrightness(50);
  matrix2.setTextColor(matrix.Color(255, 0, 0));
  
  //Gesture sensor initialization:
  if(!apds.begin())
  {
    Serial.println("Failed to initialize gesture sensor! Please check your wiring.");
  }
  else
  {
    Serial.println("Gesture sensor initialized!");
  }
  
  //Gesture mode will be entered once proximity mode senses something close
  apds.enableProximity(true);
  apds.enableGesture(true);

  //Clock initialization:
  if (! rtc.begin())
  {
    Serial.println("Couldn't find clock");
  }
  
  //Setting the time on clock (rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); will set time to when program was compiled)
  
  //Gyro initialization:
  Wire.begin();
  setupGyro();
  //calibrateGyro();////////////////////////////
}

//Standard colors:
uint16_t red = matrix.Color(255,0,0);
uint16_t orange = matrix.Color(255, 100, 0);
uint16_t green = matrix.Color(0,255,0);
uint16_t blue = matrix.Color(0,0,255);
uint16_t white = matrix.Color(255, 255, 255);
uint16_t off = matrix.Color(0, 0, 0);

//Menu default:
int menu = 0;
//Menu stuff:
int menuMax = 1;

/*
Menu items:
0 - Clock
1 - Default Message
*/

//Display flipping:
bool flip = false;

//Timer for flashing ':':
long prevTime = 0;
bool on = true;

void loop()
{
  Serial.println("Loop");
  matrix.clear();
  matrix2.clear();
  //Update gesture sensor:
  uint8_t gesture = apds.readGesture();
  //Update clock:
  DateTime now = rtc.now();
  //Update gyro:
  updateGyroValues();
  updateHeadings();
  
  //Checking gyro values and flipping display:
  //if(/*gyro normal*/)
  //{
    //flip = false;
  //}
  //else if(/*gyro flipped*/)
  //{
    //flip = true;
  //}
  
  //Detecting gesture and changing menu position:
  if(gesture == APDS9960_LEFT && menu >= 1)
  {
    menu--;
  }
  if(gesture == APDS9960_RIGHT && menu <= menuMax - 1)
  {
    menu++;
  }
  
  //Checking if enough time has passed and toggling ':':
  if(millis() - prevTime >= 500)
  {
    on = !on;
    prevTime = millis();
  }
  
  //Displaying menu items according to position:
  switch(menu)
  {
    case 0:
      //Adding a '0' if minutes < 10
      if(now.minute() < 10)
      {
        //Checking if ':' is on and printing accordingly:
        if(on)
        {
          Print((String)now.hour() + ":0" + (String)now.minute(), red, flip);
        }
        else
        {
          Print((String)now.hour() + " 0" + (String)now.minute(), red, flip);
        }
      }
      else
      {
        //Checking if ':' is on and printing accordingly:
        if(on)
        {
          Print((String)now.hour() + ":" + (String)now.minute(), red, flip);
        }
        else
        {
          Print((String)now.hour() + " " + (String)now.minute(), red, flip);
        }
      }
      break;
    case 1:
      Print("Hello", blue, flip);
      break;
  }
}

void Print(String text, uint16_t color, bool flip)
{
  Print(text, color, 0, 0, 0, flip);
}

void Print(String text, uint16_t color, int x, int y, int Size, bool flip)
{
  if(!flip)
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
void printQuaternion()
{
  Serial.print("Quat W: ");
  Serial.print(quaternion[0]);
  Serial.print("  Quat X: ");
  Serial.print(quaternion[1]);
  Serial.print("  Quat Y: ");
  Serial.print(quaternion[2]);
  Serial.print("  Quat Z: ");
  Serial.print(quaternion[3]);
}

void printEuler()
{
  Serial.print("Euler R: ");
  Serial.print(euler[0]);
  Serial.print("  P: ");
  Serial.print(euler[1]);
  Serial.print("  Y: ");
  Serial.print(euler[2]);
}

void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyroDPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyroDPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyroDPS[2]);
}

void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}

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
