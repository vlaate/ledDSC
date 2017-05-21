/*
  7 Segment Display Astronomy Tool

  This is a multipurpose tool meant to aid in the use of telescopes.

  It features:
    Zeroable Compass (not tilt compensated), with 0.1 degree resolution. Stores offset to EEPROM.
    Zeroable Clinometer with 0.1 degree resolution. Stores offset to EEPROM.
    2 axis digital Level with 0.1 degree resolution

  These can be useful for leveling a telescope, doing basic polar alignment, or as a simple 
  alt-azimuth display of telescope position.

  Copyright (c) 2017 Vladimir Atehortua. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>
*/

/**
    Hardware used:

    Arduino Mega328 development board (UNO, Pro or similar).
    HMC5883 sensor module, I used the GY-271 variant.
    MMA8452 accelerometer, I used the GY-45 variant.
    7 Segment, 8 Digit, MAX72XX controlled, 5 Wire LED display.
    KY-040 Rotary encoder, for user input (selecting mode, zeroing the compass and inclinometer, adjusting display brightness)

    Arduino pinout:
    SDA = A4   (4.7K pullup to VCC recommended)
    SCL = A5   (4.7K pullup to VCC recommended)

    MMA8452 pinout:
    SDA, SCL and GND: matching the Arduino
    VCC (from Arduino) to VCC_IN, not to the 3V3  pin
    SAO (address select) to GND.
    When ised as a digital level, the accelerometer is meant to be used hosizontal (Z axis pointing to the sky).
    When used as inclinometer or DSC, the acelerometer is meant to be used vertical, with the Z axis pointing to the horizon.

    HMC5883 (or GY271) pinout:
    SDA, SCL, VCC and GND: matching the Arduino
    DRDY pin disconnected
    The HMC5883 is meant to be used on a permanently horizontal position (not attached to the telescope tube)

    MAX72XX LED Display:
    VCC and GND: matching the Arduino
    DIN (Data In) connected to Arduino pin 12
    CLK connected to Arduino pin 11
    LOAD (CS) connected to Arduino pin 10

    KY-040 Rotary Encoder:
    CLK connected to Arduino pin 2
    DT connected to Arduino pin 3
    SW connected to Arduino pin 4
    VCC (sometimes labeled "+" connected to Arduino VCC
    GND connected to Arduino GND
*/

#include "LedControl.h"       // library for the MAX72XX LED display
#include "Wire.h"             // i2C library
#include <HMC58X3.h>          // magnetometer library, for compass
#include <EEPROM.h>           // EEPROM library, to store offsets of the zeroable compass and inclinometer

#define RADIAN_TO_DEGREES 57.295779

//Define a few of the registers that we will be accessing on the MMA8452
#define MMA8452_ADDRESS 0x1C  // 0x1D if SA0 is high, 0x1C if low
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
// Accelerometer calibration values, you need to measure max and min for your particular accelerometer:
#define xMin  -1031
#define xMax  1023
#define yMin  -1013
#define yMax  1030
#define zMin  -992
#define zMax  1017

// Magnetometer calibration values, must be adjusted for each device:
#define minX -258.54   
#define maxX 260.97
#define minY -340.19
#define maxY 184.62


/** Class to encapsulate the behavior of the MMA8452 accelerometer for calculations of inclination and 2 axis digital level with moving average smoothing algorithm.
*   Includes public domain code originally from Nathan Seidle @SparkFun */
class MMA8452
{
#define AL_EMA_WEIGHT    0.07f  // weight coefficient for new values vs old values used in the moving average smoothing algorithm for Altitude
  public: float gx, gy, gz, angleX, angleY, altitudeReading;

  public: void measure()
    {
      int accelCount[3];  // Stores the 12-bit signed value
      readAccelData(accelCount);  // Read the x/y/z adc values
      gx = gx * (1 - AL_EMA_WEIGHT) + accelCount[0] * AL_EMA_WEIGHT;
      gy = gy * (1 - AL_EMA_WEIGHT) + accelCount[1] * AL_EMA_WEIGHT;
      gz = gz * (1 - AL_EMA_WEIGHT) + accelCount[2] * AL_EMA_WEIGHT;
    }

  public: void calculateLevelReadings()
    {
      float fangleX = 90 + atan2(-1*gz, gx) * RADIAN_TO_DEGREES;
      angleX = (1 - AL_EMA_WEIGHT) * angleX + AL_EMA_WEIGHT * fangleX;

      float fangleY = 90 + atan2(-1*gz, gy) * RADIAN_TO_DEGREES;
      angleY = (1 - AL_EMA_WEIGHT) * angleY + AL_EMA_WEIGHT * fangleY;
    }

  public: void calculateAltitudeReadings()
    {
      //float newAltitudeReading = atan2(gz, sqrt(gx * gx + gy * gy));
      float newAltitudeReading = atan2(gx, sqrt(gz * gz + gy * gy));
      newAltitudeReading = 90 + newAltitudeReading * RADIAN_TO_DEGREES;
      altitudeReading = (1 - AL_EMA_WEIGHT) * altitudeReading + AL_EMA_WEIGHT * newAltitudeReading;
    }

    // reads the accelerometer raw data, converts to 16 bit integerm, and writes it to output array
    void readAccelData(int *destination)
    {
      byte rawData[6];  // x/y/z accel register data stored here

      readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

      // Loop to calculate 12-bit ADC and g value for each axis
      for (int i = 0; i < 3 ; i++)
      {
        int gCount = (rawData[i * 2] << 8) | rawData[(i * 2) + 1]; //Combine the two 8 bit registers into one 12-bit number
        gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

        // If the number is negative, we have to make it so manually (no 12-bit data type)
        if (rawData[i * 2] > 0x7F)
        {
          gCount = ~gCount + 1;
          gCount *= -1;  // Transform into negative 2's complement #
        }

        destination[i] = gCount; //Record this gCount into the 3 int array
      }
    }

  // Initialize the MMA8452 registers, more info at http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
  public : void setup()
    {
      byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
      if (c == 0x1A) // WHO_AM_I should always be 0x2A
      {
        Serial.println("MMA8452Q is online...");
      }
      else
      {
        Serial.print("Could not connect to MMA8452Q: 0x");
        Serial.println(c, HEX);
        while (1) ; // Loop forever if communication doesn't happen
      }

      MMA8452Standby();  // Must be in standby to change registers

      // Set up the full scale range to 2, 4, or 8g.
      byte fsr = GSCALE;
      if (fsr > 8) fsr = 8; //Easy error check
      fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
      writeRegister(XYZ_DATA_CFG, fsr);

      //The default data rate is 800Hz and we don't modify it in this example code

      MMA8452Active();  // Set to active to start reading
    }

    // Sets the MMA8452 to standby mode. It must be in standby to change most register settings
    void MMA8452Standby()
    {
      byte c = readRegister(CTRL_REG1);
      writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
    }

    // Sets the MMA8452 to active mode. Needs to be in this mode to output data
    void MMA8452Active()
    {
      byte c = readRegister(CTRL_REG1);
      writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
    }

    // Read bytesToRead sequentially, starting at addressToRead into the dest byte array
    void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
    {
      Wire.beginTransmission(MMA8452_ADDRESS);
      Wire.write(addressToRead);
      Wire.endTransmission(false); //endTransmission but keep the connection active

      Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

      while (Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

      for (int x = 0 ; x < bytesToRead ; x++)
        dest[x] = Wire.read();
    }

    // Read a single byte from addressToRead and return it as a byte
    byte readRegister(byte addressToRead)
    {
      Wire.beginTransmission(MMA8452_ADDRESS);
      Wire.write(addressToRead);
      Wire.endTransmission(false); //endTransmission but keep the connection active

      Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

      while (!Wire.available()) ; //Wait for the data to come back
      return Wire.read(); //Return this one byte
    }

    // Writes a single byte (dataToWrite) into addressToWrite
    void writeRegister(byte addressToWrite, byte dataToWrite)
    {
      Wire.beginTransmission(MMA8452_ADDRESS);
      Wire.write(addressToWrite);
      Wire.write(dataToWrite);
      Wire.endTransmission(); //Stop transmitting
    }
};

/** Class to encapsulate the behaviour of the HMC58X3 magnetometer for calculation of azimuth heading with moving average smoothing algorithm */
class Compass
{
#define AZ_EMA_WEIGHT    0.04f  // weight coefficient for new values vs old values used in the moving average smoothing algorithm for Azimuth
  HMC58X3 sensor;  // compass API object
  float fx, fy;
  public : float azimuthReading;

  /** Magnetometer setup using the HMC58X3 library. */
  public : void setup()
    {
      Serial.println("\nSetting up Compass");
      sensor.init(false);      // initialize compass in single mode conversion
      sensor.calibrate(1, 32); // calibrate scale using in-built, known, current induced magnetic field, default gain, 32 samples
      sensor.setMode(0);       // Single mode conversion was used in scale calibration, now set continuous mode.
      delay(100);
      Serial.println("\nCompass ready.");
    }

  /**
    Method to obtain the azimuth orientation of the scope from the HMC58X3 magnetometer.
    This code requires the magetic sensor to be horizontal (parallel to the ground), not attached to the telescope's OTA.
  */
  public : void performAzimuthMeasurement()
    {
      float ix, iy, iz;
      sensor.getValues(&ix, &iy, &iz); // read as int values from HMC58X3

      ix = 10000.0 * (ix - minX) / (maxX - minX) - 5000.0;
      iy = 10000.0 * (iy - minY) / (maxY - minY) - 5000.0;

      fx = (1 - AZ_EMA_WEIGHT) * fx + AZ_EMA_WEIGHT * ix;
      fy = (1 - AZ_EMA_WEIGHT) * fy + AZ_EMA_WEIGHT * iy;

      // Calculating the heading requires the magnetometer sensor to be close to horizontal, so attach it to the mount, not the telescope.
      float heading = atan2(fy, fx);
      heading -= M_PI / 2; // the compass outputs 90º at north, so we substract 90º (PI/2) to offset the scale to zero, which is what Skysafari expects for north

      if (heading < 0.0)
      {
        heading += 2 * M_PI;    // atan2 produces negative values for west side, we add 360º (2*PI) to put them in a scale from 0 to 360 
      }
      float newazimuthReading = heading * RADIAN_TO_DEGREES;

      if (azimuthReading < 90)
      {
        azimuthReading += 360;
      }
      if (newazimuthReading < 90)
      {
        newazimuthReading += 360;
      }

      if (abs (azimuthReading - newazimuthReading) > 4)
      {
        azimuthReading = azimuthReading * 0.8 + newazimuthReading * 0.2;
      }
      else if (abs (azimuthReading - newazimuthReading) > 1)
      {
        azimuthReading = azimuthReading * (1 - AZ_EMA_WEIGHT) + AZ_EMA_WEIGHT * newazimuthReading;
      }
      else
      {
        azimuthReading = azimuthReading * (1 - 0.1 * AZ_EMA_WEIGHT) + 0.1 * AZ_EMA_WEIGHT * newazimuthReading;
      }

      while (azimuthReading >= 360.0)
      {
        azimuthReading -= 360.0;
      }
    }
};

/** Class to encapsulate the behavior of the 7 Segment, 8 Digit, MAX72XX 5 Wire LED display */
class Display
{
    LedControl lc = LedControl(12, 11, 10, 1);
    char* text = new char[6];

  public : void setup(int defaultBrightness)
    {
      // The MAX72XX is in power-saving mode on startup, we have to do a wakeup call:
      lc.shutdown(0, false);
      // Set the brightness to a medium values
      setBrightness(defaultBrightness);
      // and clear the display
      lc.clearDisplay(0);
    }

  public : void setBrightness(int bright)
    {
      if (bright > 15)
      {
        bright = 15;
      }
      if (bright < 0)
      {
        bright = 0;
      }
      lc.setIntensity(0, bright);
    }

  public : void clear()
    {
      lc.clearDisplay(0);
    }

  public : void printText(char* text, int numChars)
  {
    for (int j = 0; j<=7; j++)
    {
      if (j < numChars)
      {
      lc.setChar(0, 7-j, text[j], false);
      }
      else
      {
      lc.setChar(0, 7-j, ' ', false);
      }
    }
  }

  public : void printNumber(float number, boolean left)
    {
      dtostrf(number, 5, 1, text);
      int j = left ? 7 : 3;

      for (int i = 0; i < 5; i++)
      {
        byte digit = text[i] - 48;
        if (j >= 0)
        {
          if (text[i] == ' ')
          {
            lc.setChar(0, j, ' ', false);
            j--;
          }
          else if (text[i] != '.')
          {
            lc.setDigit(0, j, digit, text[i + 1] == '.');
            j--;
          }
        }
      }
    }
};

/***************  Rotary Encoder code ************************************
  uses code from http://domoticx.com/arduino-rotary-encoder-keyes-ky-040/ */

#define KY40_BUTTON_PIN   4
enum { PinA = 2, PinB = 3, IPINMODE = INPUT };
static  byte abOld;     // Initialize state
volatile int count;     // current rotary count
int old_count;     // old rotary count

// On interrupt, read input pins, compute new state, and adjust count
void pinChangeISR() {
  enum { upMask = 0x66, downMask = 0x99 };
  byte abNew = (digitalRead(PinA) << 1) | digitalRead(PinB);
  byte criterion = abNew ^ abOld;
  if (criterion == 1 || criterion == 2) {
    if (upMask & (1 << (2 * abOld + abNew / 2)))
      count++;
    else count--;       // upMask = ~downMask
  }
  abOld = abNew;        // Save new state
}

/***************** Main Code ******************/
#define EEPROM_DISPLAY_BRIGHTNESS 0 // EEPROM address for storing the preferred display brightness
#define EEPROM_OFFSET_TRUE_NORTH 2  // EEPROM address for storing offset of true north (local magnetic declination)
#define EEPROM_OFFSET_ALTITUDE 4    // EEPROM address for storing altitude offset
#define EEPROM_OFFSET_LEVEL_X 6  // EEPROM address for storing offset of true north (local magnetic declination)
#define EEPROM_OFFSET_LEVEL_Y 8    // EEPROM address for storing altitude offset
MMA8452 accelerometer;
Compass compass;
Display display;
int16_t brightness = 8;
int16_t localMagneticDeclination = 0;  // local magnetic declination, to convert magnetic heading to true north if desired
int16_t localAltitudeOffset = 0;       // altitude offset, to be able to zero the inclinometer
int16_t levelXOffset = 0;              // offset for the X axis, to be able to zero the level
int16_t levelYOffset = 0;              // offset for the Y axis, to be able to zero the level
byte mode = 0;
enum { dsc = 0, zeroCompass = 1, zeroAlt = 2, level = 3, zerolevelX = 4, zerolevelY = 5 };
/** 0 = working as dsc, rotating the encoder does brightness control
 *  1 = for zeroing compass, rotating the encoder adjusts offset
 *  2 = for zeroing altitude, rotating the encoder adjusts offset
 *  3 = working as 2 axis level, rotating the encoder  does brightness control
 */ 

void setup()
{
  // display a greeting message, because setup will take several seconds
  display.setup(0);
  display.printText("load1n9",7);   

  // Setting up interrupts for rotary encoder / user input:
  pinMode(KY40_BUTTON_PIN, IPINMODE);
  pinMode(PinA, IPINMODE);
  pinMode(PinB, IPINMODE);
  attachInterrupt(0, pinChangeISR, CHANGE); // Set up pin-change interrupts
  attachInterrupt(1, pinChangeISR, CHANGE);

  Serial.begin(9600);
  // get brightness from eeprom
  EEPROM_read(EEPROM_DISPLAY_BRIGHTNESS, brightness);
  if (brightness > 15 || brightness < 0)
  {
    brightness = 8;
  }
  display.setBrightness(brightness);

  // get local magnetic declination from eeprom
  EEPROM_read(EEPROM_OFFSET_TRUE_NORTH, localMagneticDeclination);
  if (abs(localMagneticDeclination) > 20*20)  // more than 20 degrees offset could mean corrupted eeprom
  {
    localMagneticDeclination = 0;
  }

  // get altitude offset from eeprom
  EEPROM_read(EEPROM_OFFSET_ALTITUDE, localAltitudeOffset);
  if (abs(localAltitudeOffset) > 180*20)  // more than 180 degrees offset could mean corrupted eeprom
  {
    localAltitudeOffset = 0;
  }

  // get level offsets from eeprom
  EEPROM_read(EEPROM_OFFSET_LEVEL_X, levelXOffset);
  EEPROM_read(EEPROM_OFFSET_LEVEL_Y, levelYOffset);
  if (abs(levelXOffset) > 100 || abs(levelYOffset) > 100)  // more than 100 degrees offset could mean corrupted eeprom
  {
    levelXOffset = 0;
    levelXOffset = 0;
  }

  updateCounter();
  Wire.begin();
  compass.setup();
  accelerometer.setup();
}

void loop()
{
  updateRotaryEncoderCount();
  compass.performAzimuthMeasurement();
  accelerometer.measure();
  accelerometer.calculateAltitudeReadings();
  accelerometer.calculateLevelReadings();
  printReadings();
}

long lastChange;
boolean unsaved = false;
long lastPoll;
/**
   Updates the count of steps measured by the rotary encoder interrupt.
   Polls the rotary encoder button (no more interrupt ports available, so it has to be polled)
   Also, stores the current counts on eeprom, 10 seconds after the last modification.
*/
void updateRotaryEncoderCount()
{
  if (old_count != count) {
//    Serial.println(count);
    old_count = count;
    lastChange = millis();
    unsaved = true;
    switch(mode)
      {
      case zeroCompass:
        localMagneticDeclination = count;
        break;
      case zeroAlt:
        localAltitudeOffset  = count;
        break;
      case zerolevelX:
        levelXOffset  = count;
        break;
      case zerolevelY:
        levelYOffset  = count;
        break;
      default:
        if (count > 30) {count = 30;}
        if (count < 0) {count = 0;}
        brightness = count;
        display.setBrightness(brightness / 2);
        break;
      }
  }
  
  if (unsaved && millis() - lastChange > 10000) // only save to EEPROM after 10 seconds of last change
  {
    EEPROM_write(EEPROM_DISPLAY_BRIGHTNESS, brightness);
    EEPROM_write(EEPROM_OFFSET_TRUE_NORTH, count);
    EEPROM_write(EEPROM_OFFSET_ALTITUDE, localAltitudeOffset);
    EEPROM_write(EEPROM_OFFSET_LEVEL_X, levelXOffset);
    EEPROM_write(EEPROM_OFFSET_LEVEL_Y, levelYOffset);
    unsaved = false;
  }

  // The encoder button has to be polled, because the rotary encoder is already using the two ports usable for interrupts on Mega328 Aruino boards
  if (millis() - lastPoll > 500 && !digitalRead(KY40_BUTTON_PIN))
  {
    Serial.println("button pressed!!!");
    lastPoll = millis();
    display.clear();
    mode++;
    if (mode > zerolevelY) {mode = 0;}
    updateCounter();
  }
}

/** When switching moes, makes sure the counter value is the correct one */
void updateCounter()
{
    switch (mode)
    {
      case dsc:
        abOld = count = old_count = brightness;
        break;
      case zeroCompass:
        abOld = count = old_count = localMagneticDeclination;
        break;
      case zeroAlt:
        abOld = count = old_count = localAltitudeOffset;
        break;
      case level:
        abOld = count = old_count = brightness;
        break;
      case zerolevelX:
        abOld = count = old_count = levelXOffset;
        break;
      case zerolevelY:
        abOld = count = old_count = levelYOffset;
        break;
    }
}

/**
 * Prints the readings on the Display, according to the current mode, but only once every 100 milliseconds
 */
long lastTime;
void printReadings()
{
  if (millis() - 100 > lastTime)
  {
    float azimuth = compass.azimuthReading + localMagneticDeclination / 20.0 + 0.05; // for rounding of second decimal
    while (azimuth >= 360.0)
    {
      azimuth -= 360.0;
    }
    float altitude = accelerometer.altitudeReading + localAltitudeOffset  / 20.0 + 0.05; // for rounding of second decimal
    while (altitude >= 360.0)
    {
      altitude -= 360.0;
    }
    float levelX = accelerometer.angleX + ((float)levelXOffset / 20.0);
    float levelY = accelerometer.angleY + ((float)levelYOffset / 20.0);   

    switch (mode)
    {
      case dsc:
        display.printNumber(azimuth, true);
        display.printNumber(altitude, false);
        break;
      case zeroCompass:
        display.printNumber(azimuth, true);
        break;      
      case zeroAlt:
        display.printNumber(altitude, false);
        break;
      case level:      
        display.printNumber(levelX, true);
        display.printNumber(levelY, false);
        break;
      case zerolevelX:
        display.printNumber(levelX, true);
        break;      
      case zerolevelY:
        display.printNumber(levelY, false);
        break;      
    }
    lastTime = millis();
  }
}

/** Utility method for storing stuff in Arduino EEPROM*/
template <class T> int EEPROM_write(int address, const T& value)
{
  Serial.println("writing");
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(address++, *p++);
  return i;
}

/** Utility method for retrieving stuff from Arduino EEPROM*/
template <class T> int EEPROM_read(int address, T& value)
{
  Serial.println("reading eeprom");
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(address++);
  return i;
}
