// --------------------------------------
// Info:
//
// The Blue Arduino Nano V3 I have used in this project came from Aliexpress
// It contained a ATmega328PB chip. This chip has a different chip ID than
// the ATmega328P and therefore gave a warning when using the "Arduino UNO"
// as a target board. 
// If you use an ATmega328PB chip, please add the mcudude minicore Arduino package URL: 
// https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
// to the "Additional Boards Manager URLs" list in preferences.
// This package also contains a new bootloader, but the Optiboot bootloader may also be used.
// Then after installing the MiniCore package, 
// use the tools menu entry to set Tools-->Board as "ATmega328" 
// and also set Tools-->Variant as 328PB
//
// The black and white Makefun Nano board from Aliexpress does have a ATmega328P chip and can be used
// as a regular Arduino Uno board.

// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not known.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20,20,4);  // set the PCF8574 I2C address to 0x20 display = 20 chars and 4 lines (2004A)

# define LCD_LINE1 0
# define LCD_LINE2 1
# define LCD_LINE3 2
# define LCD_LINE4 3

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial); // Leonardo: wait for Serial Monitor
  Serial.println("\nI2C Scanner");
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,LCD_LINE1); 
  lcd.print("Hello, world!");
  lcd.setCursor(2,LCD_LINE2);
  lcd.print("Arduino UNO Nano!");
   lcd.setCursor(1,LCD_LINE3);
  lcd.print("PCF8574 to HD44780");
   lcd.setCursor(1,LCD_LINE4);
  lcd.print("Powered By Arduino!");
}

void loop() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
}
