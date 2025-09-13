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
#include <LiquidCrystal_I2C.h> /* from Library "LiquidCrystal_I2C-1.1.2" */
#include <SPI.h>
#include "printf.h"
#include "RF24.h" /* See documentation at https://nRF24.github.io/RF24 */

#define CE_PIN 7
#define CSN_PIN 8
// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;

LiquidCrystal_I2C lcd(0x20,20,4);  // set the PCF8574 I2C address to 0x20 display = 20 chars and 4 lines (2004A)

# define LCD_LINE1 0
# define LCD_LINE2 1
# define LCD_LINE3 2
# define LCD_LINE4 3

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial); // Leonardo: wait for Serial Monitor
  //Serial.println("\nRF controller");

  lcd.init();                      // initialize the lcd 
  // turn the backlight on.
  lcd.backlight();
  // Print a message to the LCD.
  lcd.setCursor(0,LCD_LINE1); 
  lcd.print("nRF24 intializing...");
    // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    //Serial.println(F("radio hardware is not responding!!"));
    lcd.setCursor(0,LCD_LINE1); 
    lcd.print("nRF24 not responding");
    while (1) {}  // hold in infinite loop
    // add some error handling!
  }
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
