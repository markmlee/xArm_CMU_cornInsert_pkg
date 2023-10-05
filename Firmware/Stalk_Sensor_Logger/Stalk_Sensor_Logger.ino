#include "Wire.h"
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>
#include "RTClib.h"

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_ADS1115 ads;

// The PCF8523 Real Time Clock that's on the AdaLogger FeatherWing
RTC_PCF8523 rtc;

// Define constant values and calibration curve parameters


#define BOOST_EN 12
#define CHIP_SELECT 10

void blinkCode(int delay_ms, int times) {
  for (int i = 0; i < times; i++) {
    delay(delay_ms);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {


  // Set built in LED to output modes
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise the SD card
  pinMode(SS, OUTPUT);

  // Configure the boost converter enable pin
  pinMode(BOOST_EN, OUTPUT);
  digitalWrite(BOOST_EN, HIGH);

  delay(3000);

  // set up the ADC
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  if (!ads.begin()) {
    while (1) {
      blinkCode(25, 20);
      Watchdog.sleep(5000);
    }
  }

  if (!rtc.begin()) {
    while (1) {
      blinkCode(50, 10);
      Watchdog.sleep(5000);
    }
  } else {
    rtc.start();
  }

  // See if the card is present and can be initialized:
  if (!SD.begin(CHIP_SELECT)) {
    while (1) {
      blinkCode(100, 5);
      Watchdog.sleep(5000);
    }
  }

  // Succesful init code
  blinkCode(500, 3);
}

void loop() {
  bool error = false;

  //int16_t adc0 = 0;
  digitalWrite(BOOST_EN, HIGH);
  delay(1000);

  if (!ads.begin()) {
    error = true;
    blinkCode(25, 20);
  } else {

    // Read the ADC
    int16_t adc0 = ads.readADC_SingleEnded(0);

    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltageReading = ads.computeVolts(adc0);

    // Convert the voltage value to PPM
    float sensorValue = 1000 * (voltageReading - 2.054) / 2;

    if (!rtc.begin()) {
      error = true;
      blinkCode(50, 10);
    } else {
      rtc.start();

      //write to SD card
      File dataFile_1 = SD.open("log.csv", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile_1) {
        dataFile_1.print(sensorValue, 4);
        dataFile_1.print(",");
        dataFile_1.println(rtc.now().unixtime());
        dataFile_1.close();
      } else {
        error = true;
        blinkCode(100, 5);
      }
    }
  }

  digitalWrite(BOOST_EN, LOW);
  Watchdog.disable();
  if (!error) {
    blinkCode(100, 2);

    // wait 5 minutes (300 seconds / 5 seconds = 60 iterations)
    for (int i = 0; i < 60; i++) {
      int sleepMS = Watchdog.sleep(5000);  // Sleep for up to 5 seconds.

      //Blink the status LED for 50ms to show that the program is running
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    }
    Watchdog.enable(8000);
  }
}
