/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
const String SENSOR0_ID = "sensor0";
const String SENSOR1_ID = "sensor1";

boolean sensorVal0State = false; //is pressing
boolean sensorVal1State = false; //is pressing

float sensorVal0;
float sensorVal1;
float threshold = 900;

const int PRESS = 1;
const int RELEASE = 0;
 
void setup(void)
{
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  //pinMode(A4, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Buttons Test 1.0"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /*Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();*/
  
  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Keyboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Bluefruit Buttons" )) ) {
    error(F("Could not set device name?"));
  }
  
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
   // Check for switch and send keystroke
  sensorVal0 = analogRead(A0); 
  sensorVal1 = analogRead(A1);//A4 on others //
  
  /*Serial.print(sensorVal0);
  Serial.print(",  ");
  Serial.print(sensorVal1);
  //Serial.print(sensorVal0State);
  */
  
  //turns led on to show a button has been pressed
  if (sensorVal0<threshold || sensorVal1<threshold) {
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  if(sensorVal1State == false) {  
     if(sensorVal0<threshold && sensorVal0State == false) {
        sensorVal0State = true;
        sendSensorTriggerEvent(SENSOR0_ID, 1);
        Serial.println("sensor 0 press");
     }else if(sensorVal0>threshold && sensorVal0State) {
        sensorVal0State = false;
        sendSensorTriggerEvent(SENSOR0_ID, 0);
        Serial.println("sensor 0 release");
     }
  }
  
  if(sensorVal0State == false) {
    if(sensorVal1 < threshold && sensorVal1State == false) {
        sensorVal1State = true;
        sendSensorTriggerEvent(SENSOR1_ID, 1);
        Serial.println("sensor 1 press");
     }else if(sensorVal1>threshold && sensorVal1State) {
        sensorVal1State = false;
        sendSensorTriggerEvent(SENSOR1_ID, 0);
        Serial.println("sensor 1 release");
     }
   }
 
  delay(100);//prevent fast press and release
}

/**************************************************************************/
/*!
    @brief  send sensor trigger event value to client
*/
/**************************************************************************/
bool sendSensorTriggerEvent(const String sensorID, int state) {
  //char stateS[2];
  //itoa(state, stateS, 10);
  String newString = sensorID + ":" + String(state);
  
  // Send characters to client
  Serial.print("[Send] ");
  Serial.println(newString);
  
  ble.print("AT+BLEUARTTX=");
  ble.println(newString);
  
  // check response stastus
  if (! ble.waitForOK() ) {
   Serial.println(F("Failed to send?"));
   return false;
  }
  return true;
}
