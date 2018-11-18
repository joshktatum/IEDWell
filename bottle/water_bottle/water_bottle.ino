/* Smart Water Bottle
 * Developed for WELL IED project
 *
 * For use on an Adafruit Metro M0 Express
 * with BlueFruit LE Shield, MMA8451 Accelerometer,
 * and MILONE TECHNOLOGIES 8" eTape
 *
 * Last Updated 11/18/2018
 */


//------------------------------------------------------------------------------
// Include Libraries
//------------------------------------------------------------------------------
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include "BluefruitConfig.h"
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <cassert>


#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------
void Port_Init(void);

sensors_event_t Accelerometer_Read(void);
bool Accelerometer_Read(uint8_t num);
float Hydrostatic_Read(void);
float Hydrostatic_Read(uint8_t num);
bool getUserInput(char buffer[], uint8_t maxSize);
void error(const __FlashStringHelper* str);


// -----------------------------------------------------------------------------
// Global Variables and Constants
// -----------------------------------------------------------------------------
// These can be changed at startup as needed
#define ECHO_TO_BLUETOOTH       1   // echo data to bluetooth out
#define ECHO_TO_SERIAL          0   // echo data to serial port
#define WAIT_TO_START           1   // Wait for serial input in setup()


// Analog Reference Voltage
#define aref_voltage            5.0

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
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

/* Create the bluefruit object with a hardware SPI,
 * using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
 */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// 1 NeoPixel built in on Pin 40
#define NeoPin                  40
Adafruit_NeoPixel pixel =       Adafruit_NeoPixel(1, NeoPin, NEO_GRB + NEO_KHZ800);
#define greenLEDpin             3
#define redLEDpin               2


// Accelerometer
Adafruit_MMA8451 mma =          Adafruit_MMA8451();

// Hydrostatic Resistor
// Analog Input and DAC pins
const int HydroPin =            A0;   // Analog Input Pin Location
// Built in Reference Resistor
#define Rref                    1300


// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  while(!Serial);
  delay(500);

  Port_Init();
  Serial.begin(9600);


  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0,255,0) ); // Green
  pixel.show(); // Initialize all pixels to 'off'


  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ) {
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

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Well': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit Well")) ) {
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


  // Clear any previous custom services/characteristics
  ble.sendCommandCheckOK("AT+GATTCLEAR");

  // Add a battery service (UUID = 0x180F) to the peripheral
  ble.sendCommandCheckOK("AT+GATTADDSERVICE=UUID=0x180F");

  // Add a battery measurement characteristic (UUID = 0x2A19), notify enabled
  ble.sendCommandCheckOK("AT+GATTADDCHAR=UUID=0x2A19,PROPERTIES=0x10");



  if (! mma.begin()) {
    Serial.println("No MMA8451 found. Couldnt start.");
    while (1);
  }
  Serial.println("MMA8451 found:");
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = ");
  Serial.print(2 << mma.getRange());
  Serial.println("G");


  #if WAIT_TO_START
    pixel.setPixelColor(0, pixel.Color(255,0,0) ); // Red
    pixel.show();
    Serial.println("Type any character to start:");
    while ( !Serial.available() );
  #endif //WAIT_TO_START
  pixel.setPixelColor(0, pixel.Color(0,0,255) ); // Blue
  pixel.show();

}


// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop(){

  bool isUpright = Accelerometer_Read(10);

  if ( isUpright ) {
    float hydro = Hydrostatic_Read(10);

    #if ECHO_TO_BLUETOOTH
      /* Command is sent when \n (\r) or println is called */
      /* NOTE: Missing command generation, currently sending a constant*/
      ble.sendCommandCheckOK("AT+GATTCHAR=1,1400");

      /* Check if command executed OK */
      if ( !ble.waitForOK() ) {
        pixel.setPixelColor(0, pixel.Color(255,0,0) );  // Red
        pixel.show();
        Serial.println(F("Failed to get response!"));
      } else {
        pixel.setPixelColor(0, pixel.Color(0,0,255) );  // Blue
        pixel.show();
        Serial.println(F("Sent response!"));
      }
    #endif

  } else {

    #if ECHO_TO_BLUETOOTH
      /* Command is sent when \n (\r) or println is called */
      /* Currently sending -1 to mean no information available */
      ble.sendCommandCheckOK("AT+GATTCHAR=1,-1");

      /* Check if command executed OK */
      if ( !ble.waitForOK() ) {
        pixel.setPixelColor(0, pixel.Color(255,0,0) );  // Red
        pixel.show();
        Serial.println(F("Failed to get response!"));
      } else {
        pixel.setPixelColor(0, pixel.Color(127,0,127) );  // Purple
        pixel.show();
        Serial.println(F("Sent response!"));
      }
    #endif

  }

  /* Delay before next measurement update */
  delay(500);
}


//-----------------------------------------------------------------------------
// Initialization Definitions
//-----------------------------------------------------------------------------
void Port_Init(void) {
  // Analog Inputs
  pinMode(HydroPin, INPUT);
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(BLUEFRUIT_SPI_CS, OUTPUT);
}


//-----------------------------------------------------------------------------
// Function Definitions
//-----------------------------------------------------------------------------
/* Reads the Accelerometer once*/
sensors_event_t Accelerometer_Read(void) {
  /* Read the 'raw' data in 14-bit counts */
  mma.read();
  /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);
}

/* Reads the accelerometer a number of times */
/* Averages the acceletrations */
/* Returns whether the bottle is upright and steady */
bool Accelerometer_Read(uint8_t num) {
  /* declare storage variables for the accelerometer data */
  float x = 0.0; float y = 0.0; float z = 0.0;
  /* read the accelerometer for num times */
  for (uint8_t i = 0; i < num; i++){
    sensors_event_t event = Accelerometer_Read();
    x += event.acceleration.x;
    y += event.acceleration.y;
    z += event.acceleration.z;
  }
  /* average the accelerometer reads */
  x /= num; y /= num; z /= num;
  /* Calculate the total acceleration */
  float total = sqrt(pow(x,2) + pow(y,2) + pow(z,2));

  #if ECHO_TO_SERIAL
    /* Display the results on serial (acceleration is measured in m/s^2) */
    Serial.print("Acceleration:\t");
    Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
    Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
    Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
    Serial.print("Total:\t"); Serial.print(total); Serial.print("\t");
    Serial.println("m/s^2");
    Serial.println();
  #endif // ECHO_TO_SERIAL

  /* Decide whether the water bottle is upright */
  if (z < 9.0) { return false; }
  /* Decide whether the bottle is in motion */
  if ( total > 11.0) { return false; }
  /* Otherwise, the bottle must be upright and stable */
  return true;
}


float Hydrostatic_Read(void) {
  /* Analog Read the 'raw' data in 10-bit counts */
  uint16_t raw = analogRead(HydroPin);
  /* Convert the 'raw' data to Resistance */
  float RSENSE = (raw * Rref) / (1023.0 - raw);
  return RSENSE;
}

float Hydrostatic_Read(uint8_t num) {
  /* declare storage variable for the Hydrostatic data */
  float average = 0.0;
  /* read the hydrostatic Sensor for num times */
  for (uint8_t i = 0; i < num; i++){
    average += Hydrostatic_Read();
  }
  /* average the hydrostatic reads */
  average /= num;
  return average;
}


bool getUserInput(char buffer[], uint8_t maxSize) {
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while ( !Serial.available() && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) { return false; }

  uint8_t count = 0;
  do {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while ( (count < maxSize) && (Serial.available()) );

  return true;
}


void error(const __FlashStringHelper* str) {
  Serial.print("error: ");
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  pixel.setPixelColor(0, pixel.Color(255,0,0) );  // Red
  pixel.show();
  while (1);   // If an error is found, the code pauses.
}
