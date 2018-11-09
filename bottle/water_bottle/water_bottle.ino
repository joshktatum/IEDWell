/* Smart Water Bottle
 * Developed for WELL IED project
 *
 * For use on an Adafruit Metro M0 Express
 * with BlueFruit LE Shield, MMA8451 Accelerometer,
 * and MILONE TECHNOLOGIES 8" eTape
 *
 * Last Updated 11/6/2018
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


#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif


//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------
void Port_Init(void);

void Accelerometer_Read(void);
void Hydrostatic_Read(void);
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
sensors_event_t event;


// Hydrostatic Resistor
// Analog Input and DAC pins
const int HydroPin =            A0;   // Analog Input Pin Location
// Built in Reference Resistor
#define Rref                    1300

unsigned int raw =              0;
unsigned int RHydro =           0.0;


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

  Accelerometer_Read();
  Hydrostatic_Read();


  #if ECHO_TO_BLUETOOTH
    /* Command is sent when \n (\r) or println is called */
     // Send command
     ble.print("AT+BLEUARTTX=");
     ble.print(RHydro);
     ble.println(",");

    /* Check if command executed OK */
    if ( !ble.waitForOK() ) {
      Serial.println(F("Failed to get response!"));
    } else {
      Serial.println(F("Sent response!"));
    }
  #endif


  #if ECHO_TO_SERIAL
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("Acceleration:\t");
    Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
    Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
    Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
    Serial.println("m/s^2");
    /* Display the results (Hydrostatic pressure is measured in Volts) */
    Serial.print("Hydrostatic Pressure:\t"); Serial.print(RHydro);
    Serial.println("\tOhms");

    Serial.println();
  #endif // ECHO_TO_SERIAL


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
void Accelerometer_Read(void) {
  /* Read the 'raw' data in 14-bit counts */
  mma.read();
  /* Get a new sensor event */
  mma.getEvent(&event);
}


void Hydrostatic_Read(void) {
  /* Analog Read the 'raw' data in 10-bit counts */
  raw = analogRead(HydroPin);
  /* Convert the 'raw' data to Resistance */
  RHydro = (raw * Rref) / (1023 - raw);
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
