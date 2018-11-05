/* Smart Water Bottle
 * Developed for WELL IED project
 *
 * For use on an Adafruit Metro M0 Express
 * with BlueFruit LE Shield, MMA8451 Accelerometer,
 * and MILONE TECHNOLOGIES 8" eTape
 *
 * Last Updated 11/5/2018
 */


//------------------------------------------------------------------------------
// Include Libraries
//------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>


//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------
void Port_Init(void);

void Accelerometer_Read(void);
float Hydrostatic_Read(int Pin);
void error(const char *str);


// -----------------------------------------------------------------------------
// Global Variables and Constants
// -----------------------------------------------------------------------------
// These can be changed at startup as needed
#define ECHO_TO_BLUETOOTH       0   // echo data to bluetooth out
#define ECHO_TO_SERIAL          1   // echo data to serial port
#define WAIT_TO_START           1   // Wait for serial input in setup()


// Analog Input and DAC pins
const int HydroPin = A0; // Analog Input Pin Location


// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3
// for the data logging shield, we use digital pin 10 for the SD cs line
#define chipSelect 10
#define NeoPin 40
// 1 NeoPixel built in on Pin 40
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, NeoPin, NEO_GRB + NEO_KHZ800);


// Accelerometer
Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;


// Hydrostatic Resistor
// Analog Reference Voltage
#define aref_voltage 5.0
// Series Resistor Value
#define SeriesResistor 560
double Vin = 0.0;


// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Port_Init();
  Serial.begin(9600);


  pixel.begin();
  pixel.show(); // Initialize all pixels to 'off'


  #if WAIT_TO_START
    pixel.setPixelColor(0, pixel.Color(255,0,0) ); // Red
    pixel.show();
    Serial.println("Type any character to start:");
    while ( !Serial.available() );
  #endif //WAIT_TO_START
  pixel.setPixelColor(0, pixel.Color(0,0,255) ); // Blue
  pixel.show();


  if (! mma.begin()) {
    Serial.println("No MMA8451 found. Couldnt start.");
    while (1);
  }

  Serial.println("MMA8451 found:");
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = ");
  Serial.print(2 << mma.getRange());
  Serial.println("G");

}


// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop(){

  Accelerometer_Read();
  Hydrostatic_Read();


  #if ECHO_TO_SERIAL
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("Acceleration:\t");
    Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
    Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
    Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
    Serial.println("m/s^2");
    /* Display the results (Hydrostatic pressure is measured in Volts) */
    Serial.print("Hydrostatic Pressure:\t"); Serial.print(Vin);
    Serial.println("\tV");

    Serial.println();
  #endif // ECHO_TO_SERIAL


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
  pinMode(chipSelect, OUTPUT);
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
  int raw = analogRead(HydroPin);     // read the input pin
  Vin = raw * (aref_voltage / 1023.0);
}


void error(const char *str) {
  Serial.print("error: ");
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  pixel.setPixelColor(0, pixel.Color(255,0,0) );  // Red
  pixel.show();
  while(1);   // If an error is found, the code pauses.
}
