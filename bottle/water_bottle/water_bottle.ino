/* Smart Water Bottle
 * Developed for WELL for IED project
 * 
 * Last Updated 11/2/2018
 */


//------------------------------------------------------------------------------
// Include Libraries
//------------------------------------------------------------------------------
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"


//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------
void Port_Init(void);
void TCC_Init(void);
ISR(TIMER0_COMPA_vect);
ISR(TIMER1_COMPA_vect);

float Analog_Read(int Pin);
void error(const char *str);

// -----------------------------------------------------------------------------
// Global Variables and Constants
// -----------------------------------------------------------------------------
// These can be changed at startup as needed
#define ECHO_TO_SERIAL          1   // echo data to serial port
#define WAIT_TO_START           0   // Wait for serial input in setup()
#define PULSE_WIDTH_MODULATION  1   // Pulse Width Modulates the LED driver, see Duty_Cycle below

// If you want to set the aref to something other than 5v
#define aref_voltage 5

// Analog Input and DAC pins
const int APins[] = {A0, A1}; // Analog Input Pin Locations
#define PWM_Pin 8         // There is a PWM signal attached to this pin


// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3
// for the data logging shield, we use digital pin 10 for the SD cs line
#define chipSelect 10


// Internal Timers for sampling and PWM
#define ClockSpeed0
volatile unsigned long timer0 = 0;
const int Sample_Rate = 125;        // Samples all data x times per second (Hz)


#define ClockSpeed1 1
volatile unsigned char timer1 = 0;
unsigned char Duty_Cycle = 50;      // Percent of the PWM wave that outputs high

// storage variables



// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup(){
  Port_Init();
  TCC_Init();
  Serial.begin(9600);

  #if !PULSE_WIDTH_MODULATION
    Duty_Cycle = 100;
  #endif

  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START



  timer0 = 0; // reset timer0
}


// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop(){
  // Reset all floats
  for(char i = 0; i < num_thermocouples; i++) {
    Thermocouple_T[i] = 0.0;
  }

  // Every 1/Sample_Rate Sample Data
  for(char n = 1; n <= Sample_Rate; n++){

    if(n < Sample_Rate){    // skips the last delay
      while(timer0 < ((ClockSpeed0 / Sample_Rate) * n));   // Delay for 1/Sample_Rate
    }
  }





  while(timer0 < ClockSpeed0);  // Finish out the 1 second delay
  timer0 = 0;           // reset timer0
}


//-----------------------------------------------------------------------------
// Initialization Definitions
//-----------------------------------------------------------------------------
void Port_Init(void){
  // Analog Inputs
  for(char i = 0; i < num_thermocouples; i++) {
    pinMode(APins[i], INPUT);
  }
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(chipSelect, OUTPUT);
}


void TCC_Init(void){
  cli();  //stop interrupts

  //set timer0 interrupt
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; //initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = (250000 / ClockSpeed0); // (16000000 / (ClockSpeed0 * 64)) - 1 (must be <256)
  TCCR0A |= (1 << WGM01); // turn on CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00);  // Set CS01 and CS00 bits for 64 prescaler
  TIMSK0 |= (1 << OCIE0A);  // enable timer compare interrupt

  //set timer1 interrupt
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = (250000 / ClockSpeed1); // (16000000 / (ClockSpeed1 * 64)) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Set CS11 and CS10 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  sei();  //allow interrupts
}


ISR(TIMER0_COMPA_vect){   //timer0 interrupts every 2kHz
  timer0++;
}


ISR(TIMER1_COMPA_vect){   //timer1 interrupts every 50kHz
  timer1++;
  if (timer1 >= Duty_Cycle){
    digitalWrite(PWM_Pin,LOW);   // PWM low
  }
  if (timer1 >= 100){
    digitalWrite(PWM_Pin,HIGH);  // PWM high
    timer1 = 0;   // reset counter
  }
}


//-----------------------------------------------------------------------------
// Function Definitions
//-----------------------------------------------------------------------------
float Analog_Read(int Pin){
  float temp;
  int raw = analogRead(Pin);     // read the input pin
  float Vout = raw * (aref_voltage / 1023.0);
  return Vout;
}


void error(const char *str){
  Serial.print("error: ");
  Serial.println(str);
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  while(1);   // If an error is found, the code pauses.
}
