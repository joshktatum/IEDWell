/* 
  IR Breakbeam sensor demo!
*/
 
 
#define SENSORPIN 4
 
// variables will change:
int sensorState = 0, lastState=0;         // variable for reading the pushbutton status
 
void setup() {
  Serial.begin(9600);
  pinMode(SENSORPIN, INPUT);     
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
}
 
void loop(){
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);
 
  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (sensorState == LOW) {     
    Serial.println("Broken");
  } 
  else {
    Serial.println("Unbroken");
  }
}
