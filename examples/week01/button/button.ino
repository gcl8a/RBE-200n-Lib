#include <RBE1001Lib,h>

// pin definitions https://wpiroboticsengineering.github.io/RBE1001Lib/RBE1001Lib_8h.html#define-members
const int buttonPin = BOOT_FLAG_PIN;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

void setup() 
{
  pinMode(ledPin, OUTPUT);        // initialize the LED pin as an output:
  pinMode(buttonPin, INPUT);      // initialize the pushbutton pin as an input:
}

void loop()
{
  // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is LOW (inverted logic!):
  if (buttonState == LOW) 
  {     
    // turn LED on:    
    digitalWrite(ledPin, HIGH);  
  } 
  else 
  {
    // turn LED off:
    digitalWrite(ledPin, LOW); 
  }
}
