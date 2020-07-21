/*
 * A basic analog read example. This uses Arduino's pre-packaged analogRead() function, which 
 * doesn't use the ESP32's calibration system, but is good enough for our purposes here.
 */

#include <Arduino.h>
#include <RBE1001Lib.h>

const int photoresistorPin = A3;

/*
 * This is the standard setup function that is called when the ESP32 is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial as 115200 for prints
  Serial.begin(115200);
}


/*
 * The main loop for the program. The loop function is repeatedly called
 * once the ESP32 is started.
 */
void loop() 
{
  delay(100);

  int adcPhotoresistor = analogRead(photoresistorPin);
  Serial.println(adcPhotoresistor);
}
