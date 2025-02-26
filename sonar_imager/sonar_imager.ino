#include "sonar_imager.h"

const int transmitterPins[NUM_TRANSMITTERS] = {4, 14, 15, 18, 19, 22, 25, 26};
const int enablePin = 27;

void setup()
{
  Serial.begin(921600);
  for (int i = 0; i < NUM_TRANSMITTERS; i++) {
    pinMode(transmitterPins[i], OUTPUT);
  }
  pinMode(enablePin, OUTPUT);
}

void loop()
{
  ;
}