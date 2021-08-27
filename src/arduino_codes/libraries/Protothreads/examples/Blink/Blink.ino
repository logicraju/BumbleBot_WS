/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly. Rewritten with Protothreads.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman
  modified 2020-Jul-13
  by Ben Artin
*/

#include "protothreads.h"

pt ptBlink;
int blinkThread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    PT_SLEEP(pt, 1000);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    PT_SLEEP(pt, 1000);
  }

  PT_END(pt);
}

// the setup function runs once when you press reset or power the board
void setup() {
  PT_INIT(&ptBlink);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  PT_SCHEDULE(blinkThread(&ptBlink));
}
