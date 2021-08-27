/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2. Rewritten for Protothreads.
  Unlike the stock Button example, this example blinks the LED when it's on.

  The circuit:
  - LED attached from pin 13 to ground
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe
  modified 2020-Jul-13
  by Ben Artin
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

#include "protothreads.h"

pt ptBlink;
int blinkThread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
	if (buttonState == HIGH) {
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		PT_SLEEP(pt, 200);
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
		PT_SLEEP(pt, 100);
	} else {
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
		PT_YIELD(pt);
	}
  }

  PT_END(pt);
}

pt ptButton;
int buttonThread(struct pt* pt) {
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
	// read the state of the pushbutton value:
  	buttonState = digitalRead(buttonPin);
	PT_YIELD(pt);
  }

  PT_END(pt);
}

void setup() {
  PT_INIT(&ptBlink);
  PT_INIT(&ptButton);

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  PT_SCHEDULE(blinkThread(&ptBlink));
  PT_SCHEDULE(buttonThread(&ptButton));
}