#include "configuration.h"
#include "custom-stepper/custom-stepper.h"

#include <Wire.h>

bool start = false;

bool switch_pressed = false;
long last_switch_press = 0;

void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //pinMode(ARM_SYTEM_PIN, INPUT_PULLUP);

  // Setup serial
  Serial.begin(9600);

  delay(50);

  Serial.println("Starting Speed Control");

  // intialize stepper driver
  Serial.println("Initializing stepper driver");
  stepper_setup();
  Serial.println("Stepper driver initialized");


}

void loop() {



  // check button
  if (!digitalRead(BUTTON_PIN)) {
    Serial.println("Button pressed. Initiating sequence...");
    start = true;
  }


  if(start) {
    start = false;

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    stepper_setSGT(40);

    // first value: steps, second value speed (lower means faster)
    stepper_move(3000, 200);

     Serial.println("starting");

    // wait
    while(stepper_getSteps() != 0) { 
      stepper_loop();
      delay(100);
      Serial.println(stepper_getSteps());
    }

    stepper_move(39000, 100);

    while(stepper_getSteps() != 0) { 
      stepper_loop();
      delay(100);
      Serial.println(stepper_getSteps());
    }

    stepper_disable();
    
    Serial.println("Sequence done. Waiting for another button press...");

  }

  delay(1);

}

