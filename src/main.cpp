#include <Arduino.h>
#include "custom-servo/custom-servo.h"
#include "custom-motor/custom-motor.h"
#include "configuration.h"

CustomServo servo_falltuer(SERVO_FALLTUER_PIN, SERVO_FALLTUER_POS_ZU);
CoupledServo servo_ausschieber(SERVO_AUSSCHIEBER_PIN1, SERVO_AUSSCHIEBER_PIN2, SERVO_AUSSCHIEBER_POS_NORMAL, true, 0);

CustomMotor motor_kette(MOTOR_KETTE_PIN_DIRECTION, MOTOR_KETTE_PIN_BRAKE, MOTOR_KETTE_PIN_SPEED);
CustomMotor motor_band(MOTOR_BAND_PIN_DIRECTION, MOTOR_BAND_PIN_BRAKE, MOTOR_BAND_PIN_SPEED);

bool button_pressed = false;

bool switch_falltuer_pressed = false;
bool switch_ausschieber_pressed = false;

unsigned long debounce_switch_falltuer = 0;
unsigned long debounce_switch_ausschieber = 0;

void setup() {

  // Setup serial
  Serial.begin(9600);

  Serial.println("Starting Matilde Prototype v0.1");
  
  // initialize servos
  Serial.println("Initializing servos");
  servo_falltuer.enable();
  servo_ausschieber.enable();

  // initialize motor (enable brake)
  Serial.println("Initializing motors");
  motor_kette.move(0);
  motor_band.move(0);

  #ifdef TEST_SWITCHES

  Serial.println("Initializing switches");
  pinMode(SWITCH_KETTE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_BAND_PIN, INPUT_PULLUP);
  pinMode(SWITCH_KETTE_REVERSE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_BAND_REVERSE_PIN, INPUT_PULLUP);
  pinMode(SWITCH_FALLTUER_PIN, INPUT_PULLUP);
  pinMode(SWITCH_AUSSCHIEBER_PIN, INPUT_PULLUP);
  
  #else // TEST_SWITCHES false
  // initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  #endif // TEST_SWITCHES

  Serial.println("Starting loop!");

}

void loop() {

  #ifdef TEST_SWITCHES

  if (digitalRead(SWITCH_KETTE_PIN) == LOW) {
    motor_kette.move(255);
  } else {
    if (digitalRead(SWITCH_KETTE_REVERSE_PIN) == LOW) {
      motor_kette.move(-255);
    } else {
      motor_kette.move(0);
    }
  }

  if (digitalRead(SWITCH_BAND_PIN) == LOW) {
    motor_band.move(255);
  } else {
    if (digitalRead(SWITCH_BAND_REVERSE_PIN) == LOW) {
      motor_band.move(-255);
    } else {
      motor_band.move(0);
    }
  }

  if (digitalRead(SWITCH_FALLTUER_PIN) == LOW && !switch_falltuer_pressed && millis() - debounce_switch_falltuer > 100) {
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_OFFEN, SERVO_FALLTUER_SPEED_AUF);
    switch_falltuer_pressed = true;
    debounce_switch_falltuer = millis();
  } else if (digitalRead(SWITCH_FALLTUER_PIN) == HIGH && switch_falltuer_pressed && millis() - debounce_switch_falltuer > 100) {
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_ZU, SERVO_FALLTUER_SPEED_ZU);
    switch_falltuer_pressed = false;
    debounce_switch_falltuer = millis();
  }

  if (digitalRead(SWITCH_AUSSCHIEBER_PIN) == LOW && !switch_ausschieber_pressed && millis() - debounce_switch_ausschieber > 100) {
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_DRAUSSEN, SERVO_AUSSCHIEBER_SPEED_RAUS);
    switch_ausschieber_pressed = true;
    debounce_switch_ausschieber = millis();
  } else if (digitalRead(SWITCH_AUSSCHIEBER_PIN) == HIGH && switch_ausschieber_pressed && millis() - debounce_switch_ausschieber > 100) {
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_NORMAL, SERVO_AUSSCHIEBER_SPEED_REIN);
    switch_ausschieber_pressed = false;
    debounce_switch_ausschieber = millis();
  }


  #else // TEST_SWITCHES false
  
  // check button
  if (!digitalRead(BUTTON_PIN)) {
    button_pressed = true;
  }

  if(button_pressed) {

    Serial.println("Button pressed. Initiating sequence...");

    // open falltuer
    Serial.println("Opening falltuer");
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_OFFEN, SERVO_FALLTUER_SPEED_AUF);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    // move band
    motor_band.move(255);
    motor_kette.move(255);

    // wait
    delay(20000);

    Serial.println("Stoppe band und kette. Schließe falltuer...");

    // close falltuer
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_ZU, SERVO_FALLTUER_SPEED_ZU);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    Serial.println("Falltuer geschlossen. Ausschieber raus und rein...");

    // ausschieber raus
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_DRAUSSEN, SERVO_AUSSCHIEBER_SPEED_RAUS);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    // wait
    delay(100);

    // ausschieber zurück
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_NORMAL, SERVO_AUSSCHIEBER_SPEED_REIN);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    Serial.println("Sequence done. Waiting for another button press...");
  }

  delay(5);

  #endif // TEST_SWITCHES
}