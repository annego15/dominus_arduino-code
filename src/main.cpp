#include <Arduino.h>
#include "custom-servo/custom-servo.h"
#include "custom-motor/custom-motor.h"
#include "configuration.h"

CustomServo servo_falltuer(SERVO_PIN_FALLTUER, SERVO_POS_FALLTUER_ZU);
CustomServo servo_ausschieber(SERVO_PIN_AUSSCHIEBER, SERVO_POS_AUSSCHIEBER_NORMAL);

CustomMotor motor_kette(MOTOR_KETTE_PIN_DIRECTION, MOTOR_KETTE_PIN_BRAKE, MOTOR_KETTE_PIN_SPEED);
CustomMotor motor_band(MOTOR_BAND_PIN_DIRECTION, MOTOR_BAND_PIN_BRAKE, MOTOR_BAND_PIN_SPEED);

bool button_pressed = false;

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

  // initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Waiting for button press...");

}

void loop() {
  
  // check button
  if (!digitalRead(BUTTON_PIN)) {
    button_pressed = true;
  }

  if(button_pressed) {

    Serial.println("Button pressed. Initiating sequence...");

    // open falltuer
    Serial.println("Opening falltuer");
    servo_falltuer.moveTo(SERVO_POS_FALLTUER_OFFEN, SERVO_SPEED_FALLTUER_AUF);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    // move band
    motor_band.move(255);
    motor_kette.move(100);

    // wait
    delay(20000);

    Serial.println("Stoppe band und kette. Schließe falltuer...");

    // close falltuer
    servo_falltuer.moveTo(SERVO_POS_FALLTUER_ZU, SERVO_SPEED_FALLTUER_ZU);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    Serial.println("Falltuer geschlossen. Ausschieber raus und rein...");

    // ausschieber raus
    servo_ausschieber.moveTo(SERVO_POS_AUSSCHIEBER_DRAUSSEN, SERVO_SPEED_AUSSCHIEBER_RAUS);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    // wait
    delay(100);

    // ausschieber zurück
    servo_ausschieber.moveTo(SERVO_POS_AUSSCHIEBER_NORMAL, SERVO_SPEED_AUSSCHIEBER_REIN);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    Serial.println("Sequence done. Waiting for another button press...");
  }

  delay(5);
}