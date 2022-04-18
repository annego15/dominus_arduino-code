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
  
  // initialize servos
  servo_falltuer.enable();
  servo_ausschieber.enable();

  // initialize motor (enable brake)
  motor_kette.move(0);
  motor_band.move(0);

  // initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

}

void loop() {
  
  // check button
  if (!digitalRead(BUTTON_PIN)) {
    button_pressed = true;
  }

  if(button_pressed) {
    // open falltuer
    servo_falltuer.moveTo(SERVO_POS_FALLTUER_OFFEN, SERVO_SPEED_FALLTUER_AUF);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    // move band
    motor_band.move(255);
    motor_kette.move(100);

    // wait
    delay(20000);

    // close falltuer
    servo_falltuer.moveTo(SERVO_POS_FALLTUER_ZU, SERVO_SPEED_FALLTUER_ZU);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

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
  }
}