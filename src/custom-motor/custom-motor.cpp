#include "custom-motor.h"

CustomMotor::CustomMotor(int pin_direction, int pin_brake, int pin_speed) {
  this->pin_direction = pin_direction;
  this->pin_brake = pin_brake;
  this->pin_speed = pin_speed;

  pinMode(pin_direction, OUTPUT);
  pinMode(pin_brake, OUTPUT);
  pinMode(pin_speed, OUTPUT);

  digitalWrite(pin_brake, HIGH);
  digitalWrite(pin_direction, LOW);
  analogWrite(pin_speed, 0);

  this->running = false;
  this->timer_set = false;
  this->speed = 0;
}

void CustomMotor::move(int speed) {
    this->setSpeed(speed);
}

void CustomMotor::move(int speed, unsigned long time) {
    this->move(speed);
    this->timer_set = true;
    this->end_time = millis() + time;
}

void CustomMotor::loop() {
    if (this->timer_set) {
        if (millis() >= this->end_time) {
            this->timer_set = false;
            this->move(0);
        }
    }
}

void CustomMotor::setDirection(bool direction) {
    digitalWrite(this->pin_direction, direction);
}

void CustomMotor::setSpeed(int speed) {
    this->speed = speed;
    if (speed > 0) {
        this->setDirection(true);
    } else if (speed < 0) {
        this->setDirection(false);
    } else {
        this->setBrake(true);
    }
    this->setBrake(false);
    analogWrite(this->pin_speed, abs(speed));
}

void CustomMotor::setBrake(bool brake) {
    digitalWrite(this->pin_brake, brake);
}

bool CustomMotor::getRunning() {
    return this->running;
}

int CustomMotor::getSpeed() {
    return this->speed;
}

