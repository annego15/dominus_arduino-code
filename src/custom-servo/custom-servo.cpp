#include "custom-servo.h"

CustomServo::CustomServo(int pin, int start_pos) {
  this->pin = pin;
  this->currentPos = start_pos;
  this->servo = new Servo();
  this->transition = false;
  this->enable();
}

void CustomServo::enable() {
  this->enabled = true;
  this->servo->attach(this->pin);
  this->servo->write(this->currentPos);
}

void CustomServo::disable() {
  this->enabled = false;
  this->servo->detach();
}

void CustomServo::moveTo(int pos, unsigned long time) {
  if (this->enabled) {
    this->transition = true;
    this->start_time = millis();
    this->end_time = this->start_time + time;
    this->start_pos = this->currentPos;
    this->end_pos = pos;
  }
}

void CustomServo::moveTo(int pos) {
  if (this->enabled) {
    this->servo->write(pos);
    this->currentPos = pos;
  }
}


void CustomServo::loop() {
  if (this->transition) {
    unsigned long current_time = millis();
    if (current_time >= this->end_time) {
      this->transition = false;
      this->servo->write(this->end_pos);
      this->currentPos = this->end_pos;
    } else {
      int current_pos = this->start_pos + (this->end_pos - this->start_pos) * (current_time - this->start_time) / float(this->end_time - this->start_time);
      this->servo->write(current_pos);
      this->currentPos = current_pos;
    }
  }
}

bool CustomServo::getEnabled() {
  return this->enabled;
}

bool CustomServo::getTransition() {
  return this->transition;
}

int CustomServo::getCurrentPos() {
  return this->currentPos;
}