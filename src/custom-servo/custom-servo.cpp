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


CoupledServo::CoupledServo(int pin_servo1, int pin_servo2, int start_pos, bool invert_direction, int offset) {
  this->pin_servo1 = pin_servo1;
  this->pin_servo2 = pin_servo2;
  this->invert_direction = invert_direction;
  this->offset = offset;
  this->servo1 = new CustomServo(this->pin_servo1, start_pos);
  this->servo2 = new CustomServo(this->pin_servo2, convertPos(start_pos));
}

void CoupledServo::enable() {
  this->servo1->enable();
  this->servo2->enable();
}

void CoupledServo::disable() {
  this->servo1->disable();
  this->servo2->disable();
}

void CoupledServo::moveTo(int pos, unsigned long time) {
  this->servo1->moveTo(pos, time);
  this->servo2->moveTo(convertPos(pos), time);
}

void CoupledServo::moveTo(int pos) {
  this->servo1->moveTo(pos);
  this->servo2->moveTo(convertPos(pos));
}

void CoupledServo::loop() {
  this->servo1->loop();
  this->servo2->loop();
}

bool CoupledServo::getEnabled() {
  return this->servo1->getEnabled() && this->servo2->getEnabled();
}

bool CoupledServo::getTransition() {
  return this->servo1->getTransition() || this->servo2->getTransition();
}

int CoupledServo::getCurrentPos() {
  return this->servo1->getCurrentPos();
}

int CoupledServo::convertPos(int pos) {
  if (this->invert_direction) {
    return 180 - pos - this->offset;
  } else {
    return pos + this->offset;
  }
}