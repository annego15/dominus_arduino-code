#include "custom-servo.h"

CustomServo::CustomServo(int pin_, int start_pos_) {
  pin = pin_;
  currentPos = start_pos_;
  servo = new Servo();
  transition = false;
}

void CustomServo::enable() {
  if (!enabled) {
    enabled = true;
    Serial.print("Enabling servo on pin ");
    Serial.println(pin);
    servo->attach(pin);
    servo->write(currentPos);
  }
}

void CustomServo::disable() {
  Serial.println("Disabling servo");
  enabled = false;
  servo->detach();
}

void CustomServo::moveTo(int pos) {
  if (enabled) {
    Serial.print("Moving servo to specific pos ");
      Serial.println(pos);
    servo->write(pos);
    currentPos = pos;
  }
}

void CustomServo::moveTo(int pos, unsigned long time) {
  if (enabled) {
    transition = true;
    start_time = millis();
    end_time = start_time + time;
    start_pos = currentPos;
    end_pos = pos;
  }
}

void CustomServo::moveTo(int pos, unsigned long time, bool wait) {
  moveTo(pos, time);
  while(wait && getTransition()) {
    this->loop();
    delay(1);
  }
  Serial.println("Done moving single");
}


void CustomServo::loop() {
  if (transition) {
    unsigned long current_time = millis();
    if (current_time >= end_time) {
      transition = false;
      servo->write(end_pos);
      currentPos = end_pos;
      Serial.print("Moving servo to end ");
      Serial.println(currentPos);
    } else {
      /*Serial.print("start pos: ");
      Serial.print(start_pos);
      Serial.print(" end pos: ");
      Serial.print(end_pos);
      Serial.print(" current time: ");
      Serial.print(current_time);
      Serial.print(" end time: ");
      Serial.print(end_time);
      Serial.print("start time: ");
      Serial.println(start_time);*/
      int current_pos = start_pos + (end_pos - start_pos) * (float(current_time) - start_time) / float(end_time - start_time);
      servo->write(current_pos);
      Serial.print("Moving servo to ");
      Serial.println(currentPos);
      currentPos = current_pos;
    }
  }
}

bool CustomServo::getEnabled() {
  return enabled;
}

bool CustomServo::getTransition() {
  return transition;
}

int CustomServo::getCurrentPos() {
  return currentPos;
}


CoupledServo::CoupledServo(int pin_servo1_, int pin_servo2_, int start_pos, bool invert_direction_, int offset_) {
  pin_servo1 = pin_servo1_;
  pin_servo2 = pin_servo2_;
  invert_direction = invert_direction_;
  offset = offset_;
  servo1 = new CustomServo(pin_servo1, start_pos);
  servo2 = new CustomServo(pin_servo2, convertPos(start_pos));
}

void CoupledServo::enable() {
  servo1->enable();
  servo2->enable();
}

void CoupledServo::disable() {
  servo1->disable();
  servo2->disable();
}

void CoupledServo::moveTo(int pos) {
  servo1->moveTo(pos);
  servo2->moveTo(convertPos(pos));
}

void CoupledServo::moveTo(int pos, unsigned long time) {
  servo1->moveTo(pos, time);
  servo2->moveTo(convertPos(pos), time);
}

void CoupledServo::moveTo(int pos, unsigned long time, bool wait) {
  moveTo(pos, time);
  while(wait && getTransition()) {
    this->loop();
    delay(1);
    Serial.println("loop");
  }
  Serial.println("Done moving");
}

void CoupledServo::loop() {
  servo1->loop();
  servo2->loop();
}

bool CoupledServo::getEnabled() {
  return servo1->getEnabled() && servo2->getEnabled();
}

bool CoupledServo::getTransition() {
  return servo1->getTransition() || servo2->getTransition();
}

int CoupledServo::getCurrentPos() {
  return servo1->getCurrentPos();
}

int CoupledServo::convertPos(int pos) {
  if (invert_direction) {
    return 180 - pos - offset;
  } else {
    return pos + offset;
  }
}