#include "custom-motor.h"

CustomMotor::CustomMotor(int pin_direction_, int pin_brake_, int pin_speed_, bool reverse_) {
  pin_direction = pin_direction_;
  pin_brake = pin_brake_;
  pin_speed = pin_speed_;
  reverse = reverse_;

  pinMode(pin_direction, OUTPUT);
  pinMode(pin_brake, OUTPUT);
  pinMode(pin_speed, OUTPUT);

  digitalWrite(pin_brake, HIGH);
  digitalWrite(pin_direction, LOW);
  analogWrite(pin_speed, 0);

  running = false;
  timer_set = false;
  speed = 0;
}

void CustomMotor::move(int speed) {
    setSpeed(speed);
}

void CustomMotor::move(int speed, unsigned long time) {
    move(speed);
    timer_set = true;
    end_time = millis() + time;
}

void CustomMotor::sequence_start(int speed_, unsigned long time_forward_, unsigned long time_backward_) {
    sequence_running = true;
    sequence_forward = true;
    time_forward = time_forward_;
    time_backward = time_backward_;
    end_time = millis() + time_forward;
    move(speed_);
}

void CustomMotor::sequence_stop() {
    sequence_running = false;
    timer_set = false;
    move(0);
}

bool CustomMotor::loop() {
    if (timer_set) {
        if (millis() >= end_time) {
            timer_set = false;
            move(0);
        }
    }
    if (sequence_running && millis() >= end_time) {
        sequence_forward = !sequence_forward;
        end_time = millis() + ((sequence_forward) ? time_forward : time_backward);
        move(-this->speed);
        return true;
    }

    return false;
}

void CustomMotor::setDirection(bool direction) {
    digitalWrite(pin_direction, (reverse) ? !direction : direction);
}

void CustomMotor::setSpeed(int speed) {
    //Serial.print("Setting Speed: ");
    Serial.println(speed);
    this->speed = speed;
    if (speed > 0) {
        setDirection(true);
    } else if (speed < 0) {
        setDirection(false);
    } else {
        setBrake(true);
    }
    setBrake(false);
    analogWrite(pin_speed, abs(speed));
}

void CustomMotor::setBrake(bool brake) {
    digitalWrite(pin_brake, brake);
}

bool CustomMotor::getRunning() {
    return running;
}

int CustomMotor::getSpeed() {
    return speed;
}

