#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include "configuration.h"




void stepper_setup();

void stepper_enable();
void stepper_disable();

void stepper_loop();

long stepper_getSteps();

bool stepper_getDirection();

void stepper_setDirection(bool dir);

bool stepper_getRunning();

void stallISR();

void stepper_move(long steps, int speed);

void stepper_setSGT(int8_t sgt);

void stepper_setSpeed(int speed_);

void stepper_stop();

void stepper_pause();

void stepper_resume();



