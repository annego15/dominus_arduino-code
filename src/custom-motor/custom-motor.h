#pragma once

#include <Arduino.h>

class CustomMotor {
    public:
        CustomMotor(int pin_direction, int pin_brake, int pin_speed, bool reverse);

        void move(int speed);
        void move(int speed, unsigned long time);

        void sequence_start(int speed, unsigned long time_forward, unsigned long time_backward);
        void sequence_stop();

        bool loop();

        bool getRunning();

        int getSpeed();

    
    private:

        void setDirection(bool direction);
        void setSpeed(int speed);
        void setBrake(bool brake);

        int pin_direction, pin_brake, pin_speed;
        bool reverse;

        bool running;

        bool timer_set;
        bool sequence_running, sequence_forward;

        unsigned long time_forward, time_backward;

        int speed;

        unsigned long end_time;

};

