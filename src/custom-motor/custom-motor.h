#pragma once

#include <Arduino.h>

class CustomMotor {
    public:
        CustomMotor(int pin_direction, int pin_brake, int pin_speed);

        void move(int speed);
        void move(int speed, unsigned long time);

        void loop();

        bool getRunning();

        int getSpeed();

    
    private:

        void setDirection(bool direction);
        void setSpeed(int speed);
        void setBrake(bool brake);

        int pin_direction, pin_brake, pin_speed;

        bool running;

        bool timer_set;

        int speed;

        unsigned long end_time;

};

