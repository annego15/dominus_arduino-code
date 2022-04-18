#ifndef CUSTOM_SERVO_H
#define CUSTOM_SERVO_H

#include <Arduino.h>
#include <Servo.h>

class CustomServo {
    public:

        CustomServo(int pin, int start_pos);

        void enable();
        void disable();

        void moveTo(int pos, unsigned long time);
        void moveTo(int pos);

        void loop();

        bool getEnabled();
        bool getTransition();

        int getCurrentPos();
    
    private:

        int pin;

        int currentPos;

        unsigned long start_time;
        unsigned long end_time;

        int start_pos;
        int end_pos;

        bool enabled;

        bool transition;

        Servo *servo;
};

#endif