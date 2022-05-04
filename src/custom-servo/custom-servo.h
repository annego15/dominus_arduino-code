#ifndef CUSTOM_SERVO_H
#define CUSTOM_SERVO_H

#include <Arduino.h>
#include <Servo.h>

class CustomServo {
    public:

        CustomServo(int pin, int start_pos);

        void enable();
        void disable();

        void moveTo(int pos);
        void moveTo(int pos, unsigned long time);
        void moveTo(int pos, unsigned long time, bool wait);

        void loop();

        bool getEnabled();
        bool getTransition();

        int getCurrentPos();

        Servo *servo;
    
    private:

        int pin;

        int currentPos;

        unsigned long start_time;
        unsigned long end_time;

        int start_pos;
        int end_pos;

        bool enabled;

        bool transition;

        
};

class CoupledServo {

    public:

        CoupledServo(int pin_servo1, int pin_servo2, int start_pos, bool invert_direction, int offset);

        void enable();
        void disable();

        void moveTo(int pos);
        void moveTo(int pos, unsigned long time);
        void moveTo(int pos, unsigned long time, bool wait);

        void loop();

        bool getEnabled();
        bool getTransition();

        int getCurrentPos();

        CustomServo *servo1;
        CustomServo *servo2;
    
    private:

        int convertPos(int pos);

        int pin_servo1, pin_servo2;

        bool invert_direction;
        int offset;


};


#endif