#pragma once

#define SERVO_FALLTUER_PIN 5
#define SERVO_AUSSCHIEBER_PIN1 6
#define SERVO_AUSSCHIEBER_PIN2 10

#define SERVO_FALLTUER_POS_OFFEN 170
#define SERVO_FALLTUER_POS_ZU 10

#define SERVO_FALLTUER_SPEED_AUF 2000
#define SERVO_FALLTUER_SPEED_ZU  2000

#define SERVO_AUSSCHIEBER_POS_NORMAL 20
#define SERVO_AUSSCHIEBER_POS_DRAUSSEN 150

#define SERVO_AUSSCHIEBER_SPEED_RAUS 2500
#define SERVO_AUSSCHIEBER_SPEED_REIN 1000

#define MOTOR_KETTE_PIN_DIRECTION 12
#define MOTOR_KETTE_PIN_BRAKE 9
#define MOTOR_KETTE_PIN_SPEED 3

#define MOTOR_BAND_PIN_DIRECTION 13
#define MOTOR_BAND_PIN_BRAKE 8
#define MOTOR_BAND_PIN_SPEED 11

#define TEST_SWITCHES

#ifdef TEST_SWITCHES

#define SWITCH_KETTE_PIN A0
#define SWITCH_BAND_PIN A1
#define SWITCH_KETTE_REVERSE_PIN A2
#define SWITCH_BAND_REVERSE_PIN A3
#define SWITCH_FALLTUER_PIN A4
#define SWITCH_AUSSCHIEBER_PIN A5

#else // TEST_SWITCHES false

#define BUTTON_PIN 10

#endif // TEST_SWITCHES



