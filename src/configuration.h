#pragma once

#define THRESHOLD_FALL 35

#define ARM_SYTEM_PIN 4

#define SERVO_FALLTUER_PIN 10
#define SERVO_AUSSCHIEBER_PIN1 5
#define SERVO_AUSSCHIEBER_PIN2 6

#define SERVO_FALLTUER_POS_OFFEN 155
#define SERVO_FALLTUER_POS_ZU 37

#define SERVO_FALLTUER_SPEED_AUF 500
#define SERVO_FALLTUER_SPEED_ZU  1000

#define SERVO_AUSSCHIEBER_POS_NORMAL 0
#define SERVO_AUSSCHIEBER_POS_DRAUSSEN 180

#define SERVO_AUSSCHIEBER_SPEED_RAUS 1000
#define SERVO_AUSSCHIEBER_SPEED_REIN 200

#define MOTOR_KETTE_PIN_DIRECTION 12
#define MOTOR_KETTE_PIN_BRAKE 9
#define MOTOR_KETTE_PIN_SPEED 3

#define MOTOR_BAND_PIN_DIRECTION 13
#define MOTOR_BAND_PIN_BRAKE 8
#define MOTOR_BAND_PIN_SPEED 11

#define STEP_PIN         4 // Step
#define CS_PIN           A0 // Chip select
#define SW_MOSI          A2 // Software Master Out Slave In (MOSI)
#define SW_MISO          7 // Software Master In Slave Out (MISO)
#define SW_SCK           A1 // Software Slave Clock (SCK)

#define STALL_PIN           2
#define STALL_VALUE         25 // [-64..63]
#define STALL_VALUE_FAST    29 // [-64..63]

#define R_SENSE 0.11f   // don't change

#define STEPPER_CURRENT 800 // [0..1000 mA]

#define REVERSE_STEPPER true

#define SENSOR_PIN A3

#define THRESHOLD_EMPTY 750

//#define TEST_SWITCHES

#ifdef TEST_SWITCHES

#define SWITCH_KETTE_PIN A0
#define SWITCH_BAND_PIN A1
#define SWITCH_KETTE_REVERSE_PIN A2
#define SWITCH_BAND_REVERSE_PIN A3
#define SWITCH_FALLTUER_PIN A4
#define SWITCH_AUSSCHIEBER_PIN A5

#else // TEST_SWITCHES false

#define BUTTON_PIN 12

#endif // TEST_SWITCHES



