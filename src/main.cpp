#include "configuration.h"
#include "custom-stepper/custom-stepper.h"

#include <Wire.h>

bool start = false;
unsigned long last_fall = 0;

bool switch_falltuer_pressed = false;
bool switch_ausschieber_pressed = false;

unsigned long debounce_switch_falltuer = 0;
unsigned long debounce_switch_ausschieber = 0;

bool deactivated_sensor = false;

float a[100];
int i = 0;


void ausschieben() {

  if (!servo_ausschieber.getEnabled()) {
    servo_ausschieber.enable();
    delay(200);
  }

  // ausschieber raus
  servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_DRAUSSEN, SERVO_AUSSCHIEBER_SPEED_RAUS, true);

  // wait
  delay(100);

  // ausschieber zurück
  servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_NORMAL, SERVO_AUSSCHIEBER_SPEED_REIN, true);

  delay(500);

  servo_ausschieber.disable();
}

int measure() {
  digitalWrite(SENSOR_PIN, LOW);
  pinMode(SENSOR_PIN, INPUT);
  int measurement = analogRead(SENSOR_PIN);
  pinMode(SENSOR_PIN, OUTPUT);
  digitalWrite(SENSOR_PIN, HIGH);
  return measurement;
}

void setup_sensor() {
  pinMode(SENSOR_PIN, OUTPUT);
  digitalWrite(SENSOR_PIN, HIGH);
}

void disable_sensor() {
  digitalWrite(SENSOR_PIN, LOW);
}

void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //pinMode(ARM_SYTEM_PIN, INPUT_PULLUP);

  // Setup serial
  Serial.begin(9600);

  delay(50);

  Serial.println("Starting Speed Control");

  // intialize stepper driver
  Serial.println("Initializing stepper driver");
  stepper_setup();
  Serial.println("Stepper driver initialized");
  
  // initialize servos
  Serial.println("Initializing servos");
  servo_falltuer.enable();
  servo_ausschieber.enable();

  // initialize motor (enable brake)
  Serial.println("Initializing band motor");
  motor_band.move(0);

  // initialize accelerometer
  Serial.println("Initializing accelerometer");
  Wire.begin();
  accel.init();
  accel.enableDefault();
  accel.writeAccReg(LSM303::CTRL_REG1_A, 0x67);

  Serial.println("Ausschieber raus und rein...");

  ausschieben();

  //delay(500);

  //motor_band.sequence_start(255, 2000, 1000);

  deactivated_sensor = true;

  for (int i = 0; i < 10; i++) {
    if (analogRead(SENSOR_PIN) < 1000) {
      deactivated_sensor = false;
    }
  }

  if (deactivated_sensor) {
    Serial.println("Sensor deactivated");
  } else {
    Serial.println("Sensor activated");
  }

  Serial.println("Matilde Prototype v0.1 started");

  last_fall = millis();


}

void loop() {



  // check button
  /*if (!digitalRead(BUTTON_PIN)) {
    Serial.println("Button pressed. Initiating sequence...");
    start = true;
  }*/


  if(start) {
    start = false;

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    // move band
    motor_band.sequence_start(255, 3500, 1000);

    unsigned long start = millis();
    setup_sensor();
    while (millis() < (start + 1000)) {
      Serial.println(measure());
      delay(50);
    }

    stepper_setSGT(STALL_VALUE);

    // first value: steps, second value speed (lower means faster)
    stepper_move(3000, 200);

     Serial.println("starting");

    // wait
    while(stepper_getSteps() != 0) { 
      stepper_loop();
      delay(100);
      Serial.println(measure());
    }

    stepper_setSGT(STALL_VALUE_FAST);

    stepper_move(39000, 100);

    unsigned long stepper_paused = 0;
    bool timeout = false;

    while(stepper_getSteps() != 0) {
      if (deactivated_sensor) {
        if(motor_band.loop()) {
          if (motor_band.getSpeed() < 0) {
            delay(500);
            stepper_setSpeed(250);
          } else {
            stepper_setSpeed(110);
          }
        }
      } else {
        motor_band.loop();
        int m = measure();
        if (m > THRESHOLD_EMPTY) {
          if (!timeout) {
            if (stepper_getRunning()) {
              Serial.println("Pausing, because empty");
              stepper_pause();
              stepper_paused = millis();
            }
            else if (millis() > (stepper_paused + 10000)) {
              timeout = true;
              stepper_paused = millis();
              stepper_resume();
            }
          } else { // timeout == true
            if (millis() > (stepper_paused + 3000)) {
              timeout = false;
              stepper_pause();
              stepper_paused = millis();
            }
          }
        } else if (stepper_getRunning() == false) { // full and not running
          stepper_resume();
        }

        Serial.print(m);
        Serial.print(" ");
      }
      
      Serial.println(stepper_getSteps());
      stepper_loop();
      delay(50);
      
    }

    stepper_disable();
    disable_sensor();
    
    Serial.println("Sequence done. Waiting for another button press...");

  }

  delay(1);

}

