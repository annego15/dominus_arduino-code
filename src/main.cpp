#include "configuration.h"
#include "custom-motor/custom-motor.h"
#include "custom-servo/custom-servo.h"
#include "custom-stepper/custom-stepper.h"

CustomMotor motor_band(MOTOR_KETTE_PIN_DIRECTION, MOTOR_KETTE_PIN_BRAKE, MOTOR_KETTE_PIN_SPEED, true);

CustomServo servo_falltuer(SERVO_FALLTUER_PIN, SERVO_FALLTUER_POS_ZU);
CoupledServo servo_ausschieber(SERVO_AUSSCHIEBER_PIN1, SERVO_AUSSCHIEBER_PIN2, SERVO_AUSSCHIEBER_POS_NORMAL, true, -5);

bool button_pressed = false;

bool switch_falltuer_pressed = false;
bool switch_ausschieber_pressed = false;

unsigned long debounce_switch_falltuer = 0;
unsigned long debounce_switch_ausschieber = 0;


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


void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup serial
  Serial.begin(9600);

  delay(50);

  Serial.println("Starting Matilde Prototype v0.1");
  
  // initialize servos
  Serial.println("Initializing servos");
  servo_falltuer.enable();
  servo_ausschieber.enable();

  // initialize motor (enable brake)
  Serial.println("Initializing band motor");
  motor_band.move(0);


  // intialize stepper driver
  Serial.println("Initializing stepper driver");
  stepper_setup();
  Serial.println("Stepper driver initialized");

  Serial.println("Ausschieber raus und rein...");

  ausschieben();

  delay(500);

  //motor_band.sequence_start(255, 2000, 1000);

  Serial.println("Matilde Prototype v0.1 started");


}

void loop() {



  // check button
  if (!digitalRead(BUTTON_PIN)) {
    button_pressed = true;
  }

  if(button_pressed) {
    button_pressed = false;

    Serial.println("Button pressed. Initiating sequence...");

    servo_falltuer.enable();

    // open falltuer
    Serial.println("Opening falltuer");
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_OFFEN, SERVO_FALLTUER_SPEED_AUF, true);

    delay(100);
    servo_falltuer.disable();

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    // move band
    motor_band.sequence_start(255, 3500, 1000);

    delay(1000);

    stepper_move(5000, 200);

     Serial.println("starting");

    // wait
    while(stepper_getSteps() != 0) { 
      motor_band.loop();
      stepper_loop();
      delay(100);
      Serial.println(stepper_getSteps());
    }

    stepper_move(45000, 200);

    while(stepper_getSteps() != 0) { 
      if(motor_band.loop()) {
        if (motor_band.getSpeed() < 0) {
          delay(500);
          stepper_setSpeed(250);
        } else {
          stepper_setSpeed(90);
        }
      }
      stepper_loop();
      delay(100);
      Serial.println(stepper_getSteps());
    }

    stepper_disable();
    

    servo_ausschieber.enable();
    servo_falltuer.enable();

    Serial.println("Stoppe band und kette. Schließe falltuer...");

    // close falltuer
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_ZU, SERVO_FALLTUER_SPEED_ZU, true);
    

    Serial.println("Falltuer geschlossen. Ausschieber raus und rein...");

    ausschieben();

    Serial.println("Sequence done. Waiting for another button press...");

    delay(500);

    motor_band.sequence_stop();
  }

  delay(5);

}

