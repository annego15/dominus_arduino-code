
#include <TMCStepper.h>
#include "configuration.h"
#include "custom-motor/custom-motor.h"
#include "custom-servo/custom-servo.h"

TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

CustomMotor motor_band(MOTOR_KETTE_PIN_DIRECTION, MOTOR_KETTE_PIN_BRAKE, MOTOR_KETTE_PIN_SPEED, true);

CustomServo servo_falltuer(SERVO_FALLTUER_PIN, SERVO_FALLTUER_POS_ZU);
CoupledServo servo_ausschieber(SERVO_AUSSCHIEBER_PIN1, SERVO_AUSSCHIEBER_PIN2, SERVO_AUSSCHIEBER_POS_NORMAL, true, -5);

unsigned long last_stall_time = 0;
bool reverse_stepper = true;

bool button_pressed = false;

bool switch_falltuer_pressed = false;
bool switch_ausschieber_pressed = false;

unsigned long debounce_switch_falltuer = 0;
unsigned long debounce_switch_ausschieber = 0;


void stall_isr() {
  if (reverse_stepper && millis() - last_stall_time > 500) {
    reverse_stepper = false;
    driver.shaft(false);
    last_stall_time = millis();
  }
}

ISR(TIMER2_COMPA_vect){
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void setup() {

  pinMode(STEP_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Setup serial
  Serial.begin(9600);


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

  pinMode(STALL_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);

  SPI.begin();                   


  driver.begin();

  driver.shaft(reverse_stepper);

  CHOPCONF_t chopconf{0};
  chopconf.tbl = 0b01;
  chopconf.toff = 0;
  chopconf.intpol = true;
  chopconf.hend = 2;
  chopconf.hstrt = 0;
  driver.CHOPCONF(chopconf.sr);

  driver.rms_current(STEPPER_CURRENT, 0.5);
  driver.microsteps(16);
  driver.iholddelay(10);
  driver.TPOWERDOWN(128); // ~2s until driver lowers to hold current

  driver.en_pwm_mode(0);

  PWMCONF_t pwmconf{0};
  pwmconf.pwm_freq = 0b01; // f_pwm = 2/683 f_clk
  pwmconf.pwm_autoscale = true;
  pwmconf.pwm_grad = 5;
  pwmconf.pwm_ampl = 180;
  driver.PWMCONF(pwmconf.sr);

  driver.GSTAT(); // Clear GSTAT

  // stallguard setup
  driver.TCOOLTHRS(0xFFFFF);
  driver.diag1_stall(true);
  driver.sgt(STALL_VALUE);

  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_isr, FALLING);

  cli();//stop interrupts
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  OCR2A = 200;// = (16*10^6) / (1*1024) - 1 (must be <65536) // working spee: 200
  // turn on CTC mode
  TCCR2B |= (1 << WGM21);
  // Set CS11 bits for 8 prescaler
  TCCR2B |= (11 << CS20);// | (1 << CS10);
  // enable timer compare interrupt
  //TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts


  Serial.print("Driver connection test: ");
  Serial.println(driver.test_connection());

  Serial.print("DRV_STATUS=0b");
  Serial.println(driver.DRV_STATUS(), BIN);

  Serial.println("Falltuer geschlossen. Ausschieber raus und rein...");

    // ausschieber raus
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_DRAUSSEN, SERVO_AUSSCHIEBER_SPEED_RAUS);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    // wait
    delay(100);

    // ausschieber zurück
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_NORMAL, SERVO_AUSSCHIEBER_SPEED_REIN);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    servo_ausschieber.disable();



  //motor_band.sequence_start(255, 2000, 1000);
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
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_OFFEN, SERVO_FALLTUER_SPEED_AUF);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    delay(100);
    servo_falltuer.disable();

    Serial.println("Falltuer offen. Bewege band und kette für 20 sekunden...");

    // move band
    motor_band.sequence_start(255, 3500, 1000);

    driver.toff(3);

    cli();
    TIMSK2=0x02;
    sei();

    // wait
    unsigned long start_time = millis();
    while(millis() - start_time < 30000) {
      motor_band.loop();

      if (!reverse_stepper && millis() - last_stall_time > 1000) {
        driver.shaft(true);
        reverse_stepper = true;
        last_stall_time = millis();
      }
      delay(1);
    }

    motor_band.sequence_stop();

    driver.toff(0);
    
    cli();
    TIMSK2=0x00;
    sei();

    servo_ausschieber.enable();
    servo_falltuer.enable();

    Serial.println("Stoppe band und kette. Schließe falltuer...");

    // close falltuer
    servo_falltuer.moveTo(SERVO_FALLTUER_POS_ZU, SERVO_FALLTUER_SPEED_ZU);
    while(servo_falltuer.getTransition()) {
      servo_falltuer.loop();
      delay(1);
    }

    

    Serial.println("Falltuer geschlossen. Ausschieber raus und rein...");

    

    // ausschieber raus
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_DRAUSSEN, SERVO_AUSSCHIEBER_SPEED_RAUS);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    servo_falltuer.disable();

    // wait
    delay(100);

    // ausschieber zurück
    servo_ausschieber.moveTo(SERVO_AUSSCHIEBER_POS_NORMAL, SERVO_AUSSCHIEBER_SPEED_REIN);
    while(servo_ausschieber.getTransition()) {
      servo_ausschieber.loop();
      delay(1);
    }

    delay(500);

    servo_ausschieber.disable();

    Serial.println("Sequence done. Waiting for another button press...");
  }

  delay(5);

}

