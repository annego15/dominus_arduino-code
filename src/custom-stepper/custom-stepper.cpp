#include "custom-stepper.h"

TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

bool running = false;
bool reverse = REVERSE_STEPPER;
bool direction = reverse;

volatile long step_count = 0;
volatile uint8_t speed = 50;

unsigned long last_stall_time = 0;

//ISR(TIMER2_OVF_vect){


ISR(TIMER2_COMPA_vect) {
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  if (stepper_getDirection()) {
    step_count++;
  } else {
    step_count--;
  }
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  if (step_count == 0) {
    running = false;
    stepper_stop();
  }
}

void stepper_setup() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(STALL_PIN, INPUT_PULLUP);
    SPI.begin(); 
    driver.begin();
    driver.shaft(reverse);
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
    attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallISR, FALLING);


    noInterrupts(); //stop interrupts

    TCCR2A = 0;// set entire TCCR1A register to 0
    TCCR2B = 0;// same for TCCR1B
    TCNT2  = 0;//initialize counter value to 0
    OCR2A = 200;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR2A|=(1<<WGM01);
    // Set CS11 bits for 8 prescaler
    TCCR2B|=(1<<CS21);    //Set the prescale 1/64 clock
    TCCR2B|=(1<<CS20);
    // enable timer compare interrupt
    //TIMSK1 |= (1 << OCIE1A);

    /*TCCR0A = 0;
    TCCR0B = 0;
    TCCR0A|=(1<<WGM01); 
    OCR0A=5;
    TCCR0B |= (1 << CS01); //| (1 << CS00);// | (1 << CS10);
    TCNT0 = 0;

    

    TCCR2A = 0;// set entire TCCR1A register to 0
    TCCR2B = 0;// same for TCCR1B
    TCNT2  = speed;//initialize counter value to 0
    //OCR2A = 200;// = (16*10^6) / (1*1024) - 1 (must be <65536) // working speed: 200
    // turn on CTC mode
    //TCCR2B |= (1 << WGM21);
    // Set CS11 bits for 8 prescaler
    TCCR2B |= (1 << CS21) | (1 << CS20);// | (1 << CS10);
    // enable timer compare interrupt
    //TIMSK2 |= (1 << OCIE2A);*/
    interrupts(); //allow interrupts


    Serial.print("Driver connection test: ");
    Serial.println(driver.test_connection());

    Serial.print("DRV_STATUS=0b");
    Serial.println(driver.DRV_STATUS(), BIN);

}

void stepper_enable() {
    running = true;
    driver.toff(3);
}

void stepper_disable() {
    running = false;
    driver.toff(0);
}

void enable_stall_detection() {
    driver.diag1_stall(true);
}

void disable_stall_detection() {
    driver.diag1_stall(false);
}

void stepper_loop() {
    if (stepper_getDirection() && (millis() - last_stall_time > 1500)) {
        stepper_setDirection(false);
        last_stall_time = millis();
    }
}

long stepper_getSteps() {
    return step_count;
}


bool stepper_getDirection() {
    return !direction != !reverse;
}

void stepper_setDirection(bool dir) {
    direction = !dir != !reverse;
    driver.shaft(direction);
}

bool stepper_getRunning() {
    return running;
}

void stallISR() {
  if (!stepper_getDirection() && millis() - last_stall_time > 50) {
    stepper_setDirection(true);
    last_stall_time = millis();
    Serial.println("Stall!");
  }
}

void stepper_move(long steps, int speed_) {
    if (steps == 0) {
        return;
    }
    speed = speed_;
    noInterrupts();
    step_count = steps;
    OCR2A = speed; // set compare to speed
    TIMSK2|=(1<<OCIE2A); // enable timer interrupts
    interrupts();
    stepper_enable();
    if (steps > 0) {
        stepper_setDirection(true);
    } else {
        stepper_setDirection(false);
    }
}

void stepper_setSGT(int8_t sgt) {
    driver.sgt(sgt);
}

void stepper_setSpeed(int speed_) {
    speed = speed_;
    noInterrupts();
    OCR2A = speed; // set compare to speed
    interrupts();
}

void stepper_stop() {
    step_count = 0;
    stepper_disable();
    noInterrupts();
    TIMSK2^=(1<<OCIE2A); // disable timer interrupts
    interrupts();
}

void stepper_pause() {
    stepper_disable();
    noInterrupts();
    TIMSK2^=(1<<OCIE2A); // disable timer interrupts
    interrupts();
}

void stepper_resume() {
    stepper_enable();
    noInterrupts();
    TIMSK2|=(1<<OCIE2A); // enable timer interrupts
    interrupts();
}



