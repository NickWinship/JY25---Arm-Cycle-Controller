#include <Arduino.h>

#define RB_PIN PIND3
#define LB_PIN PIND2
#define ANALOG_REF_PIN PIND4

#define MAX_TRIGGER_DUTY_CYCLE 65

volatile int inDutyCycle = 0;

void SetCrankingSpeedDirection(unsigned char dutyCycle, bool isForward = true) {
    // cap the duty cycle at the maximum value
    if(dutyCycle > MAX_TRIGGER_DUTY_CYCLE) 
        dutyCycle = MAX_TRIGGER_DUTY_CYCLE;

    if(isForward) {
        // output voltage to pin connected to RB trigger 
        analogWrite(LB_PIN, 0); 
        analogWrite(RB_PIN, (int)dutyCycle);
    }
    else {
        // output voltage to pin connected to LB trigger
        analogWrite(RB_PIN, 0);
        analogWrite(LB_PIN, (int)dutyCycle);
    }
}

void setup() {
    Serial.begin(9600);

    // configure output pins
    pinMode(RB_PIN, OUTPUT);
    pinMode(LB_PIN, OUTPUT);
    pinMode(ANALOG_REF_PIN, OUTPUT);

    digitalWrite(RB_PIN, HIGH);
    
    SetCrankingSpeedDirection(0, true);

    // output 1.8V out of the analog reference voltage pin
    analogWrite(ANALOG_REF_PIN, MAX_TRIGGER_DUTY_CYCLE);
}

void loop() {
    // to write an analog voltage out, we can use
    // analogWrite(uint8_t pin ,int dutyCycle)
    // where dutyCycle is a value from 0 (0%) to 255 (100%)
    // Since the teensy outputs 3.3V (https://www.pjrc.com/teensy/techspecs.html), to obtain 1.8V we need D = 0.5454, which would be approx dutyCycle = 139
    
    if(Serial.available() > 0) {
        int newDutyCycle;

        newDutyCycle = Serial.parseInt();

        if(newDutyCycle > 0) {
            inDutyCycle = newDutyCycle > MAX_TRIGGER_DUTY_CYCLE ? MAX_TRIGGER_DUTY_CYCLE : newDutyCycle;
            Serial.println("You entered " + String(inDutyCycle) + ". Max is " + String(MAX_TRIGGER_DUTY_CYCLE) + ".");
            SetCrankingSpeedDirection((unsigned char)inDutyCycle, true);
        }
    }
    else {
        Serial.println("The current duty cycle is " + String(inDutyCycle) + ".");
    }
    
    delay(500);
}