// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Encoder.h>

// DEFINITIIONS ---------------------------------------------------------
#define INDEX_PULSE 21
#define ENCODER_PIN_A 23
#define ENCODER_PIN_B 22
#define LED_PIN 13

/* PIN DEFINITIONS */
#define RT_REF_PIN PIND3
#define LT_REF_PIN PIND4
#define JOYSTICK_REF_PIN PIND5

#define LB_PIN PIND6
#define RB_PIN PIND7
#define JOYSTICK_X_PIN 8
#define JOYSTICK_Y_PIN 9

#define MAX_DUTY_CYCLE 65
#define JOYSTICK_CENTER_DUTY_CYCLE 26
#define JOYSTICK_LEFT_MAX_DUTY_CYCLE 23
#define JOYSTICK_RIGHT_MIN_DUTY_CYCLE 29

/* CONSTANTS */
#define PPR 192.0              // (48ppr x 4 = 192)(96ppr x 4 = 384)(quadrature)
#define PPR_ARM_CYCLE PPR*3
#define ONE_SECOND_MICRO  1000000.00    // (1sec in microseconds)

// TIMER OBJECTS -------------------------------------------------------
IntervalTimer encoderTimer;
IntervalTimer speedTimer;

// ENCODER OBJECTS ------------------------------------------------------
Encoder ArmCycle(ENCODER_PIN_A, ENCODER_PIN_B);
elapsedMicros sinceEncoderUpdate;

// GLOBAL VARIABLES -----------------------------------------------------
volatile long encoderPosition  = -999;

/**
 * @brief Sets the duty cycle of the forward or backwards movement pins.
 * 
 * @param dutyCycle The target duty cycle
 * @param isForward Flag indicating whether the target cranking direction is forward
 */
void SetCrankingSpeedDirection(unsigned char dutyCycle, bool isForward = true) {
    // cap the duty cycle at the maximum value
    if(dutyCycle > MAX_DUTY_CYCLE) 
        dutyCycle = MAX_DUTY_CYCLE;

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

/**
 * @brief Set the direction and magnitude of the simulated joystick's movement.
 * 
 * @param dutyCycle The target duty cycle
 * @param direction The joystick direction. Center is 0, left is 1 and right is 2.
 */
void SetJoystickVector(unsigned char dutyCycle, unsigned char direction = 0) {
    // check whether a valid direction was specified
    if(direction < 0 || direction > 2) return;

    // cap the duty cycle at the maximum value
    if(dutyCycle > MAX_DUTY_CYCLE) 
        dutyCycle = MAX_DUTY_CYCLE;

    // set the duty cycle according to the direction specified
    if(direction == 0) {
        analogWrite(JOYSTICK_X_PIN, (int)JOYSTICK_CENTER_DUTY_CYCLE);
    }
    else if(direction == 1) {
        // enforce the upper bound of the joystick x-axis left voltage range
        if(dutyCycle > JOYSTICK_LEFT_MAX_DUTY_CYCLE) 
            dutyCycle = JOYSTICK_LEFT_MAX_DUTY_CYCLE;

        analogWrite(JOYSTICK_X_PIN, (int)dutyCycle);
    }
    else {
        // enforce the lower bound of the
        if(dutyCycle < JOYSTICK_RIGHT_MIN_DUTY_CYCLE)
            dutyCycle = JOYSTICK_RIGHT_MIN_DUTY_CYCLE;
        
        analogWrite(JOYSTICK_X_PIN, dutyCycle);
    }
}

/**
 * @brief Setup function that runs once upon startup
 * 
 */
void setup() {

    // configure output pins
    pinMode(RT_REF_PIN, OUTPUT);
    pinMode(LT_REF_PIN, OUTPUT);
    pinMode(JOYSTICK_REF_PIN, OUTPUT);

    pinMode(RB_PIN, OUTPUT);
    pinMode(LB_PIN, OUTPUT);
    pinMode(JOYSTICK_X_PIN, OUTPUT);
    pinMode(JOYSTICK_Y_PIN, OUTPUT);
    
    // set the initial cranking speed to 0
    SetCrankingSpeedDirection(0, true);

    // output 1.8V out of the analog reference voltage pins
    analogWrite(RT_REF_PIN, MAX_DUTY_CYCLE);
    analogWrite(LT_REF_PIN, MAX_DUTY_CYCLE);
    analogWrite(JOYSTICK_REF_PIN, MAX_DUTY_CYCLE);

    // put the the joystick in the center position
    analogWrite(JOYSTICK_X_PIN, JOYSTICK_CENTER_DUTY_CYCLE);
    analogWrite(JOYSTICK_Y_PIN, JOYSTICK_CENTER_DUTY_CYCLE);

    // set Teensy 4.0 pins
    pinMode(ENCODER_PIN_A, INPUT);  // encoder channel A
    pinMode(ENCODER_PIN_B, INPUT);  // encoder channel B
    pinMode(LED_PIN, OUTPUT);           // on board LED

    // initialize serial communication at 115200 bits per second
    Serial.begin(115200);
}

/**
 * @brief Main execution loop.
 * 
 */
void loop() {

    // to write an analog voltage out, we can use
    // analogWrite(uint8_t pin ,int dutyCycle)
    // where dutyCycle is a value from 0 (0%) to 255 (100%)
    // Since the teensy outputs 3.3V (https://www.pjrc.com/teensy/techspecs.html), to obtain 1.8V we need D = 0.5454, which would be approx dutyCycle = 139
    
    // if(Serial.available() > 0) {
    //     int newDutyCycle;

    //     newDutyCycle = Serial.parseInt();

    //     if(newDutyCycle > 0) {
    //         inDutyCycle = newDutyCycle > MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : newDutyCycle;
    //         Serial.println("You entered " + String(inDutyCycle) + ". Max is " + String(MAX_DUTY_CYCLE) + ".");
    //         SetCrankingSpeedDirection((unsigned char)inDutyCycle, true);
    //     }
    // }
    // else {
    //     Serial.println("The current duty cycle is " + String(inDutyCycle) + ".");
    // }
    
    long newEncoderPosition;
    double PPS, CPS, armCycleCPS;

    newEncoderPosition = ArmCycle.read();

    if (newEncoderPosition != encoderPosition) {

        // Pulses per second
        PPS = ((double)newEncoderPosition - (double)encoderPosition)*ONE_SECOND_MICRO / (double)sinceEncoderUpdate;
        
        // Cycles per second
        CPS = PPS / PPR;
        armCycleCPS = PPS / (PPR*3); // It's roughly 3x encoder per arm cycle revolution

        // update new position
        encoderPosition = newEncoderPosition;

        // output to terminal
        /*Serial.print("Position = ");
        Serial.print(newEncoderPosition);
        Serial.print(", Time = ");
        Serial.print(sinceEncoderUpdate);
        Serial.print(", PPS = ");
        Serial.print(PPS);
        Serial.print(", CPS = ");
        Serial.print(CPS); */
        Serial.print("armCycleCPS = ");
        Serial.print(armCycleCPS);
        Serial.println();

        // Reset time between encoder increments
        sinceEncoderUpdate = 0;
    }
}