// INCLUDED LIBRARIES
#include <Arduino.h>


// DEFINITIIONS --------------------------------------------------------
#define INDEX_PULSE 21
#define Encoder_A 23
#define Encoder_B 22
#define LED_Pin 13
#define PPR 400.0                 // (400 pulses per revolution)
#define ENCODER_READ_TIME 100     // (0.1ms)
#define VELOCITY_READ_TIME 100000 // (100ms)
#define VELOCITIES_TO_AVERAGE 20
#define ONE_SECOND  1000000       // (1sec in microseconds)

#define RB_PIN PIND3
#define LB_PIN PIND2
#define ANALOG_REF_PIN PIND4

#define MAX_TRIGGER_DUTY_CYCLE 65

// TIMER OBJECTS -------------------------------------------------------
IntervalTimer encoderTimer;
IntervalTimer speedTimer;

// GLOBAL VARIABELS ----------------------------------------------------

// encoder variables
volatile int lastState, aState, bState;

// position / increment variables
volatile float encoderAngle, degPerPulse;

// velocity variables
volatile float lastAngle, angularVelocity, averageVelocity, velocitySum, cyclesPerSecond; 
volatile float velocityData[VELOCITIES_TO_AVERAGE];
volatile int averageCount;

// output voltage duty cycle variable
volatile unsigned char inDutyCycle = 0;

//-----------------------------------------------------------------------
/* Function:  Increment or Decrement Encoder Count - ISR
 * --------------------
 * Reads increment or decrement of the encoder signal to 
 * increment or decrement the current angle (using degperpulse)
 * 
 *  returns: void (Modifies global variables)
 */
void GetIncrement() {
    aState = digitalRead(Encoder_A);
    bState = digitalRead(Encoder_B);

    // Check for rising edge of A
    if((aState != lastState) && (aState == HIGH)){

        // Check if B is HIGH or LOW
        if(bState == LOW){
            encoderAngle += degPerPulse;  // increment counter
            digitalWrite(LED_Pin, HIGH);
        }
        else{
            encoderAngle -= degPerPulse;  // decrtement counter
            digitalWrite(LED_Pin, LOW);
        }
  }

  // reset last state
  lastState = aState;
}

//-----------------------------------------------------------------------
/* Function:  Calculate Rotational Velocity - ISR
 * --------------------
 * Calculates a new angular velocity for the last 100ms
 * and averages the last 10 values
 * 
 *  returns: void (Modifies global variables)
 */
void GetSpeed(){

    // Calculate new angular velocity for last 100ms
    angularVelocity = (encoderAngle - lastAngle) / (0.1); // Change in time = 100ms

    // update last angle with new value
    lastAngle = encoderAngle;

    // add new velocity to averaging array 
    velocityData[averageCount] = angularVelocity;

    // determine new array position
    averageCount ++;
    if(averageCount >= VELOCITIES_TO_AVERAGE){
        averageCount = 0;
    }

    // average velocities in the array
    for(int i = 0; i < VELOCITIES_TO_AVERAGE; i++){
        velocitySum += velocityData[i];
    }
    averageVelocity = velocitySum / VELOCITIES_TO_AVERAGE;
    velocitySum = 0; // reset sum for next average
    
    cyclesPerSecond = averageVelocity / 360.00;
}


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

//-----------------------------------------------------------------------
/* Function:  Initial setup
 * --------------------
 * Setup function, run once upon startup
 * 
 *  returns: void
 */
void setup() {

    // configure output pins
    pinMode(RB_PIN, OUTPUT);
    pinMode(LB_PIN, OUTPUT);
    pinMode(ANALOG_REF_PIN, OUTPUT);

    digitalWrite(RB_PIN, HIGH);
    
    SetCrankingSpeedDirection(0, true);

    // output 1.8V out of the analog reference voltage pin
    analogWrite(ANALOG_REF_PIN, MAX_TRIGGER_DUTY_CYCLE);

    // set Teensy 4.0 pins:
    pinMode(Encoder_A, INPUT);  // encoder channel A
    pinMode(Encoder_B, INPUT);  // encoder channel B
    pinMode(LED_Pin, OUTPUT);           // on board LED

    // initialize global variabls:
    encoderAngle = 0.0;                 // initialize encoder angle
    lastAngle = 0.0;                    // initialize encoder angle
    averageCount = 0.0;                 // initialize the average count for velocity
    lastState = digitalRead(Encoder_A); // get starting state of encoder
    degPerPulse = 360.0 / PPR;            // calculate deg/pulse from PPR

    // set up interrupt service routines:
    encoderTimer.begin(GetIncrement, ENCODER_READ_TIME); // (time in microseconds)
    speedTimer.begin(GetSpeed, VELOCITY_READ_TIME); // (time in microsecond)

    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);
}

//-----------------------------------------------------------------------
/* Function:  MAIN LOOP
 * --------------------
 * Main loop of code
 * 
 *  returns: void
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
    //         inDutyCycle = newDutyCycle > MAX_TRIGGER_DUTY_CYCLE ? MAX_TRIGGER_DUTY_CYCLE : newDutyCycle;
    //         Serial.println("You entered " + String(inDutyCycle) + ". Max is " + String(MAX_TRIGGER_DUTY_CYCLE) + ".");
    //         SetCrankingSpeedDirection((unsigned char)inDutyCycle, true);
    //     }
    // }
    // else {
    //     Serial.println("The current duty cycle is " + String(inDutyCycle) + ".");
    // }
    
    // delay(500);
    
    float angle, speed;

    noInterrupts(); // disable interrupts to read variables
    //angle = encoderAngle;
    speed = cyclesPerSecond;
    interrupts(); // enable interrupts again

    //Serial.print(angle);
    //Serial.print(" ");
    Serial.println(speed);
    delay(100);
}