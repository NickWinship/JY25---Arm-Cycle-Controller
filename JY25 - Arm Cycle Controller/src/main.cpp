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

// TIMER OBJECTS -------------------------------------------------------
IntervalTimer encoderTimer;
IntervalTimer speedTimer;

// GLOBAL VARIABELS ----------------------------------------------------

// encoder variables
volatile int lastState, aState, bState;

// position / increment variables
volatile float encoderAngle, degPerPulse;

// velocity variables
volatile float lastAngle, angularVelocity, averageVelocity, sum, CyclesPerSecond; 
volatile float velocityData[VELOCITIES_TO_AVERAGE];
volatile int averageCount;

//-----------------------------------------------------------------------
/* Function:  Increment or Decrement Encoder Count - ISR
 * --------------------
 * Reads increment or decrement of the encoder signal to 
 * increment or decrement the current angle (using degperpulse)
 * 
 *  returns: void (Modifies global variables)
 */
void get_increment() {
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
void get_speed(){

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
    sum += velocityData[i];
  }
  averageVelocity = sum / VELOCITIES_TO_AVERAGE;
  sum = 0; // reset sum for next average
  

  CyclesPerSecond = averageVelocity / 360.00;

}

//-----------------------------------------------------------------------
/* Function:  Initial setup
 * --------------------
 * Setup function, run once upon startup
 * 
 *  returns: void
 */
void setup() {

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
  encoderTimer.begin(get_increment, ENCODER_READ_TIME); // (time in microseconds)
  speedTimer.begin(get_speed, VELOCITY_READ_TIME); // (time in microsecond)

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
  float angle, speed;

  noInterrupts(); // disable interrupts to read variables
  //angle = encoderAngle;
  speed = CyclesPerSecond;
  interrupts(); // enable interrupts again

  //Serial.print(angle);
  //Serial.print(" ");
  Serial.println(speed);
  delay(100);
}