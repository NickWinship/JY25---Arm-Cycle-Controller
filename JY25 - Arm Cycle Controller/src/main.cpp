// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Encoder.h>

// DEFINITIIONS ---------------------------------------------------------
#define INDEX_PULSE 21
#define Encoder_A 23
#define Encoder_B 22
#define LED_Pin 13

#define PPR 192.0              // (48ppr x 4 = 192)(96ppr x 4 = 384)(quadrature)
#define PPR_ARM_CYCLE PPR*3
#define ONE_SECOND  1000000.00    // (1sec in microseconds)

// ENCODER OBJECTS ------------------------------------------------------
Encoder ArmCycle(Encoder_A, Encoder_B);

// TIMER OBJECTS --------------------------------------------------------
elapsedMicros sinceEncoderUpdate;

// GLOBAL VARIABELS -----------------------------------------------------
long position  = -999;

//-----------------------------------------------------------------------
/* Function:  Initial setup
 * --------------------
 * Setup function, run once upon startup
 * 
 * returns: void
 */
void setup() {
  // Initialize pins
  pinMode(LED_Pin, OUTPUT);      // on board LED

  // Initialize Communication
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
  long newPosition;
  double PPS, CPS,CPS_armcycle;
  newPosition = ArmCycle.read();

  if (newPosition != position) {
    // Pulses per second
    PPS = ((double)newPosition - (double)position)*ONE_SECOND / (double)sinceEncoderUpdate;
    
    // Cycles per second
    CPS = PPS/PPR;
    CPS_armcycle = PPS/(PPR*3); // It's roughly 3x encoder per arm cycle revolution

    // update new position
    position = newPosition;

    // output to terminal
    /*Serial.print("Position = ");
    Serial.print(newPosition);
    Serial.print(", Time = ");
    Serial.print(sinceEncoderUpdate);
    Serial.print(", PPS = ");
    Serial.print(PPS);
    Serial.print(", CPS = ");
    Serial.print(CPS); */
    Serial.print(", CPS_armcycle = ");
    Serial.print(CPS_armcycle);
    Serial.println();

    // Reset time between encoder increments
    sinceEncoderUpdate = 0;
  }
}