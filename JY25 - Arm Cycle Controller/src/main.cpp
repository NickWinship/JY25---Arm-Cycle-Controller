// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Encoder.h>
#include <Wire.h>

// DEFINITIIONS ---------------------------------------------------------
#define INDEX_PULSE 21
#define ENCODER_PIN_A 23
#define ENCODER_PIN_B 22
#define LED_PIN 13

/* PIN DEFINITIONS */
#define RT_REF_PIN PIND3
#define LT_REF_PIN PIND4
#define JOYSTICK_REF_PIN PIND5

#define LB_PIN PIND7
#define RB_PIN PIND6
#define JOYSTICK_X_PIN 9
#define JOYSTICK_Y_PIN 8

#define MAX_DUTY_CYCLE 65

/* CONSTANTS */
#define PPR 192.0              // (48ppr x 4 = 192)(quadrature)
#define PPR_ARM_CYCLE PPR*3
#define ONE_SECOND_MICRO  1000000.00    // (1sec in microseconds)
#define MAX_CPS 3.0 
#define MAX_TILT 5.0

// ENCODER OBJECTS ------------------------------------------------------
Encoder ArmCycle(ENCODER_PIN_A, ENCODER_PIN_B);
elapsedMicros sinceEncoderUpdate;

// IMU OBJECTS ----------------------------------------------------------
Adafruit_MPU6050 mpu;

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
 * @param direction The joystick direction. Center is 0, right is 1 and left is 2.
 */
void SetJoystickVector(unsigned char dutyCycle, unsigned char direction = 0) {
    // check whether a valid direction was specified
    if(direction < 0 || direction > 2) return;

    // cap the duty cycle at the maximum value
    if(dutyCycle > MAX_DUTY_CYCLE) 
        dutyCycle = MAX_DUTY_CYCLE;

    // set the duty cycle according to the direction specified:

    // CENTERED
    if(direction == 0) {
        analogWrite(JOYSTICK_X_PIN, 0);
        analogWrite(JOYSTICK_Y_PIN, 0);
    }

    // RIGHT
    else if(direction == 1) {
        analogWrite(JOYSTICK_X_PIN, (int)dutyCycle);
        analogWrite(JOYSTICK_Y_PIN, 0);
    }

    // LEFT
    else {
        analogWrite(JOYSTICK_X_PIN, 0);
        analogWrite(JOYSTICK_Y_PIN, (int)dutyCycle);
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

    // place the joystick in the centre position
    SetJoystickVector(0, 0);

    // output 1.8V out of the analog reference voltage pins
    analogWrite(RT_REF_PIN, MAX_DUTY_CYCLE);
    analogWrite(LT_REF_PIN, MAX_DUTY_CYCLE);
    analogWrite(JOYSTICK_REF_PIN, MAX_DUTY_CYCLE);

    // put the the joystick in the center position
    analogWrite(JOYSTICK_X_PIN, 0);
    analogWrite(JOYSTICK_Y_PIN, 0);

    // set Teensy 4.0 pins
    pinMode(ENCODER_PIN_A, INPUT);  // encoder channel A
    pinMode(ENCODER_PIN_B, INPUT);  // encoder channel B
    pinMode(LED_PIN, OUTPUT);           // on board LED

    // initialize serial communication at 115200 bits per second
    Serial.begin(115200);

    // attempt to initialize the MPU
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");

    }
    else {
        Serial.println("MPU6050 Found!");

        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        Serial.print("Accelerometer range set to: ");
        switch (mpu.getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case MPU6050_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case MPU6050_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case MPU6050_RANGE_16_G:
            Serial.println("+-16G");
            break;
        }

        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        Serial.print("Gyro range set to: ");
        switch (mpu.getGyroRange()) {
        case MPU6050_RANGE_250_DEG:
            Serial.println("+- 250 deg/s");
            break;
        case MPU6050_RANGE_500_DEG:
            Serial.println("+- 500 deg/s");
            break;
        case MPU6050_RANGE_1000_DEG:
            Serial.println("+- 1000 deg/s");
            break;
        case MPU6050_RANGE_2000_DEG:
            Serial.println("+- 2000 deg/s");
            break;
        }

        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.print("Filter bandwidth set to: ");
        switch (mpu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ:
            Serial.println("260 Hz");
            break;
        case MPU6050_BAND_184_HZ:
            Serial.println("184 Hz");
            break;
        case MPU6050_BAND_94_HZ:
            Serial.println("94 Hz");
            break;
        case MPU6050_BAND_44_HZ:
            Serial.println("44 Hz");
            break;
        case MPU6050_BAND_21_HZ:
            Serial.println("21 Hz");
            break;
        case MPU6050_BAND_10_HZ:
            Serial.println("10 Hz");
            break;
        case MPU6050_BAND_5_HZ:
            Serial.println("5 Hz");
            break;
        }

        Serial.println("");
        delay(100);
    }

}

/**
 * @brief Main execution loop.
 * 
 */
void loop() {
    
    long newEncoderPosition;
    double PPS, armCycleCPS;
    unsigned char cyclingDutyCycle;
    bool isCycleDirectionForward;
    unsigned char steeringDutyCycle;
    unsigned char SteeringDirection;

    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    // calculate the duty cycle and direction of the joystick based on steering position
    // RIGHT
    if(a.acceleration.y < -1){
        SteeringDirection = 1;
        steeringDutyCycle = (abs(a.acceleration.y) / MAX_TILT) * MAX_DUTY_CYCLE;
    }

    // LEFT
    else if(a.acceleration.y > 1){
        SteeringDirection = 2;
        steeringDutyCycle = (abs(a.acceleration.y) / MAX_TILT) * MAX_DUTY_CYCLE;
    }

    // CENTER
    else{
        SteeringDirection = 0;
        steeringDutyCycle = 0;
    }

    // output voltage and direction representing joystick position to controller
    SetJoystickVector(steeringDutyCycle, SteeringDirection);

    // read the encoder position
    newEncoderPosition = ArmCycle.read();

    if (newEncoderPosition != encoderPosition) {

        // Pulses per second
        PPS = ((double)newEncoderPosition - (double)encoderPosition)*ONE_SECOND_MICRO / (double)sinceEncoderUpdate;
        
        // calculate the speed in cycles per second
        armCycleCPS = PPS / (PPR*3); // It's roughly 3x encoder per arm cycle revolution

        // determine the cycling direction
        isCycleDirectionForward = armCycleCPS < 0 ? false : true;

        // calculate the level of trigger depression using the current CPS
        cyclingDutyCycle = (abs(armCycleCPS) / MAX_CPS) * MAX_DUTY_CYCLE;

        // update new position
        encoderPosition = newEncoderPosition;

        // set the duty cycle of the respective pin
        SetCrankingSpeedDirection(cyclingDutyCycle, isCycleDirectionForward);
        
        // Check Serial Port for proper functionality
        /*Serial.print(' ');
        Serial.print(cyclingDutyCycle);
        Serial.print(' ');
        Serial.println(isCycleDirectionForward);*/

        // Reset time between encoder increments
        sinceEncoderUpdate = 0;
    }

}