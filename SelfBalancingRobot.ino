#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30  

MPU6050 mpu;

bool dmpReady = false; 
uint8_t mpuIntStatus; // Holds actual interrupt status byte from MPU
uint8_t devStatus; 
uint16_t packetSize; // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; 
uint8_t fifoBuffer[64]; 


Quaternion q; 
VectorFloat gravity; 
float ypr[3]; 


double originalSetpoint = 183.5; // Target angle for balancing (upright position)
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1; // Small offset to allow movement
double input, output;


double Kp = 52;  
double Kd = 2.5;  // Derivative gain (dampens oscillations)
double Ki = 270;   
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


double motorSpeedFactorLeft = 0.8;
double motorSpeedFactorRight = 0.8;

// Motor Controller Pins
int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; 
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    Serial.begin(115200);

   
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0); // 1688 factory default for the test chip

   
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        
        dmpReady = true;

        
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255); 
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    // If programming failed, Stop the work
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

 
    fifoCount = mpu.getFIFOCount();


    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        
        fifoCount -= packetSize;

    
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

       
        input = ypr[1] * 180 / M_PI + 180;

       
        Serial.print("Input: ");
        Serial.print(input);
        Serial.print(" Output: ");
        Serial.print(output);
        Serial.print(" Setpoint: ");
        Serial.println(setpoint);
    }
}