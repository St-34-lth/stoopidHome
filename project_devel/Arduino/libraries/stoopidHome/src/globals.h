#ifndef GLOBALS_H
#define GLOBALS_H
#pragma once

#include <Arduino.h> 
#include <ArduinoJson.h>
// #include <ArduinoOTA.h>
#include <stoopidHome.h>
#include "def.h"

// common

extern JsonDocument sensorData;
extern uint32_t rootNodeId;
extern uint8_t meshChannel;
// extern const  size_t MAX_TASKS ;
// extern const  size_t MAX_SENSORS ;

#ifdef ROBOT_1

// MPU control and status variables extern MPU6050 mpu; // MPU6050 instance for 6-axis motion tracking
// extern uint8_t mpuIntStatus;                           // Holds actual interrupt status byte from MPU
// extern uint8_t devStatus;                              // Return status after each MPU operation (0 = success, non-zero = error)
// extern uint16_t packetSize;                            // Expected DMP (Digital Motion Processor) packet size (default is 42 bytes)
// extern uint16_t fifoCount;                             // Count of all bytes currently in the FIFO (First In, First Out buffer)
// extern uint8_t fifoBuffer[64];                         // FIFO storage buffer

// // Orientation and motion variables from MPU6050
// extern Quaternion q;        // Quaternion container (for orientation)
// extern VectorInt16 aa;      // Accelerometer sensor measurements (x, y, z)
// extern VectorInt16 gg;      // Gyroscope sensor measurements (x, y, z)
// extern VectorInt16 aaWorld; // Accelerometer measurements in world-frame (x, y, z)
// extern VectorInt16 ggWorld; // Gyroscope measurements in world-frame (x, y, z)
// extern VectorFloat gravity; // Gravity vector (x, y, z)
// extern float euler[3];      // Euler angles (psi, theta, phi)
// extern float ypr[3];        // Yaw, pitch, roll angles

// // Structure to hold motor commands (direction and speed)
// struct motorCmd
// {
//     int dirA;   // Direction for motor A
//     int dirB;   // Direction for motor B
//     int motorA; // Speed for motor A
//     int motorB; // Speed for motor B
// };
// Pin definitions for motor control
// extern const int pwmMotorA ; // PWM control for motor A
// extern const int pwmMotorB ; // PWM control for motor B
// extern const int dirMotorA ; // Direction control for motor A
// extern const int dirMotorB ; // Direction control for motor B

// // Motor speed constants
// extern const int motorSpeed;       // Max motor speed value for PWM
// extern const int minRange ;          // Minimum range for PWM speed control
// extern const int maxRange ;          // Maximum range for PWM speed control
// extern const int minSpeed ;          // Minimum speed for the motor
// extern const int maxSpeed ;          // Maximum speed for the motor
// extern const int noSpeed ;               // No speed (motors stopped)
// extern int sda, sdc;               // I2C pins for MPU6050
// extern int href, vsync, clk, sdat; // Placeholder for external variables (not fully defined here)
#endif


#ifdef ROBOT_2
#include <MPU6500_WE.h>

// MPU control and status variables extern MPU6050 mpu;

extern const int triggerPin;
extern const int echoPin;
extern int sda, sdc; // I2C pins for MPU6050

#endif 


#ifdef NODE_1
#include "dht_nonblocking.cpp"
// LED pins for node1
extern int rPin ;        // Red LED pin
extern int gPin ;         // Green LED pin
extern int bPin;       // Blue LED pin
extern int fireSensorAnlgPin ; // Analog pin for fire sensor
extern int fireSensorDigPin ;    // Digital pin for fire sensor
extern int fireSensorValue ;
extern int waterSensorAnlgPin; //
#define DHT_SENSOR_TYPE DHT_TYPE_11 // Define DHT sensor type

#endif 

#ifdef NODE_2
#include "Stepper.h"
// Motor control pins
extern const int IN1 ;
extern const int IN2 ;
extern const int IN3 ;
extern const int IN4 ;
extern int relayPin ;
extern int waterSensorAnlgPin;
#endif 



#endif 


#ifdef TESTING
#pragma once
#include "dht_nonblocking.cpp"
#include <Stepper.h>
#include <Arduino.h>
#include <MPU6500_WE.h>
#include <Wire.h>
extern JsonDocument sensorData;
extern const int IN1;
extern const int IN2;
extern const int IN3;
extern const int IN4;
extern int relayPin;
extern int waterSensorAnlgPin;

// LED pins for node1
extern int rPin;              // Red LED pin
extern int gPin;              // Green LED pin
extern int bPin;              // Blue LED pin
extern int fireSensorAnlgPin; // Analog pin for fire sensor
extern int fireSensorDigPin;  // Digital pin for fire sensor
extern int fireSensorValue;
extern int waterSensorAnlgPin;      //
#define DHT_SENSOR_TYPE DHT_TYPE_11 // Define DHT sensor type

#include <MPU6050_6Axis_MotionApps20.h>
// MPU control and status variables extern MPU6050 mpu; // MPU6050 instance for 6-axis motion tracking
extern uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
extern uint8_t devStatus;      // Return status after each MPU operation (0 = success, non-zero = error)
extern uint16_t packetSize;    // Expected DMP (Digital Motion Processor) packet size (default is 42 bytes)
extern uint16_t fifoCount;     // Count of all bytes currently in the FIFO (First In, First Out buffer)
extern uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation and motion variables from MPU6050
extern Quaternion q;        // Quaternion container (for orientation)
extern VectorInt16 aa;      // Accelerometer sensor measurements (x, y, z)
extern VectorInt16 gg;      // Gyroscope sensor measurements (x, y, z)
extern VectorInt16 aaWorld; // Accelerometer measurements in world-frame (x, y, z)
extern VectorInt16 ggWorld; // Gyroscope measurements in world-frame (x, y, z)
extern VectorFloat gravity; // Gravity vector (x, y, z)
extern float euler[3];      // Euler angles (psi, theta, phi)
extern float ypr[3];        // Yaw, pitch, roll angles

// // Structure to hold motor commands (direction and speed)
// struct motorCmd
// {
//     int dirA;   // Direction for motor A
//     int dirB;   // Direction for motor B
//     int motorA; // Speed for motor A
//     int motorB; // Speed for motor B
// };
// Pin definitions for motor control
extern const int pwmMotorA; // PWM control for motor A
extern const int pwmMotorB; // PWM control for motor B
extern const int dirMotorA; // Direction control for motor A
extern const int dirMotorB; // Direction control for motor B

// Motor speed constants
extern const int motorSpeed;       // Max motor speed value for PWM
extern const int minRange;         // Minimum range for PWM speed control
extern const int maxRange;         // Maximum range for PWM speed control
extern const int minSpeed;         // Minimum speed for the motor
extern const int maxSpeed;         // Maximum speed for the motor
extern const int noSpeed;          // No speed (motors stopped)
extern int sda, sdc;               // I2C pins for MPU6050
extern int href, vsync, clk, sdat; // Placeholder for external variables (not fully defined here)


#endif