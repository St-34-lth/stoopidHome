#include "globals.h"
#include "def.h"

#pragma once
JsonDocument sensorData;
uint32_t rootNodeId;
uint8_t meshChannel = 4;


#ifdef ROBOT_1

//  // MPU control and status variables  MPU6050 mpu; // MPU6050 instance for 6-axis motion tracking
//  uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
//  uint8_t devStatus;      // Return status after each MPU operation (0 = success, non-zero = error)
//  uint16_t packetSize;    // Expected DMP (Digital Motion Processor) packet size (default is 42 bytes)
//  uint16_t fifoCount;     // Count of all bytes currently in the FIFO (First In, First Out buffer)
//  uint8_t fifoBuffer[64]; // FIFO storage buffer

// // Orientation and motion variables from MPU6050
//  Quaternion q;        // Quaternion container (for orientation)
//  VectorInt16 aa;      // Accelerometer sensor measurements (x, y, z)
//  VectorInt16 gg;      // Gyroscope sensor measurements (x, y, z)
//  VectorInt16 aaWorld; // Accelerometer measurements in world-frame (x, y, z)
//  VectorInt16 ggWorld; // Gyroscope measurements in world-frame (x, y, z)
//  VectorFloat gravity; // Gravity vector (x, y, z)
//  float euler[3];      // Euler angles (psi, theta, phi)
//  float ypr[3];        // Yaw, pitch, roll angles

// // Structure to hold motor commands (direction and speed)
// struct motorCmd
// {
//     int dirA;   // Direction for motor A
//     int dirB;   // Direction for motor B
//     int motorA; // Speed for motor A
//     int motorB; // Speed for motor B
// };
// // Pin definitions for motor control
//  const int pwmMotorA; // PWM control for motor A
//  const int pwmMotorB; // PWM control for motor B
//  const int dirMotorA; // Direction control for motor A
//  const int dirMotorB; // Direction control for motor B

// // Motor speed constants
//  const int motorSpeed;       // Max motor speed value for PWM
//  const int minRange;         // Minimum range for PWM speed control
//  const int maxRange;         // Maximum range for PWM speed control
//  const int minSpeed;         // Minimum speed for the motor
//  const int maxSpeed;         // Maximum speed for the motor
//  const int noSpeed;          // No speed (motors stopped)
//  int sda, sdc;               // I2C pins for MPU6050
//  int href, vsync, clk, sdat; // Placeholder for al variables (not fully defined here)
#endif

#ifdef ROBOT_2

#define MPU6500_ADDR 0x68
 
 //sounder vars 
 const int triggerPin = 27;
 const int echoPin = 34;

 //motor vars 
uint8_t resolution = 8;
uint16_t freq = 5000; 
//imu vars 
 int sda; 
 int sdc;  // 22 / 21
               // MPU control and status variables  MPU6050 mpu; // MPU6050 instance for 6-axis motion tracking
//     uint8_t mpuIntStatus; // Holds actual interrupt status byte from MPU
//  uint8_t devStatus;      // Return status after each MPU operation (0 = success, non-zero = error)
//  uint16_t packetSize;    // Expected DMP (Digital Motion Processor) packet size (default is 42 bytes)
//  uint16_t fifoCount;     // Count of all bytes currently in the FIFO (First In, First Out buffer)
//  uint8_t fifoBuffer[64]; // FIFO storage buffer

//  // Orientation and motion variables from MPU6050
//  Quaternion q;        // Quaternion container (for orientation)
//  VectorInt16 aa;      // Accelerometer sensor measurements (x, y, z)
//  VectorInt16 gg;      // Gyroscope sensor measurements (x, y, z)
//  VectorInt16 aaWorld; // Accelerometer measurements in world-frame (x, y, z)
//  VectorInt16 ggWorld; // Gyroscope measurements in world-frame (x, y, z)
//  VectorFloat gravity; // Gravity vector (x, y, z)
//  float euler[3];      // Euler angles (psi, theta, phi)
//  float ypr[3];        // Yaw, pitch, roll angles

 // Structure to hold motor commands (direction and speed)
//  struct motorCmd
//  {
//      int dirA;   // Direction for motor A
//      int dirB;   // Direction for motor B
//      int motorA; // Speed for motor A
//      int motorB; // Speed for motor B
//  };
 // Pin definitions for motor control


 // Motor speed constants
//  const int motorSpeed;       // Max motor speed value for PWM
//  const int minRange;         // Minimum range for PWM speed control
//  const int maxRange;         // Maximum range for PWM speed control
//  const int minSpeed;         // Minimum speed for the motor
//  const int maxSpeed;         // Maximum speed for the motor
//  const int noSpeed;          // No speed (motors stopped)

//  int href, vsync, clk, sdat; // Placeholder for al variables (not fully defined here)
#endif

#ifdef NODE_1

 // LED pins for node1
 int rPin = D2;              // Red LED pin
 int gPin = D3;              // Green LED pin
 int bPin = D4;              // Blue LED pin
 int fireSensorAnlgPin = A0; // Analog pin for fire sensor
 int fireSensorDigPin = 1;   // Digital pin for fire sensor
 static const int DHT_SENSOR_PIN = 16;
 DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE); // DHT sensor instance

#endif

#ifdef NODE_2
 int waterSensorAnlgPin = A0; //
 
 // Motor control pins
 const int IN1 = 14U;
 const int IN2 = 12U;
 const int IN3 = 13U;
 const int IN4 = 15U;
 int relayPin = D4;
#endif

#ifdef TESTING_ESP8266
 int waterSensorAnlgPin = A0; //

 // Motor control pins
 const int IN1 = D5;
 const int IN2 = D6;
 const int IN3 = D7;
 const int IN4 = D8;
 int relayPin = D4;

 // LED pins for node1
 int rPin = D2;              // Red LED pin
 int gPin = D3;              // Green LED pin
 int bPin = D4;              // Blue LED pin
 int fireSensorAnlgPin = A0; // Analog pin for fire sensor
 int fireSensorDigPin = 1;   // Digital pin for fire sensor
 static const int DHT_SENSOR_PIN = 16;
 DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE); // DHT sensor instance
 // MPU control and status variables  MPU6050 mpu; // MPU6050 instance for 6-axis motion tracking
 uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
 uint8_t devStatus;      // Return status after each MPU operation (0 = success, non-zero = error)
 uint16_t packetSize;    // Expected DMP (Digital Motion Processor) packet size (default is 42 bytes)
 uint16_t fifoCount;     // Count of all bytes currently in the FIFO (First In, First Out buffer)
 uint8_t fifoBuffer[64]; // FIFO storage buffer

 // Orientation and motion variables from MPU6050
 Quaternion q;        // Quaternion container (for orientation)
 VectorInt16 aa;      // Accelerometer sensor measurements (x, y, z)
 VectorInt16 gg;      // Gyroscope sensor measurements (x, y, z)
 VectorInt16 aaWorld; // Accelerometer measurements in world-frame (x, y, z)
 VectorInt16 ggWorld; // Gyroscope measurements in world-frame (x, y, z)
 VectorFloat gravity; // Gravity vector (x, y, z)
 float euler[3];      // Euler angles (psi, theta, phi)
 float ypr[3];        // Yaw, pitch, roll angles

 // Structure to hold motor commands (direction and speed)
 struct motorCmd
 {
     int dirA = 0;   // Direction for motor A
     int dirB = 0;   // Direction for motor B
     int motorA = 0; // Speed for motor A
     int motorB = 0; // Speed for motor B
};
// Pin definitions for motor control
const int pwmMotorA = D1; // PWM control for motor A
const int pwmMotorB = D2; // PWM control for motor B
const int dirMotorA = D3; // Direction control for motor A
const int dirMotorB = D4; // Direction control for motor B

// Motor speed constants
const int motorSpeed = 1023; // Max motor speed value for PWM
const int minRange = 312;    // Minimum range for PWM speed control
const int maxRange = 712;    // Maximum range for PWM speed control
const int minSpeed = 450;    // Minimum speed for the motor
const int maxSpeed = 1020;   // Maximum speed for the motor
const int noSpeed = 0;       // No speed (motors stopped)
int sda, sdc;               // I2C pins for MPU6050
int href, vsync, clk, sdat; // Placeholder for al variables (not fully defined here)

#ifdef TESTING_ESP32


#endif 


#endif 