#ifndef DEF_H
#define DEF_H
#pragma once

// #define TESTING

// #define NODE_1
#define NODE_2
// #define NODE_3
// #define ROBOT_1
// #define ROBOT_2

///common defs 
#define MESH_SSID "stoopidMesh"
#define MESH_PASSWORD "password"
#define MESH_PORT 5555
#define MESH_MODE WIFI_STA

#ifdef NODE_1


#endif



#ifdef ROBOT_1

#define EARTH_GRAVITY_MS2 9.80665                      // Gravitational constant
#define DEG_TO_RAD 0.017453292519943295769236907684886 // Degrees to radians conversion
#define RAD_TO_DEG 57.295779513082320876798154814105   //

#endif

#ifdef ROBOT_2

#define MPU6500_ADDR 0x68
#define EARTH_GRAVITY_MS2 9.80665                      // Gravitational constant
#define DEG_TO_RAD 0.017453292519943295769236907684886 // Degrees to radians conversion
#define RAD_TO_DEG 57.295779513082320876798154814105   //
#define STEPS_PER_REV 200
#define STEPPER_SPEED 60
// Define motor control pins
#define BIA uint8_t{25} // Motor control pin 1 green
#define BIB uint8_t{26} // Motor control pin 2 blue

#define AIA uint8_t{33} // purple A1A
#define AIB uint8_t{32} // ορανγε AIB

// LEDC PWM settings
#define PWM_FREQ uint32_t{5000}
#define PWM_RESOLUTION uint8_t{8} // 8-bit resolution (0-255)

#endif

#ifdef NODE_2

// Stepper motor configuration
#define STEPS_PER_REV 200 // Define the number of steps per revolution for the stepper motor
#define STEPPER_SPEED 60


#endif

#ifdef NODE_3

#endif

#ifdef TESTING
#define STEPS_PER_REV 200
#define STEPPER_SPEED 60
// Define motor control pins
#define BIA uint8_t {25} // Motor control pin 1 green
#define BIB uint8_t {26}  // Motor control pin 2 blue

#define AIA uint8_t {33}   // purple A1A
#define AIB uint8_t {32}    // ορανγε AIB

// LEDC PWM settings
#define PWM_FREQ uint32_t {5000}
#define PWM_RESOLUTION uint8_t {8} // 8-bit resolution (0-255)
#endif 

#endif