#ifndef MOTORS_H
#define MOTORS_H
#pragma once
#include <stoopidHome.h>
void newConnectionCb(uint32_t nodeId);
void changedConnectionCb();
void nodeTimeAdjustedCb(int offset);
void onRcvCb(uint32_t from, String &msg);
void onDroppedConnectionCb(uint32_t nodeId);
const int motorSpeed = 1023; // Max motor speed value for PWM
const int minRange = 312;    // Minimum range for PWM speed control
const int maxRange = 712;    // Maximum range for PWM speed control
const int minSpeed = 450;    // Minimum speed for the motor
const int maxSpeed = 1020;   // Maximum speed for the motor
const int noSpeed = 0;
const int pwmMotorA = D1; // PWM control for motor A
const int pwmMotorB = D2; // PWM control for motor B
const int dirMotorA = D3; // Direction control for motor A
const int dirMotorB = D4; // Direction control for motor B
// Structure to hold motor commands (direction and speed)
struct esp8266motorCmd
{
    int dirA;   // Direction for motor A
    int dirB;   // Direction for motor B
    int motorA; // Speed for motor A
    int motorB; // Speed for motor B
};
int setupMotors();

int killMotors();

int moveForward();

int stayNeutral();

int moveRight();

int moveLeft();

int moveBackward();

#endif 