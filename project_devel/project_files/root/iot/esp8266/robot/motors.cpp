#include "motors.h"


void newConnectionCb(uint32_t nodeId) {

};

void changedConnectionCb()
{
    Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCb(int offset) {
    // Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
};

void onRcvCb(uint32_t from, String &msg) {

};

void onDroppedConnectionCb(uint32_t nodeId)
{
    Serial.printf("Connection Dropped: %u\n", nodeId);
}
// Function to set up the motor shield (dual motor control)
int setupMotors()
{
    // Initialize motor control pins as outputs
    pinMode(pwmMotorA, OUTPUT);
    pinMode(pwmMotorB, OUTPUT);
    pinMode(dirMotorA, OUTPUT);
    pinMode(dirMotorB, OUTPUT);

    // Initialize motor pins to a neutral (stopped) state
    analogWrite(pwmMotorA, noSpeed);
    analogWrite(pwmMotorB, noSpeed);
    analogWrite(dirMotorA, noSpeed);
    analogWrite(dirMotorB, noSpeed);
    millis();

    // Set the motors to be stopped
    digitalWrite(dirMotorA, LOW);
    digitalWrite(dirMotorB, LOW);
    digitalWrite(pwmMotorA, LOW);
    digitalWrite(pwmMotorB, LOW);

    Serial.println("Motor Shield 12E Initialized");
    return 0;
}

// Function to stop (kill) both motors
int killMotors()
{
    digitalWrite(pwmMotorA, LOW);
    digitalWrite(pwmMotorB, LOW);
    if (digitalRead(pwmMotorA) == LOW && digitalRead(pwmMotorB) == LOW)
    {
        return 0;
    }
    return -1; // Return error if motors aren't stopped
}

// Function to stop all motor movement
int stayNeutral()
{
    // Set motor speeds and directions to zero
    analogWrite(pwmMotorA, noSpeed);
    analogWrite(pwmMotorB, noSpeed);
    digitalWrite(pwmMotorA, LOW);
    digitalWrite(pwmMotorB, LOW);
    digitalWrite(dirMotorA, LOW);
    digitalWrite(dirMotorB, LOW);

    if (digitalRead(dirMotorA) == LOW && digitalRead(dirMotorB) == LOW &&
        digitalRead(pwmMotorA) == LOW && digitalRead(pwmMotorB) == LOW)
    {
        return 0;
    }
    return -1; // Return error if any motor is not neutral
}

// Function to move the robot forward
int moveForward()
{
    digitalWrite(dirMotorA, HIGH);
    digitalWrite(dirMotorB, LOW);
    analogWrite(pwmMotorA, maxSpeed);
    analogWrite(pwmMotorB, maxSpeed);

    if (digitalRead(dirMotorA) == HIGH && digitalRead(dirMotorB) == LOW)
    {
        return 0;
    }
    return -1; // Return error if forward movement isn't detected
}

// Function to move the robot backward
int moveBackward()
{
    digitalWrite(dirMotorA, LOW);
    digitalWrite(dirMotorB, HIGH);
    analogWrite(pwmMotorA, maxSpeed);
    analogWrite(pwmMotorB, maxSpeed);

    if (digitalRead(dirMotorA) == LOW && digitalRead(dirMotorB) == HIGH)
    {
        return 0;
    }
    return -1; // Return error if backward movement isn't detected
}

// Function to turn the robot left
int moveLeft()
{
    digitalWrite(dirMotorA, LOW);
    digitalWrite(dirMotorB, LOW);
    analogWrite(pwmMotorA, maxSpeed);
    analogWrite(pwmMotorB, maxSpeed);

    if (digitalRead(dirMotorA) == LOW && digitalRead(dirMotorB) == LOW)
    {
        return 0;
    }
    return -1; // Return error if left turn isn't detected
}

// Function to turn the robot right
int moveRight()
{
    digitalWrite(dirMotorA, HIGH);
    digitalWrite(dirMotorB, HIGH);
    analogWrite(pwmMotorA, maxSpeed);
    analogWrite(pwmMotorB, maxSpeed);

    if (digitalRead(dirMotorA) == HIGH && digitalRead(dirMotorB) == HIGH)
    {
        return 0;
    }
    return -1; // Return error if right turn isn't detected
}
