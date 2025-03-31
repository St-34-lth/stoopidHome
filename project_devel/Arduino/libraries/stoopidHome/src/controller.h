#ifndef CONTROLLER_H

#define CONTROLLER_H

#pragma once
#include "def.h"
#include "globals.h"

struct iFaceState
{
    const uint8_t *pins;
    size_t pinsArrSize; 
    const uint8_t *discreteStates;
    size_t discArrSize;
    const uint8_t *contStates;
    size_t contArrSize; 

    // Default Constructor
    iFaceState()
        : pins(nullptr), pinsArrSize(0),
          discreteStates(nullptr), discArrSize(0),
          contStates(nullptr), contArrSize(0) {}

    // Parameterized Constructor
    iFaceState(const uint8_t *p, size_t pSize,
               const uint8_t *d, size_t dSize,
               const uint8_t *c, size_t cSize)
        : pins(p), pinsArrSize(pSize),
          discreteStates(d), discArrSize(dSize),
          contStates(c), contArrSize(cSize) {}

 
    iFaceState &operator=(const iFaceState &other)
    {
        if (this != &other)
        {
            pins = other.pins;
            pinsArrSize = other.pinsArrSize;
            discreteStates = other.discreteStates;
            discArrSize = other.discArrSize;
            contStates = other.contStates;
            contArrSize = other.contArrSize;
        }
        return *this;
    }
};

// Base interface
class HwInterface
{
    public:
        virtual ~HwInterface() = default;

        // Converted to accept iFaceState
        virtual bool setup(iFaceState &states) = 0;
        virtual bool control(iFaceState &states) = 0;
        bool checkPinState(const uint8_t& pin)
        {
            int state = digitalRead(pin); // Read the pin state
            
            if (state == HIGH)
            {
                Serial.println("HIGH (pull-up active)");
                return true;
            }
            else 
            {
                
                Serial.println("LOW (pin pulled low or connected externally)");
                return false;
            }
            
        };

        
};
#ifdef NODE_1

enum ledColor
{
    RED,
    BLUE,
    GREEN,
    WHITE,
}; 

class LedIface : public HwInterface
{
    public:

           
        LedIface(){};
        bool setup(iFaceState& states) override
        {       
            _rPin = states.pins[0]; _gPin=  states.pins[1]; _bPin= states.pins[2];

            pinMode(_rPin, OUTPUT); // Set red pin as output
            pinMode(_gPin, OUTPUT); // Set green pin as output
            pinMode(_bPin, OUTPUT); // Set blue pin as output

            // Turn off the LED initially
            digitalWrite(_rPin, LOW);
            digitalWrite(_gPin, LOW);
            digitalWrite(_bPin, LOW);
            _rVal,_bVal,_gVal = 0;

            if (digitalRead(_rPin) == LOW && digitalRead(_gPin) == LOW && digitalRead(_bPin) == LOW)
            {
                return true;// Return success if LED is off
            }
            return false; // Return error if LED setup failed
        };
        bool control(iFaceState& states)
        {
            // reset();
            // Iterate over the pins array
            for (size_t i=0; i < states.pinsArrSize; i++)
            {
                auto pin = states.pins[i];
                auto value = states.contStates[i];
                
                if (value >= 0 && value <= 255)
                    {
                        // If pin matches one of the RGB pins, analogWrite() its corresponding contState
                        if (pin == _rPin)
                        {
                            analogWrite(_rPin, value);
                            _rVal = value;
                        }
                        else if (pin == _gPin)
                        {
                            analogWrite(_gPin, value);
                            _gVal = value;
                        }
                        else if (pin == _bPin)
                        {
                            analogWrite(_bPin, value);
                            _bVal = value;
                        }
                }
                else 
                {
                    Serial.printf("Value out of range for pin: %u", pin);
                    return false;
                }
                
            }
          
            return true;
        };
        
        bool shiftColor(ledColor color)
        {
            reset();
            switch(color)
            {
                
                case(RED):
                {
                    analogWrite(_rPin,255);
                    _rVal= 255; 
                    return true;
                }
                case (BLUE):
                {
                    analogWrite(_bPin, 255);
                    _bVal = 255;
                    return true;
                }
                case (GREEN):
                {
                    analogWrite(_gPin, 255);
                    _gVal = 255;
                    return true;
                }
                case (WHITE):
                {
                    analogWrite(_bPin, 255);
                    analogWrite(_gPin, 255);
                    analogWrite(_rPin, 255);
                    _gVal = 255;
                    _rVal = 255;
                    _rVal = 255;
                    return true;
                }
            };
            return false; 
        };
        bool isOn()
        {
            if (_rVal >0  ||
                _gVal > 0 ||
                 _bVal > 0)
            {
                
                return true;
            }
            return false;
        };
        bool reset()
        {
           _rVal = 0;
           _bVal = 0;
           _gVal = 0;
            digitalWrite(_rPin, LOW);
            digitalWrite(_gPin, LOW);
            digitalWrite(_bPin, LOW);
            return true; 
        };

    private: 
        uint8_t _rPin,_gPin,_bPin; 
        uint8_t _rVal,_gVal,_bVal;
};

#endif

enum stepperPos
{
    OPEN,
    CLOSE
};
#ifdef NODE_2
class StepperIface: public HwInterface 
{
    public:
        StepperIface(uint16_t steps_per_rev, uint8_t speed) : _steps_per_rev(steps_per_rev),
                                                              _speed(speed),
                                                              _stepCounter(0),
                                                              _stepCounterTotal(0) {}

        bool setup(iFaceState &states) override
        {
            _IN1 = states.pins[0];
            _IN2 = states.pins[1];
            _IN3 = states.pins[2];
            _IN4 = states.pins[3];
           
            _stepper = new Stepper(_steps_per_rev, _IN1, _IN3, _IN2, _IN4);

            _stepper->setSpeed(_speed); // Set the speed of the stepper motor to 60 RPM
            
            return true;
        };
        bool control(iFaceState &states) override
        {
            if (states.discArrSize < 2)
            {
                return false; // Ensure there are at least two discrete states
            }

            int direction = states.discreteStates[0]; // 1 = CW, 0 = CCW
            int move = states.discreteStates[1];      // 1 = Move, 0 = Stop

            if (!move)
            {
                _stepper->step(0); // Stop the stepper motor
                return true;
            }

            int stepCount = _steps_per_rev / 4 ;// Move stepper by small increments (e.g., 36 steps for 360-degree stepper)

            if (direction==1)
            {
                _stepper->step(stepCount); // Move forward (CW)
            }
            else
            {
                _stepper->step(-stepCount); // Move backward (CCW)
            }

            return true;
        }

        int setStepperToPreset(stepperPos &state) 
        {
            switch(state)
            {
                case(stepperPos::OPEN):
                {
                    if(_stepCounter == 200)
                    {
                        return _stepCounter;  
                    }
                   else if(_stepCounter == 0)
                    {
                        _stepper->step(_steps_per_rev);
                        _stepCounterTotal = _stepCounterTotal + _steps_per_rev;
                        _stepCounter = _steps_per_rev;
                        return _stepCounter;
                    }
                };
                case(stepperPos::CLOSE):
                {
                    if (_stepCounter == 200)
                    {
                        _stepper->step(-_steps_per_rev);
                        _stepCounterTotal = _stepCounterTotal + _stepCounter;
                        _stepCounter = 0;
                        return _stepCounter;
                    }
                    else if (_stepCounter == 0)
                    {
                        return _stepCounter;
                    }
                };
               
            }

            return 0;
        };
        bool setSteps(uint16_t stepsperrev)
        {
            _steps_per_rev = stepsperrev;
            return true ;
        }

        bool setSpeed(uint8_t speed)
        {
            if (speed > 0)
            {
                _speed = speed; 
                return true;
            }
            else return false;  
        };
        uint32_t getStepsMoved()
        {
            return _stepCounter;
        }
       
        private:
            uint8_t _IN1,_IN2,_IN3,_IN4;
            Stepper *_stepper;
            uint16_t _steps_per_rev;
            uint8_t _speed ;
            int _stepCounter;
            int _stepCounterTotal;

};

class PumpInterface : public HwInterface
{
public:
    // Constructor: Initialize with the relay pin number.
    PumpInterface(){};

    // Public method to start the pump.
    // NOTE: the pump needs to be stopped manually
    int startPump()
    {
        digitalWrite(_relayPin, HIGH); // Activate relay to start pump
        Serial.print("Pump started on relay pin: ");
        Serial.println(_relayPin);
        return 0;
    }

    // Public method to stop the pump.
    int stopPump()
    {
        digitalWrite(_relayPin, LOW); // Deactivate relay to stop pump
        Serial.print("Pump stopped on relay pin: ");
        Serial.println(_relayPin);
        return 0;
    }

    // Setup the pump interface by configuring the relay pin as an OUTPUT.
    virtual bool setup(iFaceState &states) override
    {
        
        if (states.pins == nullptr || states.pinsArrSize == 0)
        {
            Serial.println("PumpInterface setup error: No pins provided in iFaceState!");
            return false;
        }
        // Use the first pin in the array as the relay pin.
        _relayPin = states.pins[0];
        pinMode(_relayPin, OUTPUT);
        digitalWrite(_relayPin, LOW); // Ensure pump is off initially
        Serial.print("PumpInterface setup complete on relay pin: ");
        Serial.println(_relayPin);
        return true;
    }

    // Control the pump based on the discrete state (assumed at index 0).
    // For example, if states.discreteStates[0] equals 1, start the pump; if 0, stop it.
    //NOTE: the pump needs to be stopped manually
    virtual bool control(iFaceState &states) override
    {
        if (states.discArrSize > 0 && states.discreteStates != nullptr)
        {
            if (states.discreteStates[0] == 1)
            {
                startPump();
            }
            else
            {
                stopPump();
            }
            return true;
        }
        Serial.println("No pump command received in discreteStates.");
        return false;
    }

protected:
    uint8_t _relayPin; // The relay pin used to control the pump.
};

#endif 

#if defined(ROBOT_2)
enum motorCmds
{
    FORWARD,
    REVERSE,
    STOP
};

class MotorIface : public HwInterface
{
public:
    /**
     * Constructor for a single DC motor using two pins (H-bridge driver)
     * and the ESP32 LEDC for PWM speed control.
     *
     * @param resolution PWM resolution in bits (e.g. 8 => range 0-255)
     * @param freq       PWM frequency in Hz
     */
    MotorIface(uint8_t resolution, uint32_t freq) : _max_speed(0),
                                                    _resolution(resolution),
                                                    _freq(freq)
    {
        _max_speed = 255;
        _currentSpeed = 0;
        _currentCmd = STOP;
        _default_speed = 200;
        _noSpeed = 1U;
    }

    /**
     * Called once at startup to configure PWM channels and pins.
     * We'll read from iFaceState if you prefer, or just use constructor values.
     */
    bool setup(iFaceState &state) override
    {
        _pin1 = state.pins[0];
        _pin2 = state.pins[1];

        // Configure each LEDC channel
        pinMode(_pin1, OUTPUT);
        pinMode(_pin2, OUTPUT);


        // Attach the pins for this motor
        if(ledcAttach(_pin1, _freq, _resolution))
        {
            Serial.println("attaching pins");
            ledcAttach(_pin2, _freq, _resolution);

            Serial.printf("pin1: %u pin2: %u",_pin1,_pin2);
            // Optionally, set them both to 0 duty at the start
            ledcWrite(_pin1, 0);
             ledcWrite(_pin2,0);
            // digitalWrite(_pin2,LOW);
            return true;
        }
        

        return false; 
    }

    /**
     * Called periodically (or on demand) to update motor states based on
     * iFaceState or internal logic. The user might set:
     *   - a discrete state for the motor command (e.g. FORWARD)
     *   - a continuous state for speed (0.._max_speed)
     */
    bool control(iFaceState &state) override
    {
        uint8_t speedPin1 = state.contStates[0];
        uint8_t speedPin2 = state.contStates[1];
        
        if(ledcWrite(_pin1, speedPin1) &&ledcWrite(_pin2, speedPin2))
        {
            return true; 
        } 
        
        return false; 
    }

    bool setPresetControl(motorCmds &cmd)
    {
        _currentCmd = cmd;
        Serial.printf("using default speed %u: ",_default_speed, _max_speed);
        switch (cmd)
        {
            case FORWARD:
            {
                ledcWrite(_pin1, _default_speed);
                ledcWrite(_pin2, _noSpeed);
                _currentSpeed = _default_speed;
                return true;
            }
            case REVERSE:
            {
                ledcWrite(_pin1, _noSpeed);
                ledcWrite(_pin2, _default_speed);
                _currentSpeed = _default_speed;
                return true;
            }
            case STOP:
            {  
                ledcWrite(_pin1, _noSpeed);
                ledcWrite(_pin2, _noSpeed);
                _currentSpeed = 0;
                return true;
            }
        }
        return false;
    }

    bool setDefaultSpeed(uint8_t newSpeed)
    {
        _default_speed = newSpeed; 
        return true; 
    }
    
    bool setFrequency(uint32_t newFreq, uint8_t newResolution= 8)
    {
        _freq = newFreq;
        if(newResolution != _resolution)
        {
            _resolution = newResolution;
        } 

        if(ledcChangeFrequency(_pin1,_freq,_resolution) && ledcChangeFrequency(_pin2, _freq, _resolution))
        {return true; }
        else 
        return false; 
    }

private:
    // Motor pins & parameters
    uint8_t _pin1, _pin2;
    uint16_t _default_speed; // e.g. 255 if 8-bit, 4095 if 12-bit, etc.
    uint8_t _resolution; // e.g. 8 for 8-bit
    uint32_t _freq;      // e.g. 5000 Hz
    uint8_t _max_speed;
    uint8_t _noSpeed;
    // Current state
    uint16_t _currentSpeed; // last set speed 0.._max_speed
    motorCmds _currentCmd;  // last direction command
};
#endif 

#endif