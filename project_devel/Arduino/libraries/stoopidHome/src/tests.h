#include <AUnit.h>
#include <Arduino.h>
#include "controller.h"
#include "Sensor.h"

using namespace aunit;


// Define the LedTestFixture class
// const uint8_t pins[] = {rPin, gPin, bPin};
// const uint8_t continuousStates[] = {0, 0, 0};
// const uint8_t discreteStates[] = {0, 0, 0};

// iFaceState ledState = {
//     .pins = pins,
//     .pinsArrSize = sizeof(pins) / sizeof(pins[0]),
//     .discreteStates = discreteStates,
//     .discArrSize = sizeof(discreteStates) / sizeof(discreteStates[0]),
//     .contStates = continuousStates,
//     .contArrSize = sizeof(continuousStates) / sizeof(continuousStates[0])};

// class LedTestFixture : public TestOnce
// {
// public:
//     LedTestFixture() {}

//     LedIface led;

// protected:
//     // Optional: Add setup and teardown methods
//     void setup() override
//     {
//         TestOnce::setup();
//         led.setup(ledState);
//         // Setup code for each test
//     }

//     void teardown() override
//     {
//        led.reset();
//        TestOnce::teardown();
//        // Teardown code for each test
//     }

//     // Optional: Add helper methods for tests
//     void helperMethod()
//     {
//         // Example helper method
//     }
// };

// // Define tests using the testF macro
// testF(LedTestFixture, checkLedStatusTestOff)
// {
    
//     assertFalse(led.isOn());
// }

// testF(LedTestFixture, checkLedStatusTestOn)
// {

//     led.shiftColor(ledColor::WHITE);
//     assertTrue(led.isOn());
// }

// testF(LedTestFixture, ColorControlTest)
// {

    
//     // Test red channel
//     uint8_t redValues[3] = {255, 0, 0};
//     ledState.contStates = redValues;
//     assertTrue(led.control(ledState));
//     assertTrue(led.isOn());
//     delay(500);

   
//     // // Test green channel
//     uint8_t greenValues[3] = {0, 255, 0};
//     ledState.contStates = greenValues;
//     assertTrue(led.control(ledState));
//     assertTrue(led.isOn());
//     delay(500);
   
//     // // Test blue channel
//     uint8_t blueValues[3] = {0, 0, 255};
//     ledState.contStates = blueValues;
//     assertTrue(led.control(ledState));
//     assertTrue(led.isOn());
//     delay(500);
// }

// testF(LedTestFixture, shiftColorTest)
// {
 

//     assertTrue(led.shiftColor(ledColor::RED));
//     delay(1000);
    
//     assertTrue(led.shiftColor(ledColor::BLUE));
//     delay(1000);

    
//     assertTrue(led.shiftColor(ledColor::GREEN));
//     delay(1000);

//     assertTrue(led.shiftColor(ledColor::WHITE));
//     delay(1000);
// }


// testF(LedTestFixture, ResetTest)
// {
//     assertTrue(led.setup(ledState));
//     led.shiftColor(RED);
//     assertTrue(led.reset());
//     assertFalse(led.isOn());
// }

// testF(LedTestFixture, PinVerificationTestLOW)
// {
//     // const char *pinNames[3] = {"Red", "Green", "Blue"};
//     led.reset();
    
//     assertFalse(led.checkPinState(rPin));
//     assertFalse(led.checkPinState(gPin));
//     assertFalse(led.checkPinState(bPin));

//     // This will output verification results to serial
// }

// testF(LedTestFixture, PinVerificationTestHIGH)
// {
//     // const char *pinNames[3] = {"Red", "Green", "Blue"};
//     uint8_t highValues[3] = {255, 255, 255};
//     ledState.contStates = highValues;
//     led.control(ledState);

//     assertTrue(led.checkPinState(rPin));
//     assertTrue(led.checkPinState(gPin));
//     assertTrue(led.checkPinState(bPin));

//     // This will output verification results to serial
// }

// testF(LedTestFixture, InitializationTest) {
//   assertTrue(led.setup(ledState));
//   assertFalse(led.isOn());
// }


// -----------------------------------------------------------------------------
// // or wherever you keep your DHTsensor code
// DHTsensor dht{DHT_SENSOR_PIN};
// class DHTSensorTestFixture : public TestOnce
// {
//     public:
     
        

//     protected:
//         void setup() override
//         {
//             TestOnce::setup();
//         }
//         void teardown() override
//         {
//             TestOnce::teardown();
//         }
// };

// testF(DHTSensorTestFixture, BeginSuccessTest)
// {

//     assertTrue(dht.begin());

// }

// testF(DHTSensorTestFixture, BeginFailureTest)
// {

//     DHTsensor sensor2(5);
//     assertFalse(sensor2.begin());
   
// }

// testF(DHTSensorTestFixture, ReadFailureTest)
// {
//     // Force measure to fail or produce NaN
//     bool result = dht.read();
//     assertEqual(false, result);
//     assertTrue(strcmp(dht.getError(), "Read failed") == 0);
// }

// testF(DHTSensorTestFixture, ReadSuccessTest)
// {
//     // Force measure to fail or produce NaN
//     bool result = dht.read();
//     auto err = dht.getError();
//     assertEqual(false, result);
//     assertTrue((strcmp(err, "Read failed")==0));
// }


// -----------------------------------------------------------------------------
//  3. Fire Sensor Tests
// -----------------------------------------------------------------------------

// FireSensor fire{fireSensorAnlgPin};

// class FireSensorTestFixture : public TestOnce
// {
//     public:
    

//     protected:
//         void setup() override
//         {
//             TestOnce::setup();
//         }
//         void teardown() override
//         {
//             TestOnce::teardown();
//         }
// };

// testF(FireSensorTestFixture, BeginSuccessTest)
// {
//     assertTrue(fire.begin());
//     // assertEqual(nullptr, fire.getError());
// }

// testF(FireSensorTestFixture, BeginFailureTest)
// {
//     FireSensor fire2(10);

//     assertFalse(fire.begin());
//     // assertNotEqual(nullptr, fire.getError());
//     // assertTrue(strcmp(fire.getError(), "verify fire sensor is connected") == 0);
// };

// testF(FireSensorTestFixture, GetDataTest)
// {
//     StaticJsonDocument<128> doc;

//     assertTrue(fire.getData(doc));
//     assertEqual(900, doc[0]["fireValue"].as<int>());
// }

//4. stepper tests



// uint16_t stepsperrev = 200;
// uint8_t speed = 60;


// class StepperTestFixture : public TestOnce
// {
//     public:
//         StepperTestFixture()
//             : stepsperrev(STEPS_PER_REV), speed(STEPPER_SPEED), stepMtr(STEPS_PER_REV, STEPPER_SPEED)
//         {
//             const uint8_t pins[] = {14U, 12U, 13U, 15U};
//             const uint8_t continuousStates[] = {0, 0, 0, 0};
//             const uint8_t discreteStates[] = {0, 0, 0, 0};

         
//             stepState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
//                                    discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
//                                    continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));

//             stepMtr.setup(stepState);
//         }

//     protected:
//         uint16_t stepsperrev;
//         uint8_t speed ;
//         StepperIface stepMtr;
//         iFaceState stepState;

//         void setup() override
//         {
//             TestOnce::setup();
        
//         }
//         void teardown() override
//         {
//             TestOnce::teardown();
//         }
// };

// testF(StepperTestFixture, ControlStepperCW)
// {
//     const uint8_t discreteStates[] = {1, 1, 0, 0};


//     iFaceState testState(stepState.pins, stepState.pinsArrSize,
//                          discreteStates, stepState.discArrSize,
//                          stepState.contStates, stepState.contArrSize);

//     bool result = stepMtr.control(testState);
//     assertTrue(result);
// };
// testF(StepperTestFixture, ControlStepperCCW)
// {
//     const uint8_t discreteStates[] = {0, 1, 0, 0};

//     iFaceState testState(stepState.pins, stepState.pinsArrSize,
//                          discreteStates, stepState.discArrSize,
//                          stepState.contStates, stepState.contArrSize);

//     bool result = stepMtr.control(testState);
//     assertTrue(result);
// };


// testF(StepperTestFixture, MoveStepperOpenThenClose)
// {
//     // From 0 to 200
//     auto posOpen = stepperPos::OPEN;
//     int movedOpen = stepMtr.setStepperToPreset(posOpen);
//     assertEqual(movedOpen, 200);

//     // Now from 200 to 0
//     auto posClose = stepperPos::CLOSE;
//     int movedClose = stepMtr.setStepperToPreset(posClose);
//     assertEqual(movedClose, 0);
// }


// testF(StepperTestFixture, SetStepperSteps)
// {
//     uint16_t steps_per_rev = 50;
//     assertTrue(stepMtr.setSteps(steps_per_rev));
    
// }

// testF(StepperTestFixture, SetStepperSpeed)
// {
//     uint8_t speed = 10;
//     assertTrue(stepMtr.setSpeed(speed));

// };

// pump tests
// uint8_t relayPin = D4;
// class PumpTestFixture : public TestOnce
// {
// public:
//     PumpTestFixture()
//     {
//         // Define arrays for the pump interface.
//         // Use a single pin (for example, pin 5) as the pump relay.
//         static const uint8_t pins[] = {D4,0,0,0};
//         // Initially, the discrete state is 0 (pump off).
//         static uint8_t discreteStates[] = {0,0,0,0};
//         static const uint8_t continuousStates[] = {0,0,0,0};

//         pumpState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
//                                discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
//                                continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));

        
//     }

// protected:
//     PumpInterface pump;   // Instance of our pump interface.
//     iFaceState pumpState; // The state used during setup.

//     void setup() override
//     {
//         TestOnce::setup();
//     }
//     void teardown() override
//     {
//         TestOnce::teardown();
//     }
// };

// testF(PumpTestFixture,SetupPump)
// {
//     assertTrue(pump.setup(pumpState));

// }

// testF(PumpTestFixture, StartStopPump)
// {
//     assertEqual(pump.startPump(),0);
//     assertEqual(pump.stopPump(), 0);
// };


// class MotorTestFixture : public TestOnce
// {
//     public:
//         MotorTestFixture()
//             : resolution(8U),freq(5000U), mtr1(resolution,freq), mtr2(resolution,freq)
//         {

//             static const uint8_t pins[] = {25U, 26U};
//             static const uint8_t continuousStates[] = {0, 0};
//             static const uint8_t discreteStates[] = {0, 0};
//             motorState = iFaceState(pins, sizeof(pins) / sizeof(pins[0]),
//                                 discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
//                                 continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));
//             static const uint8_t pins2[] = {33U, 32U};
//             motorState2 = iFaceState(pins2, sizeof(pins2) / sizeof(pins2[0]),
//                                     discreteStates, sizeof(discreteStates) / sizeof(discreteStates[0]),
//                                     continuousStates, sizeof(continuousStates) / sizeof(continuousStates[0]));
//             mtr1.setup(motorState);
//             mtr2.setup(motorState2);
//         }

//     protected:
     
//         uint32_t freq;
//         uint8_t resolution;
//         MotorIface mtr1;
//         MotorIface mtr2;
//         iFaceState motorState;
//         iFaceState motorState2;
        
//         void setup() override
//         {
          
//             TestOnce::setup();
//         }
//         void teardown() override
//         {
//             TestOnce::teardown();
//         }
// };

// // testF(MotorTestFixture, setupMotors)
// // {
// //     assertTrue(mtr1.setup(motorState));
// //     assertTrue(mtr2.setup(motorState2));
// // };

// testF(MotorTestFixture, controlMotorFWD)
// {

//     const uint8_t continuousStates[] = {125U, 0U};
//     const uint8_t continuousStates2[] = {0, 0};

//     //mtr1
//     motorState2.contStates = continuousStates;
//     assertTrue(mtr1.control(motorState2));
//     delay(1000);
//     motorState2.contStates = continuousStates2;
//     assertTrue(mtr1.control(motorState2));

//     //mtr2
//     motorState2.contStates = continuousStates;
//     assertTrue(mtr2.control(motorState2));
//     delay(1000);
//     motorState2.contStates = continuousStates2;
//     assertTrue(mtr2.control(motorState2));

    

// };
// testF(MotorTestFixture, controlMotorREV)
// {
//     const uint8_t continuousStates[] = {0, 125U};
//     const uint8_t continuousStates2[] = {0, 0};

//     //mtr1
//     motorState2.contStates = continuousStates;
//     assertTrue(mtr1.control(motorState));
//     delay(1000);
//     motorState2.contStates = continuousStates2;
//     assertTrue(mtr1.control(motorState));

//     //mtr2
  
//     motorState2.contStates = continuousStates;
//     assertTrue(mtr2.control(motorState2));

//     delay(1000);
//     motorState2.contStates = continuousStates2;
//     assertTrue(mtr2.control(motorState2));
// };

// testF(MotorTestFixture, testMotorFWD)
// {
//     auto cmd = motorCmds::FORWARD;
//     auto flag = mtr1.setPresetControl(cmd);
//     assertTrue(flag);
//     delay(1000);
//     cmd = motorCmds::STOP;
//     flag = mtr1.setPresetControl(cmd);
//     assertTrue(flag);


//     cmd = motorCmds::FORWARD;
//     flag = mtr2.setPresetControl(cmd);
//     assertTrue(flag);
//     delay(1000);
//     cmd = motorCmds::STOP;
//     flag = mtr2.setPresetControl(cmd);
//     assertTrue(flag);
// };

// testF(MotorTestFixture, testMotorREV)
// {
//     auto cmd = motorCmds::REVERSE;
//     auto flag = mtr1.setPresetControl(cmd);
//     assertTrue(flag);
//     delay(1000);
//     cmd = motorCmds::STOP;
//     flag = mtr1.setPresetControl(cmd);
//     assertTrue(flag);

//     cmd = motorCmds::REVERSE;
//     flag = mtr2.setPresetControl(cmd);
//     assertTrue(flag);
//     delay(1000);
//     cmd = motorCmds::STOP;
//     flag = mtr2.setPresetControl(cmd);
//     assertTrue(flag);
// };

// testF(MotorTestFixture, SetDefaultSpeed)
// {
//     uint8_t newSpeed = 180; // Example speed (should be <= max speed)

//     bool result = mtr1.setDefaultSpeed(newSpeed);

//     assertTrue(result);

  
// }

// testF(MotorTestFixture, SetFrequency)
// {
//     uint32_t newFreq = 7000;    // New frequency


//     bool result = mtr1.setFrequency(newFreq);

//     assertTrue(result);

 
// }



//Ultrasonic sensor tests

// class UltraSonicTestFixture : public TestOnce
// {
//     public:
//         UltraSonicTestFixture() : triggerPin(27), echoPin(34), sr04(triggerPin, echoPin){};

//     protected:
//         const int triggerPin;
//         const int echoPin; 
//         UltraSonicIface sr04;
       
//         void setup() override
//         {

//             TestOnce::setup();
//         }
//         void teardown() override
//         {
//             TestOnce::teardown();
//         }

// };

// testF(UltraSonicTestFixture, setupTest)
// {
//     auto result = sr04.begin();
//     assertTrue(result);
// }

// testF(UltraSonicTestFixture, readTest)
// {
//     assertTrue(sr04.read());
//     assertTrue(sr04.getData(sensorData));
//     assertTrue(sensorData.containsKey("distance")); // Ensure key exists
//     assertMore(sensorData["distance"], 0);   // Check for a valid distance
// }



// using namespace aunit;

// class IMUIfaceTestFixture : public TestOnce
// {
//     public:
//         IMUIfaceTestFixture():imu()
//         {
            
//         } 
//     protected : IMUIface imu;

//         void setup() override
//         {
        
//         }
//         void teardown() override
//         {
            
//         }
// };

// // **Test 1: Initialization**
// testF(IMUIfaceTestFixture, BeginTest)
// {
//     bool initSuccess = imu.begin();
//     assertTrue(initSuccess); // Ensure initialization succeeds
// }

// // **Test 2: Sensor Reading**
// testF(IMUIfaceTestFixture, ReadTest)
// {
//     assertTrue(imu.read()); // Ensure reading returns true
// }

// // **Test 3: Data Retrieval**
// testF(IMUIfaceTestFixture, GetDataTest)
// {

//     assertTrue(imu.read());
//     bool result = imu.getData(sensorData);
//     assertTrue(result); // Ensure data retrieval succeeds
//     assertTrue(sensorData.containsKey("accX"));
//     assertTrue(sensorData.containsKey("accY"));
//     assertTrue(sensorData.containsKey("accZ"));
//     assertTrue(sensorData.containsKey("gyroX"));
//     assertTrue(sensorData.containsKey("gyroY"));
//     assertTrue(sensorData.containsKey("gyroZ"));

//     serializeJson(sensorData, Serial);
// }

// // // **Test 4: Error Handling**
// // testF(IMUIfaceTestFixture, GetErrorTest)
// // {
// //     const char *error = imu.getError();
// //     assertNotEqual(error, nullptr); // Ensure error message exists
// // }

// // **Test 5: Valid Data Range**
// testF(IMUIfaceTestFixture, ValidDataRangeTest)
// {
//     StaticJsonDocument<256> doc;
//     imu.getData(doc);

//     float accX = doc["accX"];
//     float accY = doc["accY"];
//     float accZ = doc["accZ"];
//     float gyroX = doc["gyroX"];
//     float gyroY = doc["gyroY"];
//     float gyroZ = doc["gyroZ"];

//     assertMore(accX, -2.0);
//     assertLess(accX, 2.0);
//     assertMore(accY, -2.0);
//     assertLess(accY, 2.0);
//     assertMore(accZ, -2.0);
//     assertLess(accZ, 2.0);
//     assertMore(gyroX, -250.0);
//     assertLess(gyroX, 250.0);
//     assertMore(gyroY, -250.0);
//     assertLess(gyroY, 250.0);
//     assertMore(gyroZ, -250.0);
//     assertLess(gyroZ, 250.0);
// }

// // // **Test 6: Calibration Test**
// testF(IMUIfaceTestFixture, CalibrationTest)
// {
//     imu.begin();
//     delay(1000); // Wait for calibration
  
//     imu.getData(sensorData);

//     float resultantG = sensorData["accX"].as<float>() + sensorData["accY"].as<float>() + sensorData["accZ"].as<float>();

//     assertMore(resultantG, 0.9); // Expect gravity ~1G
//     assertLess(resultantG, 1.1);
// }



#ifdef TESTING_NETWORKING

    void testSingleMsg(painlessMesh *meshObject, uint32_t targetNodeId)
{
    auto srcNodeId = meshObject->getNodeId();
    String msg{"Test message from: "};
    msg += static_cast<String>(srcNodeId);
    meshObject->sendSingle(targetNodeId, msg);
};

void testBroadCastMsg(painlessMesh *meshObject)
{
    meshObject->sendBroadcast("This is a test broadcasted message");
};

void testNetworkTopology(painlessMesh *meshObject)
{
    // Get the JSON topology
    String topology = meshObject->subConnectionJson();
    // Get the list of connected node IDs
    std::list<uint32_t> nodes = meshObject->getNodeList();

    // Print the JSON topology
    Serial.println("Network Topology (JSON):");
    Serial.println(topology);

    // Print the list of node IDs
    Serial.println("Connected Node IDs:");
    for (auto nodeId : nodes)
    {
        Serial.printf("Testing node:%u ", nodeId);
        testSingleMsg(meshObject, nodeId);
    }
}
#endif 