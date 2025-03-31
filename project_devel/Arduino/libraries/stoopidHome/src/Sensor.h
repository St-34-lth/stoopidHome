#ifndef SENSOR_H
#define SENSOR_H

#pragma once 

#include "globals.h"

// #include <Stepper.h>

// #include <ledcwrite.h>
class Sensor
{
    public:
        
        virtual ~Sensor() = default;
        virtual bool begin() = 0;                 // Initialize sensor
        virtual bool read() = 0;                  // Fetch data
        virtual bool getData(JsonDocument &doc) = 0;        // Return formatted data
        virtual  const char *getError() const = 0; // Error handling
    protected:
        mutable const char *_error = nullptr;
};
#ifdef NODE_1
#include "dht_nonblocking.h"
class DHTsensor : public Sensor
{
    public:
       
        DHTsensor(uint8_t pin) : _pin(pin), _humidity(0.0f), _temp(0.0f), _dht_sensor_type(0)
        {
            _dht_sensor = new DHT_nonblocking(_pin, _dht_sensor_type);
        };

        bool begin() override
        {
            if (!digitalRead(_pin) == HIGH)
            {
                _error = "DHT init failed";
                return false;
            };
            return true;
        };

        bool read() override
        {
            
            DHTsensor::measure(_temp, _humidity, *_dht_sensor);
            if (isnan(_temp) || isnan(_humidity))
            {
                _error = "Read failed";
                return false;
            }
            return true;
        
        };

        bool getData(JsonDocument& doc)  override 
        {
            
            
                doc[0]["temp"] = getTempData();
                doc[0]["humidity"] = getHumidityData();
                return true; 
            
            
        };

        float getTempData() const { return _temp;};
        float getHumidityData() const { return _humidity;};
        const char *getError() const override{return _error;};

        // Helper function to measure environment (temperature and humidity)
        
        static bool measure(float &temperature, float &humidity, DHT_nonblocking &dht_sensor)
        {
            static unsigned long measurement_timestamp = millis();

            if (millis() - measurement_timestamp > 3000ul) // Measure every 3 seconds
            {
                if (dht_sensor.measure(&temperature, &humidity)) // Measure temperature and humidity
                {
                    measurement_timestamp = millis(); // Reset the timestamp after a successful measurement
                    return true;
                }
            }
            return false;
        }

    private:
        DHT_nonblocking *_dht_sensor; // DHT sensor instance
        const int _pin;
        const int _dht_sensor_type;
        float _temp,_humidity;
        

        
};
#endif 

#ifdef NODE_2
class WaterSensor: public Sensor 

{
    public:
        WaterSensor(uint8_t anlgPin) : _anlgPin(anlgPin),
                                       _val(0) {}

        bool begin() override 
        {
            _val = analogRead(_anlgPin);
            if (_val > 50 && _val < 400 )
            {
                return true;
            }
            else
                _error = "verify fire sensor is connected";
            return false;
        }

        bool read() override 
        {
            _val = analogRead(_anlgPin);
            return true;
        }

        const char *getError() const override
        {
            return _error;
        };

        //water level in mm - assuming sensor is vertically placed
        bool getData(JsonDocument &doc) override
        {

            doc[0]["waterLevel"] = _val;
            return true;
        };

    private:
        uint8_t _anlgPin;
        uint32_t _val;
};

class FireSensor: public Sensor
{
    public:
        FireSensor(uint8_t anlgPin):
                                    _anlgPin(anlgPin),
                                    _val(0) {}

        
        bool begin() override 
        {
            _val = analogRead(_anlgPin);
            if(_val > 400 && _val < 1024)
            {
                return true;
            }
            else
                _error = "verify fire sensor is connected";
            return false; 
        };

        bool read() override 
        {
            //no way of knowing whether the sensor is connected since values float.
            _val = analogRead(_anlgPin);
            return true; 
        };

        bool getData(JsonDocument &doc)  override
        {
            
                doc[0]["fireValue"] = _val;
                return true;
            
        };

        const char *getError() const override
        {
            return _error;
        };

    private:
        uint8_t _anlgPin ;
        uint32_t _val;
};

#endif 

#ifdef ROBOT_2
#include <MPU6500_WE.h>
#include "SR04.h"
class UltraSonicIface : public Sensor
{
    public:
        UltraSonicIface(uint8_t trigPin, uint8_t echoPin)
            :   _trigPin(trigPin),
                _echoPin(echoPin),
                _distance(0),
                _sr04(echoPin,trigPin)
        {
            
        }

        bool begin() override
        {
        
            _sr04.Ping();
            if (_sr04.getDistance() > 0 )
            {
                return true;
            }
            else 
            return false;
        }

        bool read() override
        {
        
            _distance = _sr04.DistanceAvg();
            if (_distance <= 0 || _distance > 800) // Check for valid range
            {
                _error = "Invalid reading";
                return false;
            }

            return true;
        }

        bool getData(JsonDocument &doc) override
        {
            doc["distance"] = _distance;
            return true;
        }

        const char *getError() const override
        {
            return _error;
        }

    private:
        uint8_t _trigPin, _echoPin;
        long _distance;
        SR04 _sr04;
};

class IMUIface : public Sensor
{
public:
    IMUIface(int i2cAddress = 0x68) : _imu(i2cAddress) {}

    bool begin() override
    {
        Wire.begin();
        if (!_imu.init())
        {
            Serial.println("MPU6500 does not respond");
           
        }
        else
        {

            _imu.autoOffsets();
            Serial.println("Done!");
            _imu.enableGyrDLPF();
            _imu.setGyrDLPF(MPU6500_DLPF_6);

            _imu.setSampleRateDivider(5);

            _imu.setGyrRange(MPU6500_GYRO_RANGE_250);

            _imu.setAccRange(MPU6500_ACC_RANGE_2G);

            _imu.enableAccDLPF(true);

            _imu.setAccDLPF(MPU6500_DLPF_6);

            delay(200);
            return true;
        }
        return false;
    }

    bool read() override
    {

        gValue = _imu.getGValues();
        gyr = _imu.getGyrValues();
        temp = _imu.getTemperature();
        resultantG = _imu.getResultantG(gValue);
        return true;
    }

    bool getData(JsonDocument &doc) override
    {
        doc["accX"] = gValue.x;
        doc["accY"] = gValue.y;
        doc["accZ"] =   gValue.z;
        doc["gyroX"] = gyr.x;
        doc["gyroY"] = gyr.y;
        doc["gyroZ"] = gyr.z;
        doc["temp"] = temp; 
        doc["g"] = resultantG; 
      

        return true;
    }

    const char *getError() const override
    {
        return _error;
    }

private:
    MPU6500_WE _imu;
    xyzFloat gValue;
    xyzFloat gyr;
    float resultantG;
    float temp;
};


#endif 

#endif