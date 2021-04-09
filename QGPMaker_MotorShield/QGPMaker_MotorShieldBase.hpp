#ifndef QGPMaker_MotorShield_hpp
#define QGPMaker_MotorShield_hpp

#include <inttypes.h>
#include <assert.h>
#include <Wire.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

namespace QGPMaker
{
#if defined(ARDUINO_SAM_DUE)
    static TwoWire &WIRE = Wire1;
#else
    static TwoWire &WIRE = Wire;
#endif

    class IMotorShield
    {
    public:
        static constexpr double UnitPulseLengthPerSecond = 1000000 / 4096.0;

        IMotorShield(const uint8_t addr = 0x60) : _addr(addr), _pwm(this->_addr)
        {
        }

        void begin(uint16_t freq = 50)
        {
            // init PWM w/_freq
            WIRE.begin();
            _pwm.begin();
            _freq = freq;
            _pwm.setPWMFreq(_freq); // This is the maximum PWM frequency
            for (uint8_t pin = 0; pin < 16; pin++)
            {
                _pwm.setPWM(pin, 0, 0);
            }
            this->setAsActiveShield();
        }

        //12-bit resolution
        inline void analogWrite(uint8_t pin, uint16_t value)
        {
            (value > 4095) ? (_pwm.setPWM(pin, 4096, 0)) : (_pwm.setPWM(pin, 0, value));
        }

        inline void digitalWrite(uint8_t pin, boolean value)
        {
            _pwm.setPWM(pin, (value == LOW) ? (0) : (4096), 0);
        }

        inline void pulseWrite(uint8_t pin, uint16_t pulse)
        {
            this->analogWrite(pin, this->pulseLengthToAnalogValue(pulse));
        }

        inline uint8_t getAddress() const
        {
            return this->_addr;
        }

        inline uint16_t getFrequency() const
        {
            return this->_freq;
        }

        //pulseLength in mcs
        inline uint16_t pulseLengthToAnalogValue(uint16_t pulseLength) const
        {
            return min(pulseLength / (UnitPulseLengthPerSecond / this->getFrequency()), 4096);
        }

        virtual void setAsActiveShield(void) = 0;

    private:
        uint8_t _addr;
        uint16_t _freq;
        Adafruit_MS_PWMServoDriver _pwm;
    };

    //This is singleton, don't create object
    class MotorShieldManager
    {
    public:
        MotorShieldManager() : _pShield(nullptr) {}

        inline void link(IMotorShield &shield)
        {
            this->_pShield = &shield;
        }

        inline void reset()
        {
            this->_pShield = nullptr;
        }

        inline IMotorShield *shieldLinked()
        {
            return this->_pShield;
        }

        inline bool isOperatable() const
        {
            return this->_pShield != nullptr;
        }

    private:
        IMotorShield *_pShield;
    };

}

#endif