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
            for (uint8_t i = 0; i < 16; i++)
            {
                _pwm.setPWM(i, 0, 0);
            }
            this->setAsActiveShield();
        }

        //12-bit resolution
        void analogWrite(uint8_t pin, uint16_t value)
        {
            if (value > 4095)
            {
                _pwm.setPWM(pin, 4096, 0);
            }
            else
            {
                _pwm.setPWM(pin, 0, value);
            }
        }

        void digitalWrite(uint8_t pin, boolean value)
        {
            if (value == LOW)
            {
                _pwm.setPWM(pin, 0, 0);
            }
            else
            {
                _pwm.setPWM(pin, 4096, 0);
            }
        }

        void setAsActiveShield(void)
        {
            this->linkToDCMotors();
            this->linkToServos();
            this->linkToSteppers();
        }

        virtual void linkToDCMotors(void)
        {
        }

        virtual void linkToSteppers(void)
        {
        }

        virtual void linkToServos(void)
        {
        }

    private:
        uint8_t _addr;
        uint16_t _freq;
        Adafruit_MS_PWMServoDriver _pwm;
    };

    class IMotorShieldPart
    {
    public:
        IMotorShieldPart() : _pShield(nullptr) {}

        inline void link(IMotorShield &shield)
        {
            this->_pShield = &shield;
        }

        inline IMotorShield *shieldLinked()
        {
            return this->_pShield;
        }

        inline const IMotorShield *shieldLinkedConst() const
        {
            return this->_pShield;
        }

        inline bool isOperatable() const
        {
            return this->shieldLinkedConst() != nullptr;
        }

    protected:
        IMotorShield *_pShield;
    };
}