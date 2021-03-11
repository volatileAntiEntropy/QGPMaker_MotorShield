/******************************************************************

 It will only work with http://www.7gp.cn
 
 ******************************************************************/

#ifndef _QGPMaker_MotorShield_h_
#define _QGPMaker_MotorShield_h_

#include <inttypes.h>
#include <assert.h>
#include <Wire.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//#define MOTORDEBUG

//Stepper Motor
#define MICROSTEPS 16 // 8 or 16

namespace QGPMaker
{
  constexpr uint8_t MOTOR1_A = 2;
  constexpr uint8_t MOTOR1_B = 3;
  constexpr uint8_t MOTOR2_A = 1;
  constexpr uint8_t MOTOR2_B = 4;
  constexpr uint8_t MOTOR4_A = 0;
  constexpr uint8_t MOTOR4_B = 6;
  constexpr uint8_t MOTOR3_A = 5;
  constexpr uint8_t MOTOR3_B = 7;

  constexpr uint8_t MaxDCMotorNumber = 4;
  constexpr uint8_t MaxServoNumber = 8;
  constexpr uint8_t MaxStepperMotorNumber = 2;

  constexpr uint8_t DCMotorPins[MaxDCMotorNumber][2] = {{8, 9}, {10, 11}, {15, 14}, {13, 12}};
  constexpr uint8_t ServoPWMPins[MaxServoNumber] = {1, 2, 3, 4, 5, 6, 7, 8};
  constexpr uint8_t StepperMotorPins[MaxStepperMotorNumber][2][3]{{{8, 10, 9}, {13, 11, 12}}, {{2, 4, 3}, {7, 5, 6}}};

#if defined(ARDUINO_SAM_DUE)
  TwoWire &WIRE = Wire1;
#else
  TwoWire &WIRE = Wire;
#endif

#if (MICROSTEPS == 8)
  constexpr uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
  constexpr uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

  typedef enum __StepperStyle__ : uint8_t
  {
    SINGLE,
    DOUBLE,
    INTERLEAVE,
    MICROSTEP
  } StepperStyle;

  typedef enum __DCMotorCommand__ : uint8_t
  {
    RELEASE,
    BRAKE,
    FORWARD,
    BACKWARD
  } DCMotorCommand;

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
      this->linkToDCMotors();
      this->linkToSteppers();
      this->linkToServos();
    }

    virtual void linkToDCMotors(void) = 0;

    virtual void linkToSteppers(void) = 0;

    virtual void linkToServos(void) = 0;

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

  private:
    uint8_t _addr;
    uint16_t _freq;
    Adafruit_MS_PWMServoDriver _pwm;
  };

  class IMotorShieldPart
  {
  public:
    IMotorShieldPart() : MC(nullptr) {}

    void link(IMotorShield &shield)
    {
      this->MC = &shield;
    }

  protected:
    IMotorShield *MC;
  };

  template <uint8_t configIndex>
  class DCMotor : public IMotorShieldPart
  {
    static_assert(configIndex < MaxDCMotorNumber);

  public:
    static constexpr uint8_t IN1pin = DCMotorPins[configIndex][0];
    static constexpr uint8_t IN2pin = DCMotorPins[configIndex][1];

    DCMotor() : IMotorShieldPart(), _speed(0), MDIR(RELEASE)
    {
    }

    void release(void)
    {
      this->_speed = 0;
      this->MDIR = RELEASE;
      MC->digitalWrite(IN1pin, LOW);
      MC->digitalWrite(IN2pin, LOW);
    }

    void brake(void)
    {
      this->_speed = 0;
      this->MDIR = BRAKE;
      MC->digitalWrite(IN1pin, HIGH);
      MC->digitalWrite(IN2pin, HIGH);
    }

    void moveForward(uint8_t speed)
    {
      this->_speed = speed;
      this->MDIR = FORWARD;
      MC->digitalWrite(IN2pin, LOW); // take low first to avoid brake
      MC->analogWrite(IN1pin, _speed * 16);
    }

    void moveBackward(uint8_t speed)
    {
      this->_speed = speed;
      this->MDIR = BACKWARD;
      MC->digitalWrite(IN1pin, LOW); // take low first to avoid brake
      MC->analogWrite(IN2pin, _speed * 16);
    }

    void run(uint8_t speed, DCMotorCommand command)
    {
      switch (command)
      {
      case RELEASE:
        this->release();
        break;
      case BRAKE:
        this->brake();
        break;
      case FORWARD:
        this->moveForward(speed);
        break;
      case BACKWARD:
        this->moveBackward(speed);
        break;
      }
    }

    uint8_t currentSpeed(void) const
    {
      return this->_speed;
    }

    DCMotorCommand currentCommand(void) const
    {
      return this->MDIR;
    }

  private:
    uint8_t _speed, MDIR;
  };

  template <uint8_t configIndex>
  class StepperMotor : public IMotorShieldPart
  {
    static_assert(configIndex < MaxStepperMotorNumber);

  public:
    static constexpr uint8_t PWMApin = StepperMotorPins[configIndex][0][0];
    static constexpr uint8_t AIN1pin = StepperMotorPins[configIndex][0][1];
    static constexpr uint8_t AIN2pin = StepperMotorPins[configIndex][0][2];
    static constexpr uint8_t PWMBpin = StepperMotorPins[configIndex][1][0];
    static constexpr uint8_t BIN1pin = StepperMotorPins[configIndex][1][1];
    static constexpr uint8_t BIN2pin = StepperMotorPins[configIndex][1][2];

    StepperMotor() : IMotorShieldPart(), revsteps(0), currentstep(0)
    {
    }

    void step(uint16_t steps, DCMotorCommand dir, StepperStyle style = SINGLE)
    {
      uint32_t uspers = usperstep;
      uint8_t ret = 0;

      if (style == INTERLEAVE)
      {
        uspers /= 2;
      }
      else if (style == MICROSTEP)
      {
        uspers /= MICROSTEPS;
        steps *= MICROSTEPS;
#ifdef MOTORDEBUG
        Serial.print("steps = ");
        Serial.println(steps, DEC);
#endif
      }

      while (steps--)
      {
        //Serial.println("step!"); Serial.println(uspers);
        ret = onestep(dir, style);
        delayMicroseconds(uspers);
        yield(); // required for ESP8266
      }
    }

    void setSpeed(uint16_t rpm)
    {
      this->usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
    }

    void setRevolutionStep(uint16_t steps)
    {
      this->revsteps = steps;
    }

    uint8_t onestep(DCMotorCommand dir, StepperStyle style)
    {
      uint8_t a, b, c, d;
      uint8_t ocrb, ocra;

      ocra = ocrb = 255;

      // next determine what sort of stepping procedure we're up to
      if (style == SINGLE)
      {
        if ((currentstep / (MICROSTEPS / 2)) % 2)
        { // we're at an odd step, weird
          if (dir == FORWARD)
          {
            currentstep += MICROSTEPS / 2;
          }
          else
          {
            currentstep -= MICROSTEPS / 2;
          }
        }
        else if (dir == FORWARD)
        {
          currentstep += MICROSTEPS;
        }
        else
        {
          currentstep -= MICROSTEPS;
        }
      }
      else if (style == DOUBLE)
      {
        if (!(currentstep / (MICROSTEPS / 2) % 2))
        { // we're at an even step, weird
          if (dir == FORWARD)
          {
            currentstep += MICROSTEPS / 2;
          }
          else
          {
            currentstep -= MICROSTEPS / 2;
          }
        }
        else if (dir == FORWARD) // go to the next odd step
        {
          currentstep += MICROSTEPS;
        }
        else
        {
          currentstep -= MICROSTEPS;
        }
      }
      else if (style == INTERLEAVE)
      {
        if (dir == FORWARD)
        {
          currentstep += MICROSTEPS / 2;
        }
        else
        {
          currentstep -= MICROSTEPS / 2;
        }
      }

      if (style == MICROSTEP)
      {
        if (dir == FORWARD)
        {
          currentstep++;
        }
        else
        {
          // BACKWARDS
          currentstep--;
        }

        currentstep += MICROSTEPS * 4;
        currentstep %= MICROSTEPS * 4;

        ocra = ocrb = 0;
        if ((currentstep >= 0) && (currentstep < MICROSTEPS))
        {
          ocra = microstepcurve[MICROSTEPS - currentstep];
          ocrb = microstepcurve[currentstep];
        }
        else if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
        {
          ocra = microstepcurve[currentstep - MICROSTEPS];
          ocrb = microstepcurve[MICROSTEPS * 2 - currentstep];
        }
        else if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
        {
          ocra = microstepcurve[MICROSTEPS * 3 - currentstep];
          ocrb = microstepcurve[currentstep - MICROSTEPS * 2];
        }
        else if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
        {
          ocra = microstepcurve[currentstep - MICROSTEPS * 3];
          ocrb = microstepcurve[MICROSTEPS * 4 - currentstep];
        }
      }

      currentstep += MICROSTEPS * 4;
      currentstep %= MICROSTEPS * 4;

#ifdef MOTORDEBUG
      Serial.print("current step: ");
      Serial.println(currentstep, DEC);
      Serial.print(" pwmA = ");
      Serial.print(ocra, DEC);
      Serial.print(" pwmB = ");
      Serial.println(ocrb, DEC);
#endif
      MC->analogWrite(PWMApin, ocra * 16);
      MC->analogWrite(PWMBpin, ocrb * 16);

      // release all
      uint8_t latch_state = 0; // all motor pins to 0

      //Serial.println(step, DEC);
      if (style == MICROSTEP)
      {
        if ((currentstep >= 0) && (currentstep < MICROSTEPS))
          latch_state |= 0x03;
        if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
          latch_state |= 0x06;
        if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
          latch_state |= 0x0C;
        if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
          latch_state |= 0x09;
      }
      else
      {
        switch (currentstep / (MICROSTEPS / 2))
        {
        case 0:
          latch_state |= 0x1; // energize coil 1 only
          break;
        case 1:
          latch_state |= 0x3; // energize coil 1+2
          break;
        case 2:
          latch_state |= 0x2; // energize coil 2 only
          break;
        case 3:
          latch_state |= 0x6; // energize coil 2+3
          break;
        case 4:
          latch_state |= 0x4; // energize coil 3 only
          break;
        case 5:
          latch_state |= 0xC; // energize coil 3+4
          break;
        case 6:
          latch_state |= 0x8; // energize coil 4 only
          break;
        case 7:
          latch_state |= 0x9; // energize coil 1+4
          break;
        }
      }
#ifdef MOTORDEBUG
      Serial.print("Latch: 0x");
      Serial.println(latch_state, HEX);
#endif

      if (latch_state & 0x1)
      {
        // Serial.println(AIN2pin);
        MC->digitalWrite(AIN2pin, HIGH);
      }
      else
      {
        MC->digitalWrite(AIN2pin, LOW);
      }
      if (latch_state & 0x2)
      {
        MC->digitalWrite(BIN1pin, HIGH);
        // Serial.println(BIN1pin);
      }
      else
      {
        MC->digitalWrite(BIN1pin, LOW);
      }
      if (latch_state & 0x4)
      {
        MC->digitalWrite(AIN1pin, HIGH);
        // Serial.println(AIN1pin);
      }
      else
      {
        MC->digitalWrite(AIN1pin, LOW);
      }
      if (latch_state & 0x8)
      {
        MC->digitalWrite(BIN2pin, HIGH);
        // Serial.println(BIN2pin);
      }
      else
      {
        MC->digitalWrite(BIN2pin, LOW);
      }

      return currentstep;
    }

    void release(void)
    {
      MC->digitalWrite(AIN1pin, LOW);
      MC->digitalWrite(AIN2pin, LOW);
      MC->digitalWrite(BIN1pin, LOW);
      MC->digitalWrite(BIN2pin, LOW);
      MC->analogWrite(PWMApin, 0);
      MC->analogWrite(PWMBpin, 0);
    }

  private:
    uint32_t usperstep;
    uint16_t revsteps; // # steps per revolution
    uint8_t currentstep;
    uint8_t steppernum;
  };

  template <uint8_t configIndex>
  class Servo : public IMotorShieldPart
  {
    static_assert(configIndex < MaxServoNumber);

  public:
    static constexpr uint8_t PWMpin = ServoPWMPins[configIndex];

    Servo() : IMotorShieldPart()
    {
    }

    static constexpr double pulseMilliseconds(uint8_t angle)
    {
      if (angle <= 180)
      {
        //0.5-2.5ms pulse width mapped to 0-180 degrees
        return 0.5 + angle / 90.0;
      }
    }

    void setServoPulse(double pulse)
    {
      double pulselength = 1000000; // 1,000,000 us per second
      pulselength /= 50;            // 50 Hz
      pulselength /= 4096;          // 12 bits of resolution
      pulse *= 1000;
      pulse /= pulselength;
      MC->analogWrite(PWMpin, pulse);
    }

    void writeDegrees(uint8_t angle)
    {
      if (angle <= 180)
      {
        if (this->currentPosition < angle)
        {
          this->currentPosition++;
          for (; this->currentPosition <= angle; this->currentPosition++)
          {
            this->setServoPulse(pulseMilliseconds(this->currentPosition));
            delayMicroseconds(1000);
          }
        }
        else if (this->currentPosition > angle)
        {
          this->currentPosition--;
          for (; this->currentPosition >= angle; this->currentPosition--)
          {
            this->setServoPulse(pulseMilliseconds(this->currentPosition));
            delayMicroseconds(1000);
          }
        }
      }
    }

    uint8_t readDegrees(void) const
    {
      return this->currentPosition;
    }

  private:
    uint8_t currentPosition;
  };

  DCMotor<0> Motor0; //M1
  DCMotor<1> Motor1; //M2
  DCMotor<2> Motor2; //M3
  DCMotor<3> Motor3; //M4

  Servo<0> Servo0;
  Servo<1> Servo1;
  Servo<2> Servo2;
  Servo<3> Servo3;
  Servo<4> Servo4;
  Servo<5> Servo5;
  Servo<6> Servo6;
  Servo<7> Servo7;

  StepperMotor<0> Stepper0;
  StepperMotor<1> Stepper1;

  class MotorShield : public IMotorShield
  {
  public:
    MotorShield(const uint8_t addr = 0x60) : IMotorShield(addr)
    {
    }

    void linkToDCMotors(void)
    {
      Motor0.link(*this);
      Motor1.link(*this);
      Motor2.link(*this);
      Motor3.link(*this);
    }

    void linkToSteppers(void)
    {
      Stepper0.link(*this);
      Stepper1.link(*this);
    }

    void linkToServos(void)
    {
      Servo0.link(*this);
      Servo1.link(*this);
      Servo2.link(*this);
      Servo3.link(*this);
      Servo4.link(*this);
      Servo5.link(*this);
      Servo6.link(*this);
      Servo7.link(*this);
    }
  };
}

#endif
