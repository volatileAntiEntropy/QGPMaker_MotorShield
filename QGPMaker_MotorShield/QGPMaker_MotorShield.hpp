/******************************************************************

 It will only work with http://www.7gp.cn
 
 ******************************************************************/

#ifndef _QGPMaker_MotorShield_h_
#define _QGPMaker_MotorShield_h_

#include "QGPMaker_MotorShieldBase.hpp"

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

  constexpr uint8_t M1 = 0;
  constexpr uint8_t M2 = 1;
  constexpr uint8_t M3 = 2;
  constexpr uint8_t M4 = 3;

#if (MICROSTEPS == 8)
  constexpr uint8_t microStepCurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
  constexpr uint8_t microStepCurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

  typedef enum __StepperStyle__ : uint8_t
  {
    SINGLE,
    DOUBLE,
    INTERLEAVE,
    MICROSTEP
  } StepperStyle;

  typedef enum __MotorCommand__ : uint8_t
  {
    RELEASE,
    BRAKE,
    FORWARD,
    BACKWARD
  } MotorCommand;

  template <uint8_t configIndex>
  class DCMotor : public IMotorShieldPart
  {
  public:
    static constexpr uint8_t MaxInstanceNumber = 4;
    static constexpr uint8_t PinConfigs[MaxInstanceNumber][2] = {{8, 9}, {10, 11}, {15, 14}, {13, 12}};

    static_assert(configIndex < MaxInstanceNumber);

    static constexpr uint8_t InputPin1 = PinConfigs[configIndex][0];
    static constexpr uint8_t InputPin2 = PinConfigs[configIndex][1];

    DCMotor() : IMotorShieldPart(), _speed(0), _command(RELEASE)
    {
    }

    void release(void)
    {
      if (this->isOperatable())
      {
        this->_speed = 0;
        this->_command = RELEASE;
        this->shieldLinked()->digitalWrite(InputPin1, LOW);
        this->shieldLinked()->digitalWrite(InputPin2, LOW);
      }
    }

    void brake(void)
    {
      if (this->isOperatable())
      {
        this->_speed = 0;
        this->_command = BRAKE;
        this->shieldLinked()->digitalWrite(InputPin1, HIGH);
        this->shieldLinked()->digitalWrite(InputPin2, HIGH);
      }
    }

    void moveForward(uint8_t speed)
    {
      if (this->isOperatable())
      {
        this->_speed = speed;
        this->_command = FORWARD;
        this->shieldLinked()->digitalWrite(InputPin2, LOW); // take low first to avoid brake
        this->shieldLinked()->analogWrite(InputPin1, _speed * 16);
      }
    }

    void moveBackward(uint8_t speed)
    {
      if (this->isOperatable())
      {
        this->_speed = speed;
        this->_command = BACKWARD;
        this->shieldLinked()->digitalWrite(InputPin1, LOW); // take low first to avoid brake
        this->shieldLinked()->analogWrite(InputPin2, _speed * 16);
      }
    }

    void run(uint8_t speed, MotorCommand command)
    {
      if (this->isOperatable())
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
    }

    uint8_t currentSpeed(void) const
    {
      return this->_speed;
    }

    MotorCommand currentCommand(void) const
    {
      return this->_command;
    }

  private:
    uint8_t _speed;
    MotorCommand _command;
  };

  template <uint8_t configIndex>
  class StepperMotor : public IMotorShieldPart
  {
  public:
    static constexpr uint8_t MicroStep = MICROSTEPS;
    static constexpr uint8_t LogicalMicroStep = MICROSTEPS / 2;
    static constexpr uint8_t MaxInstanceNumber = 2;
    static constexpr uint8_t PinConfigs[MaxInstanceNumber][2][3]{{{8, 10, 9}, {13, 11, 12}}, {{2, 4, 3}, {7, 5, 6}}};

    static_assert(configIndex < MaxInstanceNumber);

    static constexpr uint8_t PWMPinA = PinConfigs[configIndex][0][0];
    static constexpr uint8_t InputPinA1 = PinConfigs[configIndex][0][1];
    static constexpr uint8_t InputPinA2 = PinConfigs[configIndex][0][2];
    static constexpr uint8_t PWMPinB = PinConfigs[configIndex][1][0];
    static constexpr uint8_t InputPinB1 = PinConfigs[configIndex][1][1];
    static constexpr uint8_t InputPinB2 = PinConfigs[configIndex][1][2];

    static constexpr uint32_t RPMToMicrosecondsPerStep(uint16_t stepsPerRevolution, uint16_t rpm)
    {
      return 60000000 / ((uint32_t)stepsPerRevolution * (uint32_t)rpm);
    }

    static constexpr uint8_t StepToLogicalStep(uint8_t steps)
    {
      return (steps / LogicalMicroStep);
    }

    StepperMotor() : IMotorShieldPart(), _stepsPerRevolution(0), _currentStep(0)
    {
    }

    void setRPM(uint16_t rpm)
    {
      this->_microsecondsPerStep = RPMToMicrosecondsPerStep(this->_stepsPerRevolution, rpm);
    }

    void setRevolutionStep(uint16_t steps)
    {
      this->_stepsPerRevolution = steps;
    }

    uint8_t currentStep() const
    {
      return this->_currentStep;
    }

    uint8_t currentLogicalStep() const
    {
      return StepToLogicalStep(this->_currentSteps);
    }

    void step(uint16_t steps, MotorCommand command, StepperStyle style = SINGLE)
    {
      if (this->isOperatable())
      {
        uint32_t usPerStep = _microsecondsPerStep;

        if (style == INTERLEAVE)
        {
          usPerStep /= 2;
        }
        else if (style == MICROSTEP)
        {
          usPerStep /= MicroStep;
          steps *= MicroStep;
#ifdef MOTORDEBUG
          Serial.print("steps = ");
          Serial.println(steps, DEC);
#endif
        }

        while (steps--)
        {
          this->oneStep(command, style);
          delayMicroseconds(usPerStep);
          yield(); // required for ESP8266
        }
      }
    }

    uint8_t oneStep(MotorCommand command, StepperStyle style)
    {
      if (command == RELEASE || command == BRAKE || !(this->isOperatable()))
      {
        return this->_currentStep;
      }

      uint8_t curveA = 255, curveB = 255;

      // next determine what sort of stepping procedure we're up to
      uint8_t logicalSteps = this->currentLogicalStep();
      switch (style)
      {
      case SINGLE:
        this->correctParityAndStep(logicalSteps % 2, command);
        break;
      case DOUBLE:
        this->correctParityAndStep(!(logicalSteps % 2), command);
        break;
      case INTERLEAVE:
        this->makeStep(MicroStep / 2, command);
        break;
      case MICROSTEP:
        this->makeStep(1, command);
        _currentStep += MicroStep * 4;
        _currentStep %= MicroStep * 4;
        if (_currentStep < MicroStep)
        {
          curveA = microStepCurve[MicroStep - _currentStep];
          curveB = microStepCurve[_currentStep];
        }
        else if ((_currentStep >= MicroStep) && (_currentStep < MicroStep * 2))
        {
          curveA = microStepCurve[_currentStep - MicroStep];
          curveB = microStepCurve[MicroStep * 2 - _currentStep];
        }
        else if ((_currentStep >= MicroStep * 2) && (_currentStep < MicroStep * 3))
        {
          curveA = microStepCurve[MicroStep * 3 - _currentStep];
          curveB = microStepCurve[_currentStep - MicroStep * 2];
        }
        else if ((_currentStep >= MicroStep * 3) && (_currentStep < MicroStep * 4))
        {
          curveA = microStepCurve[_currentStep - MicroStep * 3];
          curveB = microStepCurve[MicroStep * 4 - _currentStep];
        }
        else
        {
          curveA = curveB = 0;
        }
        break;
      }
      _currentStep += MicroStep * 4;
      _currentStep %= MicroStep * 4;

#ifdef MOTORDEBUG
      Serial.print("current step: ");
      Serial.println(_currentStep, DEC);
      Serial.print(" pwmA = ");
      Serial.print(curveA, DEC);
      Serial.print(" pwmB = ");
      Serial.println(curveB, DEC);
#endif
      this->shieldLinked()->analogWrite(PWMPinA, curveA * 16);
      this->shieldLinked()->analogWrite(PWMPinB, curveB * 16);

      // release all
      uint8_t latch_state = 0; // all motor pins to 0

      if (style == MICROSTEP)
      {
        if ((_currentStep >= 0) && (_currentStep < MicroStep))
          latch_state |= 0x03;
        if ((_currentStep >= MicroStep) && (_currentStep < MicroStep * 2))
          latch_state |= 0x06;
        if ((_currentStep >= MicroStep * 2) && (_currentStep < MicroStep * 3))
          latch_state |= 0x0C;
        if ((_currentStep >= MicroStep * 3) && (_currentStep < MicroStep * 4))
          latch_state |= 0x09;
      }
      else
      {
        switch (this->currentLogicalStep())
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
        // Serial.println(InputPinA2);
        this->shieldLinked()->digitalWrite(InputPinA2, HIGH);
      }
      else
      {
        this->shieldLinked()->digitalWrite(InputPinA2, LOW);
      }
      if (latch_state & 0x2)
      {
        this->shieldLinked()->digitalWrite(InputPinB1, HIGH);
        // Serial.println(InputPinB1);
      }
      else
      {
        this->shieldLinked()->digitalWrite(InputPinB1, LOW);
      }
      if (latch_state & 0x4)
      {
        this->shieldLinked()->digitalWrite(InputPinA1, HIGH);
      }
      else
      {
        this->shieldLinked()->digitalWrite(InputPinA1, LOW);
      }
      if (latch_state & 0x8)
      {
        this->shieldLinked()->digitalWrite(InputPinB2, HIGH);
      }
      else
      {
        this->shieldLinked()->digitalWrite(InputPinB2, LOW);
      }

      return _currentStep;
    }

    void release(void)
    {
      if (this->isOperatable())
      {
        this->shieldLinked()->digitalWrite(InputPinA1, LOW);
        this->shieldLinked()->digitalWrite(InputPinA2, LOW);
        this->shieldLinked()->digitalWrite(InputPinB1, LOW);
        this->shieldLinked()->digitalWrite(InputPinB2, LOW);
        this->shieldLinked()->analogWrite(PWMPinA, 0);
        this->shieldLinked()->analogWrite(PWMPinB, 0);
      }
    }

  protected:
    void makeStep(uint8_t step, MotorCommand command)
    {
      if (command == FORWARD)
      {
        _currentStep += step;
      }
      else if (command == BACKWARD)
      {
        _currentStep -= step;
      }
    }

    void correctParityAndStep(bool wrongParityFlag, MotorCommand command)
    {
      if (wrongParityFlag)
      {
        // we're at an incorrect parity step, weird
        this->makeStep(MicroStep / 2, command);
      }
      else
      {
        // go to next step
        this->makeStep(MicroStep, command);
      }
    }

  private:
    uint32_t _microsecondsPerStep;
    uint16_t _stepsPerRevolution; // # steps per revolution
    uint8_t _currentStep;
  };

  template <uint8_t configIndex>
  class Servo : public IMotorShieldPart
  {
  public:
    static constexpr uint8_t MaxInstanceNumber = 8;
    static constexpr uint8_t PinConfigs[MaxInstanceNumber] = {1, 2, 3, 4, 5, 6, 7, 8};

    static_assert(configIndex < MaxInstanceNumber);

    static constexpr uint8_t PWMpin = PinConfigs[configIndex];

    static constexpr double AngleToPulseMilliseconds(uint8_t angle)
    {
      //0.5-2.5ms pulse width mapped to 0-180 degrees
      return (angle <= 180) ? (0.5 + angle / 90.0) : (0.5);
    }

    Servo() : IMotorShieldPart()
    {
    }

    void setServoPulse(double pulse)
    {
      if (this->isOperatable())
      {
        double pulselength = 1000000; // 1,000,000 us per second
        pulselength /= 50;            // 50 Hz
        pulselength /= 4096;          // 12 bits of resolution
        pulse *= 1000;
        pulse /= pulselength;
        this->shieldLinked()->analogWrite(PWMpin, pulse);
      }
    }

    void writeDegrees(uint8_t angle)
    {
      if (angle <= 180 && this->isOperatable())
      {
        if (this->_currentPosition < angle)
        {
          this->_currentPosition++;
          for (; this->_currentPosition <= angle; this->_currentPosition++)
          {
            this->setServoPulse(AngleToPulseMilliseconds(this->_currentPosition));
            delayMicroseconds(1000);
          }
        }
        else if (this->_currentPosition > angle)
        {
          this->_currentPosition--;
          for (; this->_currentPosition >= angle; this->_currentPosition--)
          {
            this->setServoPulse(AngleToPulseMilliseconds(this->_currentPosition));
            delayMicroseconds(1000);
          }
        }
      }
    }

    uint8_t readDegrees(void) const
    {
      return this->_currentPosition;
    }

  private:
    uint8_t _currentPosition;
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
