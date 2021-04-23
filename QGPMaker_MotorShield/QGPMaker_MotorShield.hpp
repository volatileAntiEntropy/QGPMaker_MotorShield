/******************************************************************

 It will only work with http://www.7gp.cn

 ******************************************************************/

#ifndef _QGPMaker_MotorShield_h_
#define _QGPMaker_MotorShield_h_

#include "QGPMaker_MotorShieldBase.hpp"
#include "utility/AsyncIntervalThresholder.hpp"
#include "utility/ServoWrapper.hpp"
#include "IMotor.hpp"

//#define MOTORDEBUG

//Stepper Motor
#define MICROSTEPS 16 // 8 or 16

namespace QGPMaker
{
	static MotorShieldManager Manager;

	class IConfigDCMotor
	{
	public:
		static constexpr uint8_t MaxInstanceNumber = 4;
		static constexpr uint8_t CustomInstanceIndex = MaxInstanceNumber;
		static constexpr uint8_t PinConfigs[MaxInstanceNumber][2] = {{8, 9}, {10, 11}, {15, 14}, {13, 12}};
	};

	template <uint8_t configIndex, class ResolutionType = uint8_t, uint8_t InputPin1 = IConfigDCMotor::PinConfigs[configIndex][0], uint8_t InputPin2 = IConfigDCMotor::PinConfigs[configIndex][1]>
	class DCMotor : public IConfigDCMotor, public IDCMotor<ResolutionType>
	{
	public:
		static_assert(configIndex <= MaxInstanceNumber);

		DCMotor() : _speed(0), _command(RELEASE) {}

		void release(void)
		{
			if (Manager.isOperatable())
			{
				this->_speed = 0;
				this->_command = RELEASE;
				Manager.shieldLinked()->digitalWrite(InputPin1, LOW);
				Manager.shieldLinked()->digitalWrite(InputPin2, LOW);
			}
		}

		void brake(void)
		{
			if (Manager.isOperatable())
			{
				this->_speed = 0;
				this->_command = BRAKE;
				Manager.shieldLinked()->digitalWrite(InputPin1, HIGH);
				Manager.shieldLinked()->digitalWrite(InputPin2, HIGH);
			}
		}

		void moveForward(ResolutionType speed)
		{
			if (Manager.isOperatable())
			{
				if constexpr (is_same<ResolutionType, uint16_t>::value)
				{
					this->_speed = min(speed, 4096);
				}
				else
				{
					this->_speed = speed;
				}
				this->_command = FORWARD;
				Manager.shieldLinked()->digitalWrite(InputPin2, LOW); // take low first to avoid brake
				if constexpr (is_same<ResolutionType, uint16_t>::value)
				{
					Manager.shieldLinked()->analogWrite(InputPin1, _speed);
				}
				else
				{
					Manager.shieldLinked()->analogWrite(InputPin1, (uint16_t)_speed * 16);
				}
			}
		}

		void moveBackward(ResolutionType speed)
		{
			if (Manager.isOperatable())
			{
				if constexpr (is_same<ResolutionType, uint16_t>::value)
				{
					this->_speed = min(speed, 4096);
				}
				else
				{
					this->_speed = speed;
				}
				this->_command = BACKWARD;
				Manager.shieldLinked()->digitalWrite(InputPin1, LOW); // take low first to avoid brake
				if constexpr (is_same<ResolutionType, uint16_t>::value)
				{
					Manager.shieldLinked()->analogWrite(InputPin2, _speed);
				}
				else
				{
					Manager.shieldLinked()->analogWrite(InputPin2, (uint16_t)_speed * 16);
				}
			}
		}

		void run(ResolutionType speed, MotorCommand command)
		{
			if (Manager.isOperatable())
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

		inline ResolutionType currentSpeed(void) const
		{
			return this->_speed;
		}

		inline MotorCommand currentCommand(void) const
		{
			return this->_command;
		}

	protected:
		ResolutionType _speed;
		MotorCommand _command;
	};

#if (MICROSTEPS == 8)
	constexpr uint8_t microStepCurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
	constexpr uint8_t microStepCurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

	class IConfigStepperMotor : public IStepperMotor
	{
	public:
		static constexpr uint8_t MicroStep = MICROSTEPS;
		static constexpr uint8_t LogicalMicroStep = MICROSTEPS / 2;
		static constexpr uint8_t MaxInstanceNumber = 2;
		static constexpr uint8_t CustomInstanceIndex = MaxInstanceNumber;
		static constexpr uint8_t PinConfigs[MaxInstanceNumber][2][2]{{{10, 9}, {11, 12}}, {{4, 3}, {5, 6}}};
	};

	template <uint8_t configIndex, uint8_t InputPinA1 = IConfigStepperMotor::PinConfigs[configIndex][0][0], uint8_t InputPinA2 = IConfigStepperMotor::PinConfigs[configIndex][0][1], uint8_t InputPinB1 = IConfigStepperMotor::PinConfigs[configIndex][1][0], uint8_t InputPinB2 = IConfigStepperMotor::PinConfigs[configIndex][1][1]>
	class StepperMotor : public IConfigStepperMotor
	{
		static_assert(configIndex <= MaxInstanceNumber);

	public:
		static constexpr uint32_t RPMToMicrosecondsPerStep(uint16_t stepsPerRevolution, uint16_t rpm)
		{
			return (stepsPerRevolution == 0 || rpm == 0) ? (0) : (60000000 / ((uint32_t)stepsPerRevolution * (uint32_t)rpm));
		}

		static constexpr uint8_t StepToLogicalStep(uint8_t steps)
		{
			return (steps / LogicalMicroStep);
		}

		StepperMotor() : _stepsPerRevolution(0), _currentStep(0)
		{
		}

		inline void setRPM(uint16_t rpm)
		{
			this->_microsecondsPerStep = RPMToMicrosecondsPerStep(this->_stepsPerRevolution, rpm);
		}

		inline void setRevolutionStep(uint16_t steps)
		{
			this->_stepsPerRevolution = steps;
		}

		inline uint8_t currentStep() const
		{
			return this->_currentStep;
		}

		inline uint8_t currentLogicalStep() const
		{
			return StepToLogicalStep(this->_currentStep);
		}

		void step(uint16_t steps, MotorCommand command, StepperStyle style = SINGLE)
		{
			if (Manager.isOperatable())
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
			if (command == RELEASE || command == BRAKE || !(Manager.isOperatable()))
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
			bool isInputALock = false, isInputBLock = false;
			uint8_t PWMPinA = 0, PWMPinB = 0;
			if ((latch_state & 0x1) && (latch_state & 0x4))
			{
				isInputALock = true;
				Manager.shieldLinked()->digitalWrite(InputPinA2, HIGH);
				Manager.shieldLinked()->digitalWrite(InputPinA1, HIGH);
			}
			else
			{
				if (latch_state & 0x1)
				{
					PWMPinA = InputPinA2;
				}
				else
				{
					Manager.shieldLinked()->digitalWrite(InputPinA2, LOW);
				}

				if (latch_state & 0x4)
				{
					PWMPinA = InputPinA1;
				}
				else
				{
					Manager.shieldLinked()->digitalWrite(InputPinA1, LOW);
				}
			}

			if ((latch_state & 0x2) && (latch_state & 0x8))
			{
				isInputBLock = true;
				Manager.shieldLinked()->digitalWrite(InputPinB2, HIGH);
				Manager.shieldLinked()->digitalWrite(InputPinB1, HIGH);
			}
			else
			{
				if (latch_state & 0x2)
				{
					PWMPinB = InputPinB1;
				}
				else
				{
					Manager.shieldLinked()->digitalWrite(InputPinB1, LOW);
				}

				if (latch_state & 0x8)
				{
					PWMPinB = InputPinB2;
				}
				else
				{
					Manager.shieldLinked()->digitalWrite(InputPinB2, LOW);
				}
			}

			if (!isInputALock)
			{
				Manager.shieldLinked()->analogWrite(PWMPinA, (uint16_t)curveA * 16);
			}

			if (!isInputBLock)
			{
				Manager.shieldLinked()->analogWrite(PWMPinB, (uint16_t)curveB * 16);
			}

			return _currentStep;
		}

		void release(void)
		{
			if (Manager.isOperatable())
			{
				Manager.shieldLinked()->digitalWrite(InputPinA1, LOW);
				Manager.shieldLinked()->digitalWrite(InputPinA2, LOW);
				Manager.shieldLinked()->digitalWrite(InputPinB1, LOW);
				Manager.shieldLinked()->digitalWrite(InputPinB2, LOW);
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
	class ServoImplement
	{
	public:
		static constexpr uint8_t MaxInstanceNumber = 8;
		static constexpr uint8_t PinConfigs[MaxInstanceNumber] = {0, 1, 2, 3, 4, 5, 6, 7};
		static_assert(configIndex < MaxInstanceNumber);
		static constexpr uint8_t PWMpin = PinConfigs[configIndex];

		ServoImplement() : _currentPulse(0)
		{
		}

		bool writeMicroseconds(uint16_t pulseWidth)
		{
			if (Manager.isOperatable())
			{
				Manager.shieldLinked()->pulseWrite(PWMpin, pulseWidth);
				this->_currentPulse = pulseWidth;
				return true;
			}
			return false;
		}

		uint16_t readMicroseconds(void) const
		{
			return this->_currentPulse;
		}

		uint8_t attach(int pin)
		{
			return configIndex;
		}

	private:
		uint16_t _currentPulse;
	};

	template <uint8_t configIndex, uint8_t initialPosition = 0, uint8_t lowerLimit = 0, uint8_t upperLimit = 180, uint16_t pulseChangeRatio = 180>
	class Servo : public ServoWrapper<ServoImplement<configIndex>, initialPosition, lowerLimit, upperLimit, pulseChangeRatio>
	{
	public:
		Servo() : servoInstance(), ServoWrapper<ServoImplement<configIndex>, initialPosition, lowerLimit, upperLimit, pulseChangeRatio>(servoInstance) {}

	protected:
		ServoImplement<configIndex> servoInstance;
	};

	class MotorShield : public IMotorShield
	{
	public:
		MotorShield(const uint8_t addr = 0x60) : IMotorShield(addr)
		{
		}

		void setAsActiveShield()
		{
			Manager.link(*this);
		}
	};
}

#endif