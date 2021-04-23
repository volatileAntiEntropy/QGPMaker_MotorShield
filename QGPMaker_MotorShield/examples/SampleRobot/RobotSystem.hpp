#ifndef _RobotSystem_hpp
#define _RobotSystem_hpp

#include <QGPMaker_MotorShield.hpp>
#include <PS2X_lib.h>
#include <Servo.h>

class PS2XWrapper
{
public:
	static constexpr uint8_t ClockPin = 13;
	static constexpr uint8_t CommandPin = 11;
	static constexpr uint8_t AttentionPin = 10;
	static constexpr uint8_t DataPin = 12;

	PS2X Controller;

	void initialize()
	{
		int controllerError = 0x01;
		while (controllerError)
		{
			controllerError = this->Controller.config_gamepad(ClockPin, CommandPin, AttentionPin, DataPin);
			delay(1000);
		}
		for (size_t i = 0; i < 50; i++)
		{
			this->Controller.read_gamepad();
			delay(10);
		}
	}

	void read()
	{
		this->Controller.read_gamepad();
	}
};

namespace QGPMaker
{
	class MotorSystem
	{
	public:
		static constexpr uint8_t SlowModeSpeed = 255 / 2;
		static constexpr uint8_t TurnSpeed = 255 / 1.5;
		DCMotor<0> FrontLeftMotor;
		DCMotor<1> FrontRightMotor;
		DCMotor<2> RearLeftMotor;
		DCMotor<3> RearRightMotor;
		AsyncIntervalThresholder buttonThreshold;

		MotorSystem() : FrontLeftMotor(), FrontRightMotor(), RearLeftMotor(), RearRightMotor(), buttonThreshold(), speed(255)
		{
		}

		void runLeftMotors(uint8_t speed, MotorCommand command)
		{
			FrontLeftMotor.run(speed, command);
			RearLeftMotor.run(speed, command);
		}

		void runRightMotors(uint8_t speed, MotorCommand command)
		{
			FrontRightMotor.run(speed, command);
			RearRightMotor.run(speed, command);
		}

		void runAllMotors(uint8_t speed, MotorCommand command)
		{
			this->runLeftMotors(speed, command);
			this->runRightMotors(speed, command);
		}

		void turnLeft(uint8_t speed)
		{
			this->runLeftMotors(speed, BACKWARD);
			this->runRightMotors(speed, FORWARD);
		}

		void initialize()
		{
			this->runAllMotors(0, RELEASE);
		}

		void turnRight(uint8_t speed)
		{
			this->runLeftMotors(speed, FORWARD);
			this->runRightMotors(speed, BACKWARD);
		}

		void handleMotorEvents(PS2X &controller)
		{
			if (controller.Button(PSB_L3) && buttonThreshold.isTaskFinished(1000))
			{
				this->speed = (this->speed == 255) ? (SlowModeSpeed) : (255);
				buttonThreshold.startTask();
			}

			if (controller.Button(PSB_PAD_UP))
			{
				this->runAllMotors(this->speed, FORWARD);
			}
			else if (controller.Button(PSB_PAD_DOWN))
			{
				this->runAllMotors(this->speed, BACKWARD);
			}
			else if (controller.Button(PSB_PAD_LEFT))
			{
				this->turnLeft(TurnSpeed);
			}
			else if (controller.Button(PSB_PAD_RIGHT))
			{
				this->turnRight(TurnSpeed);
			}
			else
			{
				this->runAllMotors(0, BRAKE);
			}
		}

#ifdef DEBUG
		void print() const
		{
			Serial.print(F("Motor Speed: "));
			Serial.println(this->speed);
		}
#endif

	private:
		uint8_t speed;
	};

	class MechanicalArm
	{
	public:
		Servo<0, 120, 50, 130> LeftServo;
		Servo<1, 28, 0, 100> RightServo;
		Servo<2, 124, 0, 125> BottomServo;
		Servo<3, 57, 55, 103> ClipServo;

		MechanicalArm() : LeftServo(), RightServo(), BottomServo(), ClipServo() {}

		void resetWithoutClip()
		{
			LeftServo.initialize();
			RightServo.initialize();
			BottomServo.initialize();
		}

		void initialize()
		{
			this->resetWithoutClip();
			ClipServo.initialize();
		}

		void handleArmEvents(PS2X &controller)
		{
			if (controller.Button(PSB_SELECT))
			{
				this->resetWithoutClip();
			}

			if (controller.Button(PSB_L1))
			{
				LeftServo.differDegrees(1);
			}
			else if (controller.Button(PSB_L2))
			{
				LeftServo.differDegrees(-1);
			}

			if (controller.Button(PSB_R1))
			{
				RightServo.differDegrees(1);
			}
			else if (controller.Button(PSB_R2))
			{
				RightServo.differDegrees(-1);
			}

			if (controller.Button(PSB_SQUARE))
			{
				BottomServo.differDegrees(1);
			}
			else if (controller.Button(PSB_CIRCLE))
			{
				BottomServo.differDegrees(-1);
			}

			if (controller.Button(PSB_TRIANGLE))
			{
				ClipServo.differDegrees(1);
			}
			else if (controller.Button(PSB_CROSS))
			{
				ClipServo.differDegrees(-1);
			}
		}

#ifdef DEBUG
		void print() const
		{
			Serial.print(F("Servo Positions: "));
			Serial.print(LeftServo.readDegrees());
			Serial.print(',');
			Serial.print(RightServo.readDegrees());
			Serial.print(',');
			Serial.print(BottomServo.readDegrees());
			Serial.print(',');
			Serial.println(ClipServo.readDegrees());
		}
#endif
	};

	class Shooter
	{
	public:
		::Servo PlaceHolderInstance;
		ServoWrapper<::Servo, 180, 90, 180> PlaceHolder;
		DCMotor<4, uint8_t, 4, 5> LeftPuller;
		DCMotor<4, uint8_t, 6, 7> RightPuller;
		AsyncIntervalThresholder buttonThreshold;

		Shooter() : PlaceHolderInstance(), PlaceHolder(PlaceHolderInstance), LeftPuller(), RightPuller(), buttonThreshold(), placeHolderState(false) {}

		void initialize()
		{
			PlaceHolder.initialize(9);
		}

		void handleShooterEvents(PS2X &controller)
		{
			uint8_t LY = controller.Analog(PSS_LY);
			if (LY >= 127 + 8)
			{
				LeftPuller.run(255, FORWARD);
			}
			else if (LY <= 127 - 8)
			{
				LeftPuller.run(255, BACKWARD);
			}
			else
			{
				LeftPuller.brake();
			}

			uint8_t RY = controller.Analog(PSS_RY);
			if (RY >= 127 + 8)
			{
				RightPuller.run(255, FORWARD);
			}
			else if (RY <= 127 - 8)
			{
				RightPuller.run(255, BACKWARD);
			}
			else
			{
				RightPuller.brake();
			}

			if (controller.Button(PSB_R3) && buttonThreshold.isTaskFinished(1000))
			{
				this->placeHolderState = !(this->placeHolderState);
				PlaceHolder.writeDegrees((this->placeHolderState) ? (90) : (180));
				buttonThreshold.startTask();
			}
		}

	private:
		bool placeHolderState;
	};

	class Robot
	{
	public:
		MotorSystem Car;
		MechanicalArm Arm;
		Shooter Shoot;
		PS2XWrapper Controller;

		Robot() : Car(), Arm(), Shoot(), Controller(){};

		void setup()
		{
			Car.initialize();
			Arm.initialize();
			Shoot.initialize();
			Controller.initialize();
		}

		void loop()
		{
			Controller.read();
			Car.handleMotorEvents(Controller.Controller);
			Arm.handleArmEvents(Controller.Controller);
			Shoot.handleShooterEvents(Controller.Controller);
#ifdef DEBUG
			Car.print();
			Arm.print();
#endif
			delay(10);
		}
	};
}

#endif
