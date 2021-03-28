#ifndef _IMotor_hpp
#define _IMotor_hpp

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

typedef enum __MotorCommand__ : uint8_t
{
    RELEASE,
    BRAKE,
    FORWARD,
    BACKWARD
} MotorCommand;

class DCMotorPacket
{
public:
    DCMotorPacket(uint8_t _speed = 0, MotorCommand _command = RELEASE) : speed(_speed), command(_command) {}
    uint8_t speed;
    MotorCommand command;
};

typedef enum __StepperStyle__ : uint8_t
{
    SINGLE,
    DOUBLE,
    INTERLEAVE,
    MICROSTEP
} StepperStyle;

class IDCMotor
{
public:
    virtual void release() = 0;
    virtual void brake() = 0;
    virtual void moveForward(uint8_t speed) = 0;
    virtual void moveBackward(uint8_t speed) = 0;
    virtual void run(uint8_t speed, MotorCommand command) = 0;
    virtual void run(const DCMotorPacket &packet)
    {
        this->run(packet.speed, packet.command);
    }
};

class IStepperMotor
{
public:
    virtual void step(uint16_t steps, MotorCommand command, StepperStyle style) = 0;
    virtual uint8_t oneStep(MotorCommand command, StepperStyle style) = 0;
    virtual void release() = 0;
};

#endif