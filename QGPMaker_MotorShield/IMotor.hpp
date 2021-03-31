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

typedef enum __StepperStyle__ : uint8_t
{
    SINGLE,
    DOUBLE,
    INTERLEAVE,
    MICROSTEP
} StepperStyle;

template<typename T, typename U>
struct is_same
{
    static constexpr bool value = false;
};

template<typename T>
struct is_same<T, T>  //specialization
{
    static constexpr bool value = true;
};

template<class ResolutionType = uint8_t>
class IDCMotor
{
    static_assert(is_same<ResolutionType, uint8_t>::value || is_same<ResolutionType, uint16_t>::value);
public:
    virtual void release() = 0;
    virtual void brake() = 0;
    virtual void moveForward(ResolutionType speed) = 0;
    virtual void moveBackward(ResolutionType speed) = 0;
    virtual void run(ResolutionType speed, MotorCommand command) = 0;
};

class IStepperMotor
{
public:
    virtual void step(uint16_t steps, MotorCommand command, StepperStyle style) = 0;
    virtual uint8_t oneStep(MotorCommand command, StepperStyle style) = 0;
    virtual void release() = 0;
};

#endif