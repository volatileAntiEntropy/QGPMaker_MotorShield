#ifndef _Servo_Wrapper_hpp
#define _Servo_Wrapper_hpp

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

template <typename T>
constexpr T linearMapping(T input, T lowerLimitFrom, T upperLimitFrom, T lowerLimitTo, T upperLimitTo)
{
    return (input - lowerLimitFrom) * (upperLimitTo - lowerLimitTo) / (upperLimitFrom - lowerLimitFrom) + lowerLimitTo;
}

template <typename IServoImplement, uint8_t initialPosition = 0, uint8_t lowerLimit = 0, uint8_t upperLimit = 180, uint16_t pulseChangeRatio = 180>
class ServoWrapper
{
    static_assert(lowerLimit >= 0 && upperLimit <= 180 && lowerLimit <= initialPosition && initialPosition <= upperLimit);

public:
    static constexpr uint16_t MinPulseWidth = 544;
    static constexpr uint16_t MaxPulseWidth = 2400;

    static constexpr uint8_t normalizeAngle(uint8_t angle)
    {
        return min(max(angle, lowerLimit), upperLimit);
    }

    static constexpr uint16_t normalizePulse(uint16_t pulse)
    {
        return min(max(pulse, DegreesToPulseMicroseconds(lowerLimit)), DegreesToPulseMicroseconds(upperLimit));
    }

    static constexpr uint16_t DegreesToPulseMicroseconds(uint8_t angle)
    {
        //500-2500mcs pulse width mapped to 0-180 degrees
        return linearMapping<uint16_t>(angle, 0, 180, MinPulseWidth, MaxPulseWidth);
    }

    static constexpr uint16_t DegreesToPulseMicrosecondsNormalized(uint8_t angle)
    {
        //500-2500mcs pulse width mapped to 0-180 degrees
        return linearMapping<uint16_t>(normalizeAngle(angle), 0, 180, MinPulseWidth, MaxPulseWidth);
    }

    static constexpr uint8_t PulseMicrosecondsToDegrees(uint16_t pulseWidth)
    {
        return (uint8_t)linearMapping<uint16_t>(pulseWidth, MinPulseWidth, MaxPulseWidth, 0, 180);
    }

    static constexpr uint8_t PulseMicrosecondsToDegreesNormalized(uint16_t pulseWidth)
    {
        return normalizeAngle(PulseMicrosecondsToDegrees(pulseWidth));
    }

    static constexpr bool IsAngleInLimitRange(uint8_t angle)
    {
        return angle >= lowerLimit && angle <= upperLimit;
    }

    static constexpr bool IsPulseInLimitRange(uint16_t pulseWidth)
    {
        return IsAngleInLimitRange(PulseMicrosecondsToDegrees(pulseWidth));
    }

    ServoWrapper(IServoImplement &servoInstance) : servo(servoInstance) {}

    uint8_t readDegrees()
    {
        return PulseMicrosecondsToDegrees(this->readMicroseconds());
    }

    uint16_t readMicroseconds()
    {
        return this->servo.readMicroseconds();
    }

    void writeMicroseconds(uint16_t pulseWidth)
    {
        uint16_t originalPulseWidth = this->readMicroseconds();
        pulseWidth = normalizePulse(pulseWidth);
        this->servo.writeMicroseconds(pulseWidth);
        delayMicroseconds(abs((int16_t)(pulseWidth) - (int16_t)(originalPulseWidth)) * pulseChangeRatio);
    }

    void writeDegrees(uint8_t angle)
    {
        this->writeMicroseconds(DegreesToPulseMicrosecondsNormalized(angle));
    }

    uint16_t differMicroseconds(int16_t pulseWidth)
    {
        pulseWidth += this->readMicroseconds();
        this->writeMicroseconds(pulseWidth);
        return this->readMicroseconds();
    }

    uint8_t differDegrees(int16_t angle)
    {
        angle += (int16_t)(this->readDegrees());
        this->writeDegrees((uint8_t)(angle));
        return this->readDegrees();
    }

    virtual void initialize()
    {
        this->writeDegrees(initialPosition);
    }

    virtual void initialize(int pin)
    {
        this->servo.attach(pin);
        this->initialize();
    }

protected:
    IServoImplement &servo;
};

#endif