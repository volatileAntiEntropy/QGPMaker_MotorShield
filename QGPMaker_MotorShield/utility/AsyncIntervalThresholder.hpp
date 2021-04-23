#ifndef _AsyncIntervalThresholder_hpp
#define _AsyncIntervalThresholder_hpp

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class AsyncIntervalThresholder
{
public:
    AsyncIntervalThresholder() : currentTimeMilliseconds(0) {}

    inline bool isInitialized() const
    {
        return this->currentTimeMilliseconds == 0;
    }

    inline unsigned long lastTaskStartTime() const
    {
        return this->currentTimeMilliseconds;
    }

    void startTask()
    {
        this->currentTimeMilliseconds = millis();
    }

    bool isTaskFinished(unsigned long estimatedTaskDuration)
    {
        return !(this->isInitialized()) || (millis() - this->lastTaskStartTime() > estimatedTaskDuration);
    }

private:
    unsigned long currentTimeMilliseconds;
};

#endif