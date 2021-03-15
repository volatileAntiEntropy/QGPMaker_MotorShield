/******************************************************************

 It will only work with http://www.7gp.cn
 
 ******************************************************************/

#ifndef Encoder_h_
#define Encoder_h_
#include <assert.h>
#include <PinChangeInterrupt.h>

namespace QGPMaker
{
  typedef struct
  {
    uint8_t pin1;
    uint8_t pin2;
    uint8_t state;
    int32_t position;
  } EncoderInternalState;

  static EncoderInternalState *interruptArgs[4]; //ENCODER_ARGLIST_SIZE

  class IEncoder
  {
  public:
    static constexpr uint8_t MaxInstanceNumber = 4;
    static constexpr uint8_t PinConfigs[MaxInstanceNumber][2] = {{8, 9}, {6, 7}, {3, 2}, {5, 4}};

  public:
    // update() is not meant to be called from outside Encoder,
    // but it is public to allow static interrupt routines.
    // DO NOT call update() directly from sketches.
    static void update(EncoderInternalState *arg)
    {
      uint8_t p1val = digitalRead(arg->pin1);
      uint8_t p2val = digitalRead(arg->pin2);
      uint8_t state = arg->state & 3;
      if (p1val)
        state |= 4;
      if (p2val)
        state |= 8;
      arg->state = (state >> 2);
      switch (state)
      {
      case 1:
      case 7:
      case 8:
      case 14:
        arg->position++;
        return;
      case 2:
      case 4:
      case 11:
      case 13:
        arg->position--;
        return;
      case 3:
      case 12:
        arg->position += 2;
        return;
      case 6:
      case 9:
        arg->position -= 2;
        return;
      }
    }

  protected:
    EncoderInternalState encoder;

    static bool attach_change_interrupt(uint8_t pin, EncoderInternalState *state)
    {
      switch (pin)
      {
      case 2:
      case 3:
        interruptArgs[0] = state;
        attachPCINT(digitalPinToPCINT(pin), isr0, CHANGE);
        break;
      case 4:
      case 5:
        interruptArgs[1] = state;
        attachPCINT(digitalPinToPCINT(pin), isr1, CHANGE);
        break;
      case 6:
      case 7:
        interruptArgs[2] = state;
        attachPCINT(digitalPinToPCINT(pin), isr2, CHANGE);
        break;
      case 8:
      case 9:
        interruptArgs[3] = state;
        attachPCINT(digitalPinToPCINT(pin), isr3, CHANGE);
        break;
      default:
        return false;
      }
      return true;
    }

    static void isr0(void)
    {
      update(interruptArgs[0]);
    }

    static void isr1(void)
    {
      update(interruptArgs[1]);
    }

    static void isr2(void)
    {
      update(interruptArgs[2]);
    }

    static void isr3(void)
    {
      update(interruptArgs[3]);
    }
  };

  template <uint8_t configIndex, uint8_t Pin1 = IEncoder::PinConfigs[configIndex][0], uint8_t Pin2 = IEncoder::PinConfigs[configIndex][1]>
  class Encoder : public IEncoder
  {
  public:
    Encoder()
    {
#ifdef INPUT_PULLUP
      pinMode(Pin1, INPUT_PULLUP);
      pinMode(Pin2, INPUT_PULLUP);
#else
      pinMode(Pin1, INPUT);
      digitalWrite(Pin1, HIGH);
      pinMode(Pin2, INPUT);
      digitalWrite(Pin2, HIGH);
#endif
      encoder.pin1 = Pin1;
      encoder.pin2 = Pin2;
      encoder.position = 0;
      // allow time for a passive R-C filter to charge
      // through the pullup resistors, before reading
      // the initial state
      delayMicroseconds(2000);
      encoder.state = 0;

      attach_change_interrupt(Pin1, &encoder);
      attach_change_interrupt(Pin2, &encoder);
    }

    inline int32_t readAndReset()
    {
      int32_t position = this->read();
      encoder.position = 0;
      return position;
    }

    inline int32_t read() const
    {
      return encoder.position;
    }

    inline void write(int32_t pos)
    {
      encoder.position = pos;
    }
  };
}

#endif
