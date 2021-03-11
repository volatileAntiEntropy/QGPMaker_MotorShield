/******************************************************************

 It will only work with http://www.7gp.cn
 
 ******************************************************************/

#ifndef Encoder_h_
#define Encoder_h_
#include <PinChangeInterrupt.h>

namespace QPGMaker
{
  typedef struct
  {
    uint8_t pin1;
    uint8_t pin2;
    uint8_t state;
    int32_t position;
  } QGPMakerEncoder_internal_state_t;

  static QGPMakerEncoder_internal_state_t *interruptArgs[4]; //ENCODER_ARGLIST_SIZE

  constexpr uint8_t MaxEncoderNumber = 4;

  constexpr uint8_t EncoderPins[MaxEncoderNumber][2] = {{8, 9}, {6, 7}, {3, 2}, {5, 4}};

  class Encoder
  {
  public:
    Encoder(uint8_t num)
    {
      assert(num >= 1 && num <= 4);
      num--;
      uint8_t pin1 = EncoderPins[num][0];
      uint8_t pin2 = EncoderPins[num][1];

#ifdef INPUT_PULLUP
      pinMode(pin1, INPUT_PULLUP);
      pinMode(pin2, INPUT_PULLUP);
#else
      pinMode(pin1, INPUT);
      digitalWrite(pin1, HIGH);
      pinMode(pin2, INPUT);
      digitalWrite(pin2, HIGH);
#endif
      encoder.pin1 = pin1;
      encoder.pin2 = pin2;
      encoder.position = 0;

      delayMicroseconds(2000);
      encoder.state = 0;

      attach_change_interrupt(pin1, &encoder);
      attach_change_interrupt(pin2, &encoder);
    }

    Encoder(uint8_t pin1, uint8_t pin2)
    {
#ifdef INPUT_PULLUP
      pinMode(pin1, INPUT_PULLUP);
      pinMode(pin2, INPUT_PULLUP);
#else
      pinMode(pin1, INPUT);
      digitalWrite(pin1, HIGH);
      pinMode(pin2, INPUT);
      digitalWrite(pin2, HIGH);
#endif
      encoder.pin1 = pin1;
      encoder.pin2 = pin2;
      encoder.position = 0;
      // allow time for a passive R-C filter to charge
      // through the pullup resistors, before reading
      // the initial state
      delayMicroseconds(2000);
      encoder.state = 0;

      attach_change_interrupt(pin1, &encoder);
      attach_change_interrupt(pin2, &encoder);
    }

    inline int32_t readAndReset()
    {
      int32_t ret = encoder.position;
      encoder.position = 0;
      return ret;
    }

    inline int32_t read()
    {
      return encoder.position;
    }

    inline void write(int32_t p)
    {
      encoder.position = p;
    }

  public:
    // update() is not meant to be called from outside Encoder,
    // but it is public to allow static interrupt routines.
    // DO NOT call update() directly from sketches.
    static void update(QGPMakerEncoder_internal_state_t *arg)
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

  private:
    QGPMakerEncoder_internal_state_t encoder;

    static uint8_t attach_change_interrupt(uint8_t pin, QGPMakerEncoder_internal_state_t *state)
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
        return 0;
      }
      return 1;
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
}

#endif
