# QGPMaker_MotorShield (Revised)
A revised version of QGPMaker's Motor Shield Library V5. **In case of copyright issues, this repository may be deleted or merged!**

## Why Bother?
- In the original version:
  - Bad coding style
  - Fake C++ (A C++ appearance C)
  - Low performance (Unused Variables, Runtime Determination of Constants, ...)
  - Incompatible stepper motor class (copied from adafruit, but hardware shifted from TB6612FNG to AM2878)
- In the revised version:
  - Coding style, type names, variable names fixed
  - Better abstraction with true C++ (interfaces and namespace)
  - Compile-time determination (templates, constexpr)
  - Reduced use of macro (turned into namespace variables and references)
  - Revised object structure (parts belong to shield -> shield links to parts)
  - Higher performance (27.3\% memory performance boost)
  - Stepper motor bug fixes

## Notes
- When create custom encoders, use `configIndex=IEncoder::CustomEncoderIndex`
- `Motor0` and `Motor1` cannot be used with `Stepper0`, `Motor2` and `Motor3` cannot be used with `Stepper1`
