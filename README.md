# QGPMaker_MotorShield (Revised)
A revised version of QGPMaker's Motor Shield Library V5. **In case of copyright issues, this repository may be deleted or merged!*

## Why Bother?
- In the original version:
  - Bad coding style
  - Fake C++ (A C++ appearance C)
  - Low Performance (Unused Variables, Runtime Determination of Constants, ...)
- In the revised version:
  - Coding style, type names, variable names fixed
  - Better abstraction with true C++ (interfaces and namespace)
  - Compile-time determination (templates, constexpr)
  - Reduced use of macro (turned into namespace variables and references)
  - Revised object structure (parts belong to shield -> shield links to parts)
  - Higher performance (27.3\% memory performance boost)
