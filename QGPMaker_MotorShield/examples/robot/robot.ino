#define DEBUG

#include <QGPMaker_MotorShield.hpp>
#include <PS2X_lib.h>
using namespace QGPMaker;

constexpr uint8_t clockPin = 13;
constexpr uint8_t commandPin = 11;
constexpr uint8_t attentionPin = 10;
constexpr uint8_t dataPin = 12;

constexpr int liftDown = 20;
constexpr int liftUp = 50;
constexpr int clipClose = 80;
constexpr int clipOpen = 120;

QGPMaker::MotorShield shield;
QGPMaker::Servo<0> &clip = QGPMaker::Servo0;
QGPMaker::Servo<3> &lift = QGPMaker::Servo3;
QGPMaker::DCMotor<0> &leftMotor = QGPMaker::Motor0;
QGPMaker::DCMotor<2> &rightMotor = QGPMaker::Motor2;
PS2X ps2x;

bool isSlowMode = false;

// the setup function runs once when you press reset or power the board
void setup()
{
#ifdef DEBUG
    while (!Serial)
        ;
    Serial.begin(115200);
#endif
    shield.begin(50);
    int controllerError = 0x01;
    while (controllerError)
    {
        controllerError = ps2x.config_gamepad(clockPin, commandPin, attentionPin, dataPin);
        delay(100);
    }
    for (size_t i = 0; i < 50; i++)
    {
        ps2x.read_gamepad();
        delay(10);
    }
}

// the loop function runs over and over again until power down or reset
void loop()
{
    ps2x.read_gamepad();
    delay(5);
    uint8_t speed = 255;
    if (ps2x.Button(PSB_SELECT))
    {
        isSlowMode = !isSlowMode;
        speed = (isSlowMode) ? (55) : (255);
        delay(20);
    }
    //Motor
    if (ps2x.Button(PSB_PAD_UP))
    {
        rightMotor.run(speed, FORWARD);
        leftMotor.run(speed, FORWARD);
    }
    else if (ps2x.Button(PSB_PAD_DOWN))
    {
        leftMotor.run(speed, BACKWARD);
        rightMotor.run(speed, BACKWARD);
    }
    else if (ps2x.Button(PSB_PAD_LEFT))
    {
        leftMotor.run(55, BACKWARD);
        rightMotor.run(55, FORWARD);
    }
    else if (ps2x.Button(PSB_PAD_RIGHT))
    {
        leftMotor.run(55, FORWARD);
        rightMotor.run(55, BACKWARD);
    }
    else
    {
        leftMotor.run(0, RELEASE);
        rightMotor.run(0, RELEASE);
    }
    //Servo Delicate
    if (ps2x.Button(PSB_L1))
    {
        int S3 = min(lift.readDegrees() + 1, 360);
        lift.writeDegrees(S3);
    }
    else if (ps2x.Button(PSB_L2))
    {
        int S3 = max(lift.readDegrees() - 1, 0);
        lift.writeDegrees(S3);
    }
    if (ps2x.Button(PSB_R1))
    {
        int S0 = min(clip.readDegrees() + 1, 360);
        clip.writeDegrees(S0);
    }
    else if (ps2x.Button(PSB_R2))
    {
        int S0 = max(clip.readDegrees() - 1, 0);
        clip.writeDegrees(S0);
    }
    //Servo Modes
    if (ps2x.Button(PSB_TRIANGLE))
    {
        lift.writeDegrees(liftDown);
    }
    else if (ps2x.Button(PSB_CIRCLE))
    {
        clip.writeDegrees(clipClose);
    }
    else if (ps2x.Button(PSB_CROSS))
    {
        lift.writeDegrees(liftUp);
    }
    else if (ps2x.Button(PSB_SQUARE))
    {
        clip.writeDegrees(clipOpen);
    }
}

#ifdef DEBUG
void serialEvent()
{
    String command = Serial.readString();
    if (command[0] == ',')
    {
        lift.writeDegrees(command.substring(1).toInt());
        Serial.print(F("Lift Servo Position: "));
        Serial.println(lift.readDegrees());
    }
    else if (command[0] == '.')
    {
        clip.writeDegrees(command.substring(1).toInt());
        Serial.print(F("Clip Servo Position: "));
        Serial.println(clip.readDegrees());
    }
    else
    {
        Serial.println(F("Unknown Command!"));
    }
}
#endif
