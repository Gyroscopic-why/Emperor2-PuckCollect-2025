#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "Configs.h"
#include "utils/PDRegulator.h"
#include "utils/Sgn.h"
#include "utils/ElapsedTime.h"

class DriveSample
{
protected:
    enum Direction
    {
        backward = -1,
        stop = 0,
        forward = 1
    };

    void Drive(Direction direction, float turn) // direction = (-1, 0, 1)
    {
        float speed = ROBOT_SPEED * direction;
        float left = speed - turn;
        float right = speed + turn;
        float absedMax = max(abs(right), abs(left));

        if (absedMax > 1)
        {
            right /= absedMax;
            left /= absedMax;
        }

        leftMotor.writePower(left);
        rightMotor.writePower(right);
    }

    float GetOriantation()
    {
        return gyro.getOrientation().x;
    }

    void encoderReset()
    {
        rightMotor.softwareEncoderReset();
        leftMotor.softwareEncoderReset();
    }

    void dropProcess()
    {
        Drive(stop, 0.0);
        encoderReset();
    }

    PDRegulator<float> *PDreg;

public:
    float kp = 1.0f;
    float kd = 1.0f;

    DriveSample() {}; // для очереди

    DriveSample(PDRegulator<float> &PDr)
    {
        PDreg = &PDr;
    }

    virtual ~DriveSample() {}

    virtual void Start()
    {
        encoderReset();

        PDreg->start();
    }

    virtual bool Execute()
    {
        return true;
    }

    void ResetPd()
    {
        PDreg->reset(kp, kd);
    }
};