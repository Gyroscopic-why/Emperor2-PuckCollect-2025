#pragma once

#include <Arduino.h>
#include "Devices.h"
#include "Configs.h"
#include "utils/PDRegulator.h"
#include "utils/Queue.h"
#include "utils/Sgn.h"
#include "utils/ElapsedTime.h"
#include "DriveTrain/DriveSample.h"
#include "Intake.h"

/*
enum SimpleActions{
    DriveToWall,        // +
    TurnOnWall,         // +
    DriveAlongWall,     // +
    DriveOnEncoder,     // +
    TurnLocal,          // +
    TurnGlobal          // +
};
*/

class DriveForwardToTheLimit : public DriveSample
{
    float _distance;

public:
    DriveForwardToTheLimit(PDRegulator<float> &PDr, float distance) : DriveSample(PDr)
    {
        _distance = distance;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    bool Execute() override
    { // энкодеры сбрасываются, все норм. ПД тоже сбрасывается
        if (forwardDistanceFilter.getCurrentValue() > _distance)
        {
            Drive(forward, PDreg->update(rightMotor.readCurrentPosition() - leftMotor.readCurrentPosition()));
            return false;
        }

        return true;
    }
};

class TurnToTheWall : public DriveSample
{
    float _distance;

public:
    TurnToTheWall(PDRegulator<float> &PDr, float distance) : DriveSample(PDr)
    {
        _distance = distance;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    bool Execute() override
    {
        if (forwardDistanceFilter.getCurrentValue() < _distance)
        {
            Drive(stop, -ROBOT_SPEED);
            return false;
        }

        return true;
    }
};

class DrivingAlongTheWall : public DriveSample
{
    float _distance;

public:
    DrivingAlongTheWall(PDRegulator<float> &PDr, float distance) : DriveSample(PDr)
    {
        _distance = distance;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    bool Execute() override
    {
        if (forwardDistanceFilter.getCurrentValue() > _distance)
        {
            float errValue = rightDistanceFilter.getCurrentValue() - _distance;
            Drive(forward, PDreg->update(errValue));
            return false;
        }

        return true;
    }
};

class TravelByEncoderValue : public DriveSample
{
    float _encPos;

public:
    TravelByEncoderValue(PDRegulator<float> &PDr, float encPos) : DriveSample(PDr)
    {
        _encPos = encPos;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    bool Execute() override
    {
        if (((leftMotor.readCurrentPosition() + rightMotor.readCurrentPosition()) / 2) > _encPos)
        {
            Drive(backward, PDreg->update(rightMotor.readCurrentPosition() - leftMotor.readCurrentPosition()));
            return false;
        }

        return true;
    }
};

class TurnByGlobalCoordinates : public DriveSample
{
    float _targetRotate;

public:
    TurnByGlobalCoordinates(PDRegulator<float> &PDr, float targetRotate) : DriveSample(PDr)
    {
        _targetRotate = targetRotate;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    bool Execute() override
    {
        if (!IS_GYRO)
            return true; // просто пропустит

        float error = chopDegrees(_targetRotate - GetOriantation());

        if (abs(error) > ANGLE_ERROR)
        {
            Drive(stop, ROBOT_SPEED * sgn(error));
            return false;
        }

        return true;
    }
};

class TurnByLocalCoordinates : public DriveSample
{
private:
    float _targetTurn;
    float _startCoords;

public:
    TurnByLocalCoordinates(PDRegulator<float> &PDr, float targetTurn) : DriveSample(PDr)
    {
        _targetTurn = targetTurn;
        kp = 1.0f; // нужны норм каэфициенты!
        kd = 1.0f;
    }

    void Start() override
    {
        DriveSample::Start();

        if (IS_GYRO)
            _startCoords = GetOriantation();
    }

    bool Execute() override
    {
        if (IS_GYRO)
        {
            float error = chopDegrees(_targetTurn - (_startCoords - GetOriantation()));

            if (abs(error) > ANGLE_ERROR)
            {
                Drive(stop, ROBOT_SPEED * sgn(error));
                return false;
            }

            return true;
        }
        else
        {
            float error = chopDegrees(_targetTurn - chopDegrees(((leftMotor.readCurrentPosition() - rightMotor.readCurrentPosition()) / SINGLE_ENCODER_STEP * WHEEL_DISTANCE) / (90 * WHEEL_DISTANCE)));

            if (abs(error) > ANGLE_ERROR)
            {
                Drive(stop, ROBOT_SPEED * sgn(error));
                return false;
            }

            return true;
        }
    }
};