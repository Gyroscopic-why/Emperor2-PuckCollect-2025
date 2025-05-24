#pragma once

#include <Arduino.h>
#include "DriveTrain/DriveSamples.h"
#include "utils/Queue.h"
#include "utils/ElapsedTime.h"

Queue<DriveSample *> _trajectory;
ElapseTime _Timer;

void driveTrainBegin()
{
    PDRegulator<float> MainPDregulator(0.1f, 0.1f); // здесь надо вставить каэфициенты для первого действия (у нас это DriveForwardToTheLimit)

    _trajectory.enqueue(new DriveForwardToTheLimit(MainPDregulator, ETALON_DISTANCE));

    // _trajectory.enqueue(new DriveForwardToTheLimit(MainPDregulator, ETALON_DISTANCE));

    // if (IS_GYRO)
    //     _trajectory.enqueue(new TurnByGlobalCoordinates(MainPDregulator, 90));

    // for (int step = 1; step < 4; step++)
    // { // три круга
    //     for (int actions = 1; actions < 5; actions++)
    //     { // в каждом круге полный проезд, из четырех проездов по стене, и четырех поворотов
    //         _trajectory.enqueue(new DrivingAlongTheWall(MainPDregulator, ETALON_DISTANCE * step));

    //         if (IS_GYRO)
    //             _trajectory.enqueue(new TurnByGlobalCoordinates(MainPDregulator, chopDegrees(90 + 90 * actions)));
    //         else
    //             _trajectory.enqueue(new TurnToTheWall(MainPDregulator, ETALON_DISTANCE * step));
    //     }
    // }

    // if (IS_GYRO)
    // {
    //     _trajectory.enqueue(new TurnByGlobalCoordinates(MainPDregulator, -90));
    //     _trajectory.enqueue(new DriveForwardToTheLimit(MainPDregulator, ETALON_DISTANCE));
    //     _trajectory.enqueue(new TurnByGlobalCoordinates(MainPDregulator, -180));
    //     _trajectory.enqueue(new DriveForwardToTheLimit(MainPDregulator, ETALON_DISTANCE));
    //     _trajectory.enqueue(new TurnByGlobalCoordinates(MainPDregulator, 0));
    // }
    // остается только дописать генерацию рандомной езды (опционально), или есть идея интереснее, но ее позже расскажу
}

void driveTrainStart()
{
    _Timer.reset();

    if (!_trajectory.isEmpty())
        _trajectory.front()->Start();
}

void driveTrainUpdate()
{
    if (_trajectory.isEmpty())
        return;

    if (_trajectory.front()->Execute() || (EXECUTION_LIMIT - _Timer.seconds()) > TIME_ERROR)
    { // прирывание по времени
        delete _trajectory.frontAndDequeue();

        if (!_trajectory.isEmpty())
        {
            _trajectory.front()->Start();
            _trajectory.front()->ResetPd();
        }
    }
}