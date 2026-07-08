#include "MoveControllerBase.h"

namespace StepDirController{
    MoveControllerBase::MoveControllerBase(uint32_t updatePeriod) : ControllerBase(updatePeriod)
    {
    }

    void MoveControllerBase::setJerkSpeedUnits(double jerk)
    {
        jerkSpeedUnits = abs(jerk);

        if (regularSpeedUnits < jerkSpeedUnits)
            regularSpeedUnits = jerkSpeedUnits;
    }

    void MoveControllerBase::setRegularSpeedUnits(double speed)
    {
        speed = abs(speed);
        regularSpeedUnits = speed > jerkSpeedUnits ? speed : jerkSpeedUnits;
    }

    void MoveControllerBase::setAccelerationUnits(double acceleration)
    {
        accelerationUnits = abs(acceleration);
    }

    void MoveControllerBase::setDecelerationUnits(double deceleration)
    {
        decelerationUnits = abs(deceleration) * -1.0F;
    }

    void MoveControllerBase::setOnMoveStarted(void (*callback)())
    {
        onMoveStarted = callback;
    }

    void MoveControllerBase::setOnMoveFinished(void (*callback)())
    {
        onMoveFinished = callback;
    }

    void MoveControllerBase::setOnEmergensyStoped(void (*callback)())
    {
        onEmergensyStoped = callback;
    }

    void MoveControllerBase::stopAsync()
    {
        if (!isMoving()) // нет движения, останавливать нечего
            return;

        Axis** axes = axisList;
        noInterrupts();
        if (leadAxis->decelerationStartStep == leadAxis->movementStepsInWork){ // если отсутствует торможение перед остановкой
            stepsAll = 0;
            while (*(axes) != nullptr)
            {
                (*axes)->movementStepsInWork = (*axes)->stepsDone + 1;
                stepsAll += (*axes)->movementStepsInWork;
                (*axes++);
            }
            interrupts();
            return;
        }

        if (leadAxis->stepsDone + 1 >= leadAxis->decelerationStartStep){ // если торможение уже в процессе
            interrupts();
            return; // ничего не делаем, уже идет торможение
        }

        if (leadAxis->stepsDone + 1 < leadAxis->accelerationSteps){ // запуск досрочного торможения на этапе разгона
            stepsAll = 0;
            while (*(axes) != nullptr) {
                if ((*axes)->movementStepsInWork == 1)
                {
                    (*axes)->stepDuration = (micros() - (*axes)->startStepTime) * 2;
                }
                if ((*axes)->movementStepsInWork > 1)
                {
                    (*axes)->regularSpeed = sqrt(2.0F * ((*axes)->stepsDone + 1.0F) * (*axes)->acceleration + sq((*axes)->startSpeed));
                    (*axes)->regularSpeed = 1000000/round(1000000/(*axes)->regularSpeed); // уточнение скорости через округленную длительность периода
                    uint32_t newDecelerationSteps = round(sq((*axes)->endSpeed) - sq((*axes)->regularSpeed)) / (2.0F * (*axes)->decleleration) + 1;
                    (*axes)->accelerationSteps = (*axes)->stepsDone + 1;
                    (*axes)->movementStepsInWork = (*axes)->stepsDone + newDecelerationSteps + 1;
                    (*axes)->decelerationStartStep = (*axes)->movementStepsInWork - newDecelerationSteps;
                    stepsAll += (*axes)->movementStepsInWork;
                }
                (*axes++);
            }
            interrupts();
            return;
        }

        if (leadAxis->stepsDone + 1 < leadAxis->decelerationStartStep){ // запуск досрочного торможения
            stepsAll = 0;
            while (*(axes) != nullptr) {
                if ((*axes)->movementStepsInWork == 1)
                {
                    (*axes)->stepDuration = (micros() - (*axes)->startStepTime) * 2;
                }
                if ((*axes)->movementStepsInWork > 1)
                {
                    (*axes)->movementStepsInWork -= (*axes)->decelerationStartStep - (*axes)->stepsDone - 1;
                    (*axes)->decelerationStartStep = (*axes)->stepsDone + 1;
                    stepsAll += (*axes)->movementStepsInWork;
                }
                (*axes++);
            }
        }
        interrupts();
    }

    void MoveControllerBase::stop()
    {
        stopAsync();
        while (isMoving())
        {
            delay(1);
        }
    }

    void MoveControllerBase::emergencyStop()
    {
        if (!isMoving()) // нет движения, останавливать нечего
            return;

        ControllerBase::emergencyStop();
        if (onEmergensyStoped != nullptr)
            onEmergensyStoped();
    }

    void MoveControllerBase::prepareMove()
    {
        leadAxis = axisList[0];

        if (leadAxis == nullptr)
            return;

        stepsDoneAll = 0;
        stepsAll = 0;

        Axis** axes = axisList;
        while (*(axes) != nullptr)
        {
           if (abs(leadAxis->movementUnits) < abs((*axes)->movementUnits))
                leadAxis = (*axes);

            (*axes)->stepsDone = 0;
            (*axes)->movementStepsInWork = (*axes)->movementSteps;
            stepsAll += (*axes)->movementStepsInWork;
            (*axes++);
        }
        
        double accelerationPath = ((accelerationUnits > 0.1F) || (accelerationUnits < -0.1F)) ? ((sq(regularSpeedUnits) - sq(jerkSpeedUnits)) / (2.0F * accelerationUnits)) : 0.0F;
        double decelerationPath = ((decelerationUnits > 0.1F) || (decelerationUnits < -0.1F)) ? ((sq(jerkSpeedUnits) - sq(regularSpeedUnits)) / (2.0F * decelerationUnits)) : 0.0F;

        double maxPath = abs(leadAxis->movementUnits);
        
        if (accelerationPath + decelerationPath > maxPath) {
            double delta = (accelerationPath + decelerationPath - maxPath) * (accelerationPath / (accelerationPath + decelerationPath));
            regularSpeedUnits = sqrt(2 * accelerationUnits * (accelerationPath - delta) + sq(jerkSpeedUnits));
            accelerationPath = (sq(regularSpeedUnits) - sq(jerkSpeedUnits)) / (2.0F * accelerationUnits);
            decelerationPath = (sq(jerkSpeedUnits) - sq(regularSpeedUnits)) / (2.0F * decelerationUnits);
        }

        double accelerationTime = ((accelerationUnits > 0.1F) || (accelerationUnits < -0.1F)) ? (regularSpeedUnits - jerkSpeedUnits) / accelerationUnits : 0.0F; // теоретическое время разгона в секундах
        double decelerationTime = ((decelerationUnits > 0.1F) || (decelerationUnits < -0.1F)) ? (jerkSpeedUnits - regularSpeedUnits) / decelerationUnits : 0.0F; // теоретическое время торможения в секундах
        double regularTime = (maxPath - accelerationPath - decelerationPath) / regularSpeedUnits; // теоретическое время движения на крейсерской скорости в секундах
        
        axes = axisList;
        while (*(axes) != nullptr)
        {
            double syncCoefficient = abs((*axes)->movementUnits) / maxPath;

            (*axes)->accelerationSteps = round((*axes)->unitsToSteps(accelerationPath * syncCoefficient));
            uint32_t decelerationSteps = round((*axes)->unitsToSteps(decelerationPath * syncCoefficient));

            if ((*axes)->accelerationSteps + decelerationSteps > (*axes)->movementStepsInWork) // уточнение из-за округлений
                decelerationSteps > 0 ? decelerationSteps-- : (*axes)->accelerationSteps--; // из-за округлений может появиться лишний шаг

            (*axes)->decelerationStartStep = (*axes)->movementStepsInWork - decelerationSteps;

            double regularTimeTmp = regularTime;
            regularTimeTmp += (*axes)->accelerationSteps > 0 ? 0.0F : accelerationTime;
            regularTimeTmp += decelerationSteps > 0 ? 0.0F : decelerationTime;

            uint32_t regularStepsTmp = (*axes)->decelerationStartStep - (*axes)->accelerationSteps;
            double maxSpeedTmp = (*axes)->unitsToSteps(regularSpeedUnits * syncCoefficient);
            (*axes)->regularSpeed = ((regularTimeTmp >= (1.0F / maxSpeedTmp)) && (regularStepsTmp > 0)) ? (regularStepsTmp / regularTimeTmp) : maxSpeedTmp;
            (*axes)->regularSpeed = 1000000/round(1000000/(*axes)->regularSpeed); // уточнение скорости через округленную длительность периода

            double jerkSpeedTmp = (*axes)->unitsToSteps(jerkSpeedUnits); // рывок шагов/сек ведущей оси в шагах текущей
            double accelerationTmp = (*axes)->unitsToSteps(accelerationUnits); // ускорение шагов/сек^2 ведущей оси в шагах текущей
            double decelerationTmp = (*axes)->unitsToSteps(decelerationUnits); // замедление шагов/сек^2 ведущей оси в шагах текущей
            uint32_t stepsMaxTmp = round((*axes)->unitsToSteps(maxPath)); // шагов ведущей оси в шагах текущей           

            (*axes)->startSpeed = (*axes)->regularSpeed;
            (*axes)->endSpeed = (*axes)->regularSpeed;

            if ((*axes)->accelerationSteps > 0)
                (*axes)->startSpeed = (abs(accelerationTmp) * jerkSpeedTmp) / (sqrt(2 * (stepsMaxTmp / (double)(*axes)->movementStepsInWork - 1) * abs(accelerationTmp) + sq(jerkSpeedTmp)) * jerkSpeedTmp - sq(jerkSpeedTmp) + abs(accelerationTmp));
            if (decelerationSteps > 0)
                (*axes)->endSpeed = (abs(decelerationTmp) * jerkSpeedTmp) / (sqrt(2 * (stepsMaxTmp / (double)(*axes)->movementStepsInWork - 1) * abs(decelerationTmp) + sq(jerkSpeedTmp)) * jerkSpeedTmp - sq(jerkSpeedTmp) + abs(decelerationTmp));

            (*axes)->startSpeed = 1000000/round(1000000/(*axes)->startSpeed); // уточнение скорости через округленную длительность периода
            (*axes)->endSpeed = 1000000/round(1000000/(*axes)->endSpeed); // уточнение скорости через округленную длительность периода
            
            (*axes)->acceleration = (*axes)->accelerationSteps > 1 ? ((sq((*axes)->regularSpeed) - sq((*axes)->startSpeed)) / (2 * ((*axes)->accelerationSteps - 1))) : 0.0F;
            (*axes)->decleleration = decelerationSteps > 1 ? ((sq((*axes)->endSpeed) - sq((*axes)->regularSpeed)) / (2 * (decelerationSteps - 1))) : 0.0F;

            (*axes)->stepDuration = (*axes)->startSpeed > 0.0F ? round(1000000 / (*axes)->startSpeed) : 0;

            (*axes++);
        }

        axes = axisList;
        startGlobalTime = micros();
        while (*(axes) != nullptr)
        {
            (*axes++)->startStepTime = startGlobalTime;
        }
    }

    void MoveControllerBase::tick()
    {
        if (stepsDoneAll < stepsAll) {
            double tmpSpeed;
            Axis** axes = axisList;
            while (*(axes) != nullptr)
            {
                if ((*axes)->stepsDone < (*axes)->movementStepsInWork) {
                    if (micros() - (*axes)->startStepTime < (*axes)->stepDuration / 2) {
                        (*axes)->stepOn();
                    } else if (micros() - (*axes)->startStepTime < (*axes)->stepDuration) {
                        (*axes)->stepOff(); 
                    } else {
                        (*axes)->stepDone();
                        stepsDoneAll++;
                        if ((*axes)->stepsDone < (*axes)->movementStepsInWork) {
                            (*axes)->stepOn(); // чтобы не сбивать тактирование двигателей
                            (*axes)->startStepTime += (*axes)->stepDuration;
                            if ((*axes)->stepsDone < (*axes)->accelerationSteps) {
                                tmpSpeed = sqrt(sq((*axes)->startSpeed) + 2 * (*axes)->acceleration * (*axes)->stepsDone);
                                (*axes)->stepDuration = round(1000000.0F / tmpSpeed);
                            } else if (((*axes)->stepsDone >= (*axes)->accelerationSteps) && ((*axes)->stepsDone < (*axes)->decelerationStartStep)) {
                                (*axes)->stepDuration = round(1000000.0F / (*axes)->regularSpeed);
                            } else {
                                tmpSpeed = sqrt(sq((*axes)->regularSpeed) + 2 * (*axes)->decleleration * ((*axes)->stepsDone - (*axes)->decelerationStartStep));
                                (*axes)->stepDuration = round(1000000.0F / tmpSpeed);
                            }
                        }
                    }
                }
                (*axes++);
            }
            // timeFromPositionResolved = 0;
        } else{
            // if (timeFromPositionResolved < 1000)
            //     return;
            referenceGenerator.end();
            Axis** axes = axisList;
            while (*(axes) != nullptr)
            {
                (*axes++)->setTargetPositionRelativeInSteps(0);
            }
            movingInProgress = false;

            if (onMoveFinished != nullptr)
                onMoveFinished();
        }
    }
}