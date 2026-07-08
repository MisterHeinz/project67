#include "Axis.h"

namespace StepDirController{

    Axis::Axis(const int _stepPin, const int _dirPin, const int _enablePin)
        :   stepPin(_stepPin), dirPin(_dirPin), enablePin(_enablePin)
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        if (enablePin >= 0)
            pinMode(enablePin, OUTPUT);

        setCurrentPositionInSteps(0);
    }

    Axis &Axis::setStepsPerRevolution(uint32_t steps)
    {
        stepsPerRevolution = steps;
        return *this;
    }

    Axis &Axis::setUnitsPerRevolution(double units)
    {
        unitsPerRevolution = units;
        return *this;
    }

    Axis &Axis::enableLimits(double minUnits, double maxUnits)
    {
        minPositionUnits = maxUnits > minUnits ? minUnits : maxUnits;
        maxPositionUnits = maxUnits > minUnits ? maxUnits : minUnits;
        return enableLimits();
    }

    Axis &Axis::enableLimits()
    {
        usePositionLimits = true;
        return *this;
    }

    Axis &Axis::disableLimits()
    {
        usePositionLimits = false;
        return *this;
    }

    Axis &Axis::enable()
    {
        if (enablePin >= 0){
            digitalWriteFast(enablePin, !inverseEnable);
        }
        return *this;
    }

    Axis &Axis::disable()
    {
        if (enablePin >= 0){
            digitalWriteFast(enablePin, inverseEnable);
        }
        return *this;
    }

    Axis &Axis::setInverseDirPin(bool isInverted)
    {
        inverseDir = isInverted;
        return *this;
    }

    Axis &Axis::setInverseStepPin(bool isInverted)
    {
        inverseStep = isInverted;
        return *this;
    }

    Axis &Axis::setInverseEnablePin(bool isInverted)
    {
        inverseEnable = isInverted;
        return *this;
    }

    Axis &Axis::setCurrentPositionInUnits(double units)
    {
        int32_t steps = round(unitsToSteps(units));
        return setCurrentPositionInSteps(steps);
    }

    Axis &Axis::setCurrentPositionInSteps(int32_t steps)
    {
        currentPosition = steps;
        return *this;
    }

    Axis &Axis::setOnStepDone(void (*callback)())
    {
        onStepDone = callback;
        return *this;
    }

    bool Axis::setTargetPositionAbsoluteInUnits(double units)
    {
        return setTargetPositionRelativeInUnits(units - getPositionInUnits());
    }

    bool Axis::setTargetPositionAbsoluteInSteps(int32_t steps)
    {
        return setTargetPositionRelativeInSteps(steps - getPositionInSteps());
    }

    bool Axis::setTargetPositionRelativeInUnits(double units)
    {
        int32_t steps = round(unitsToSteps(units));

        return setTargetPositionRelativeInSteps(steps);
    }

    bool Axis::setTargetPositionRelativeInSteps(int32_t steps)
    {
        if ((usePositionLimits) && 
            ((getPositionInSteps() + steps > round(unitsToSteps(maxPositionUnits))) || 
             (getPositionInSteps() + steps < round(unitsToSteps(minPositionUnits)))))
            return false;

        Direction dir = steps > 0 ? Direction::POSITIVE : Direction::NEGATIVE;
        setDirection(dir);

        movementUnits = stepsToUnits(steps);
        movementSteps = abs(steps);

        return true;
    }

    double Axis::getPositionInUnits() const
    {
        noInterrupts();
        double position = stepsToUnits(currentPosition);
        interrupts();
        return position;
    }

    int32_t Axis::getPositionInSteps() const
    {
        noInterrupts();
        int32_t position = currentPosition;
        interrupts();
        return position;
    }

    uint32_t Axis::getStepsPerRevolution() const
    {
        return stepsPerRevolution;
    }

    double Axis::getUnitsPerRevolution() const
    {
        return unitsPerRevolution;
    }

    double Axis::stepsToUnits(int32_t steps) const
    {
        return steps / (double)stepsPerRevolution * unitsPerRevolution;
    }

    double Axis::unitsToSteps(double units) const
    {
        return units / unitsPerRevolution * stepsPerRevolution;
    }

    Axis &Axis::setDirection(Direction newDirection)
    {
        direction = newDirection;
        uint8_t dir = direction == Direction::POSITIVE ? !inverseDir : inverseDir;
        digitalWriteFast(dirPin, dir);
        return *this;
    }

    Axis &Axis::stepOn()
    {
        digitalWriteFast(stepPin, !inverseStep);
        return *this;
    }

    Axis &Axis::stepOff()
    {
        digitalWriteFast(stepPin, inverseStep);
        return *this;
    }

    Axis & Axis::stepDone()
    {
        direction == Direction::POSITIVE ? currentPosition++ : currentPosition--;
        stepsDone++;
        if (onStepDone != nullptr)
            onStepDone();
            
        return *this;
    }

    double Axis::getMaxLimitUnits() const
    {
        return maxPositionUnits;
    }

    double Axis::getMinLimitUnits() const
    {
        return minPositionUnits;
    }
}