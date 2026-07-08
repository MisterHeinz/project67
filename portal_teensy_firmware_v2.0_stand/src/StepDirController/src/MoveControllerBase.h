#pragma once

#include "ControllerBase.h"

namespace StepDirController{

    class MoveControllerBase : public ControllerBase{
        public:
            MoveControllerBase(uint32_t updatePeriod = 10);
            
            void setJerkSpeedUnits(double jerk) override; // настройка начальной/конечной скорости (рывка) в единицах измерения в секунду
            void setRegularSpeedUnits(double speed) override; // настройка крейсерской скорости в единицах измерения в секунду
            void setAccelerationUnits(double acceleration) override; // настройка ускорения в единицах измерения в секунду^2
            void setDecelerationUnits(double deceleration) override; // настройка замедления в единицах измерения в секунду^2

            void setOnMoveStarted(void (*callback)());
            void setOnMoveFinished(void (*callback)());
            void setOnEmergensyStoped(void (*callback)());

            template<typename... Axes>
            void moveAsync(Axis& axis, Axes&... axes);
            void stopAsync() override;

            template<typename... Axes>
            void move(Axis& axis, Axes&... axes);
            void stop() override;

            void emergencyStop();

        protected:
            void prepareMove();
            void tick() override; // функия тактирования двигателей, вызываемая генератором

            void (*onMoveStarted)() = nullptr; // callback перед началом движения
            void (*onMoveFinished)() = nullptr; // callback после завершения движения

            void (*onEmergensyStoped)() = nullptr; // callback после аварийной остановки

        private:
            // elapsedMillis timeFromPositionResolved = 0;
            
    };

    template <typename... Axes>
    void MoveControllerBase::moveAsync(Axis& axis, Axes&... axes)
    {
        if (isMoving())
            return;

        movingInProgress = true;
        attachAxes(axis, axes...);
        if (onMoveStarted != nullptr)
                onMoveStarted();
        prepareMove();
        referenceGenerator.begin([this] { this->tick(); }, period);
    }

    template <typename... Axes>
    inline void MoveControllerBase::move(Axis &axis, Axes &...axes)
    {
        moveAsync(axis, axes...);
        while (isMoving())
        {
            delay(1);
        }       
    }
}