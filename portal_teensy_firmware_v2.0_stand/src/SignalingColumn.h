#include <Arduino.h>

class SignalingColumn
{
    public:
        enum Light
        {
            RED,
            YELLOW,
            GREEN
        };

        SignalingColumn(const int _pinRed, const int _pinYellow, const int _pinGreen, const int _pinBuzzer);
        void setColor(Light color);
        void enableBuzzer();
        void disableBuzzer();
        void testBuzzer();
        void setInvertLogic(bool inverted);


        void tick();

    private:
        const int pinRed;
        const int pinYellow;
        const int pinGreen;
        const int pinBuzzer;
        bool isInverted;

        volatile bool isBuzzerEnabled;

        uint32_t buzzerInterval;
        volatile uint32_t buzzerStartTime;
};