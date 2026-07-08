#include "SignalingColumn.h"

SignalingColumn::SignalingColumn(const int _pinRed, const int _pinYellow, const int _pinGreen, const int _pinBuzzer)
    : pinRed(_pinRed), pinYellow(_pinYellow), pinGreen(_pinGreen), pinBuzzer(_pinBuzzer)
{
    pinMode(pinRed, OUTPUT);
    pinMode(pinYellow, OUTPUT);
    pinMode(pinGreen, OUTPUT);
    pinMode(pinBuzzer, OUTPUT);
    isInverted = false;
    isBuzzerEnabled = false;
    buzzerInterval = 500;
    buzzerStartTime = millis();

    setColor(YELLOW);
    disableBuzzer();
}

void SignalingColumn::setColor(Light color)
{
    digitalWriteFast(pinRed, color == RED ? !isInverted : isInverted);
    digitalWriteFast(pinYellow, color == YELLOW ? !isInverted : isInverted);
    digitalWriteFast(pinGreen, color == GREEN ? !isInverted : isInverted);
}

void SignalingColumn::enableBuzzer()
{ 
    if (isBuzzerEnabled)
        return;
         
    buzzerStartTime = millis();
    isBuzzerEnabled = true;
}

void SignalingColumn::disableBuzzer()
{
    if (!isBuzzerEnabled)
        return;        
    
    isBuzzerEnabled = false;
    // digitalWriteFast(pinBuzzer, isInverted);
}

void SignalingColumn::setInvertLogic(bool inverted)
{
    isInverted = inverted;
}

void SignalingColumn::testBuzzer()
{
    digitalWriteFast(pinBuzzer, !isInverted);
	delay(2000);
	digitalWriteFast(pinBuzzer, isInverted);
}

void SignalingColumn::tick()
{
    if (!isBuzzerEnabled)
    {
        digitalWriteFast(pinBuzzer, isInverted);
        return;
    }

    uint32_t passedTime;
    passedTime = millis() - buzzerStartTime;
    
    if (passedTime < buzzerInterval / 2)
        digitalWriteFast(pinBuzzer, !isInverted);
    else if (passedTime < buzzerInterval)
        digitalWriteFast(pinBuzzer, isInverted);   
    else {
        buzzerStartTime += buzzerInterval;
        digitalWriteFast(pinBuzzer, !isInverted);       
    }
}
