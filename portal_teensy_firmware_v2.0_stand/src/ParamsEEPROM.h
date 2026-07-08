#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "Struct.h"

constexpr int SLOTS = 5;

constexpr int ADDRESS_OFFSET_X = 0;
constexpr int ADDRESS_OFFSET_Y = 8;
constexpr int ADDRESS_OFFSET_Z = 16;
constexpr int ADDRESS_ROTATION = 24;

constexpr int ADDRESS_HOME_X = 32;
constexpr int ADDRESS_HOME_Y = 40;

constexpr int ADDRESS_MAGAZINE_X1 = 48;
constexpr int ADDRESS_MAGAZINE_Y1 = 56;
constexpr int ADDRESS_MAGAZINE_Z1 = 64;

constexpr int ADDRESS_MAGAZINE_X2 = 72;
constexpr int ADDRESS_MAGAZINE_Y2 = 80;
constexpr int ADDRESS_MAGAZINE_Z2 = 88;

constexpr int ADDRESS_MAGAZINE_X3 = 96;
constexpr int ADDRESS_MAGAZINE_Y3 = 104;
constexpr int ADDRESS_MAGAZINE_Z3 = 112;

constexpr int ADDRESS_MAGAZINE_X4 = 120;
constexpr int ADDRESS_MAGAZINE_Y4 = 128;
constexpr int ADDRESS_MAGAZINE_Z4 = 136;

constexpr int ADDRESS_MAGAZINE_X5 = 144;
constexpr int ADDRESS_MAGAZINE_Y5 = 152;
constexpr int ADDRESS_MAGAZINE_Z5 = 160;

constexpr int ADDRESS_SCALE_X = 168;
constexpr int ADDRESS_SCALE_Y = 176;

constexpr int ADDRESS_TOOL_OFFSET_X1 = 254;
constexpr int ADDRESS_TOOL_OFFSET_Y1 = 262;

constexpr int ADDRESS_TOOL_OFFSET_X2 = 270;
constexpr int ADDRESS_TOOL_OFFSET_Y2 = 278;

constexpr int ADDRESS_TOOL_OFFSET_X3 = 286;
constexpr int ADDRESS_TOOL_OFFSET_Y3 = 294;

constexpr int ADDRESS_TOOL_OFFSET_X4 = 302;
constexpr int ADDRESS_TOOL_OFFSET_Y4 = 310;

constexpr int ADDRESS_TOOL_OFFSET_X5 = 318;
constexpr int ADDRESS_TOOL_OFFSET_Y5 = 326;

constexpr int ADDRESS_SERIAL_NUMBER = 1024; // size 2

class ParamsEEPROM{
    public:
        ParamsEEPROM();

        const double* getOffsetX() const;
        const double* getOffsetY() const;
        const double* getOffsetZ() const;
        const double* getRotation() const;

        const double* getScaleX() const;
        const double* getScaleY() const;

        const double* getHomeX() const;
        const double* getHomeY() const;

        const uint16_t* getSerialNumber() const;

        const MagazineSlot* getMagazineSlot(uint8_t slotNum) const;

        const ValuesXY* getToolOffset(uint8_t toolNum) const;

        void setOffsetX(double value);
        void setOffsetY(double value);
        void setOffsetZ(double value);
        void setRotation(double value);
        void setScaleX(double value);
        void setScaleY(double value);
        void setPlazParams(double offsetX, double offsetY, double offsetZ, double rotation, double scaleX, double scaleY);

        void setHomeX(double value);
        void setHomeY(double value);
        void setHome(double x, double y);

        void setSerialNumber(uint16_t number);

        void setMagazineSlot(uint8_t slotNum, MagazineSlot slot);
        void setMagazineSlot(uint8_t slotNum, double x, double y, double z);

        void setToolOffset(uint8_t toolNum, ValuesXY tool);
        void setToolOffset(uint8_t toolNum, double x, double y);

    private:
        double offsetX;
        double offsetY;
        double offsetZ;
        double rotation;
        double scaleX;
        double scaleY;

        double homeX;
        double homeY;

        uint16_t serialNumber;

        MagazineSlot magazineSlots[SLOTS];

        ValuesXY toolOffset[SLOTS];

        template<typename T>
        T readDataFromEEPROM(int address)
        {
            T value;
            EEPROM.get(address, value);

            return value;
        }
};
