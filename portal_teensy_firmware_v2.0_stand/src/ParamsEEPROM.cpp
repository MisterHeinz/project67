#include "ParamsEEPROM.h"

ParamsEEPROM::ParamsEEPROM()
{
    //TODO: удалить
    // for (int i = ADDRESS_SERIAL_NUMBER; i < ADDRESS_SERIAL_NUMBER + 2; i++)
    // {
    //     EEPROM.update(i, 255);
    // }
    
    offsetX = readDataFromEEPROM<double>(ADDRESS_OFFSET_X);
    offsetY = readDataFromEEPROM<double>(ADDRESS_OFFSET_Y);
    offsetZ = readDataFromEEPROM<double>(ADDRESS_OFFSET_Z);
    rotation = readDataFromEEPROM<double>(ADDRESS_ROTATION);
    scaleX = readDataFromEEPROM<double>(ADDRESS_SCALE_X);
    scaleY = readDataFromEEPROM<double>(ADDRESS_SCALE_Y);

    if (isnan(offsetX))
        setOffsetX(0.0F);
    if (isnan(offsetY))
        setOffsetY(0.0F);
    if (isnan(offsetZ))
        setOffsetZ(0.0F);

    if (isnan(rotation))
        setRotation(0.0F);

    if (isnan(scaleX))
        setScaleX(1.0F);
    if (isnan(scaleY))
        setScaleY(1.0F);

    homeX = readDataFromEEPROM<double>(ADDRESS_HOME_X);
    homeY = readDataFromEEPROM<double>(ADDRESS_HOME_Y);

    serialNumber = readDataFromEEPROM<uint16_t>(ADDRESS_SERIAL_NUMBER);

    for (int i = 0; i < SLOTS; i++)
    {
        magazineSlots[i].X = readDataFromEEPROM<double>(ADDRESS_MAGAZINE_X1 + 24 * i);
        magazineSlots[i].Y = readDataFromEEPROM<double>(ADDRESS_MAGAZINE_Y1 + 24 * i);
        magazineSlots[i].Z = readDataFromEEPROM<double>(ADDRESS_MAGAZINE_Z1 + 24 * i);

        toolOffset[i].X = readDataFromEEPROM<double>(ADDRESS_TOOL_OFFSET_X1 + 16 * i);
        toolOffset[i].Y = readDataFromEEPROM<double>(ADDRESS_TOOL_OFFSET_Y1 + 16 * i);

        if (isnan(toolOffset[i].X) || isnan(toolOffset[i].Y))
            setToolOffset(i, 0.0F, 0.0F);
    }
}

const double* ParamsEEPROM::getOffsetX() const
{
    if (isnan(offsetX))
        return nullptr;

    return &offsetX;
}

const double* ParamsEEPROM::getOffsetY() const
{
    if (isnan(offsetY))
        return nullptr;

    return &offsetY;
}

const double* ParamsEEPROM::getOffsetZ() const
{
    if (isnan(offsetZ))
        return nullptr;

    return &offsetZ;
}

const double* ParamsEEPROM::getRotation() const
{
    if (isnan(rotation))
        return nullptr;

    return &rotation;
}

const double *ParamsEEPROM::getScaleX() const
{
    if (isnan(scaleX))
        return nullptr;

    return &scaleX;
}

const double *ParamsEEPROM::getScaleY() const
{
    if (isnan(scaleY))
        return nullptr;

    return &scaleY;
}

const double* ParamsEEPROM::getHomeX() const
{
    if (isnan(homeX))
        return nullptr;

    return &homeX;
}

const double* ParamsEEPROM::getHomeY() const
{
    if (isnan(homeY))
        return nullptr;

    return &homeY;
}

const uint16_t* ParamsEEPROM::getSerialNumber() const
{
    return &serialNumber;
}

const MagazineSlot* ParamsEEPROM::getMagazineSlot(uint8_t slotNum) const
{
    if ((slotNum >= SLOTS) || (slotNum < 0))
        return nullptr;

    MagazineSlot tmp = magazineSlots[slotNum];
    if ((isnan(tmp.X)) || (isnan(tmp.Y)) || (isnan(tmp.Z)))
        return nullptr;

    return &magazineSlots[slotNum];
}

const ValuesXY *ParamsEEPROM::getToolOffset(uint8_t toolNum) const
{
    if ((toolNum >= SLOTS) || (toolNum < 0))
        return nullptr;

    ValuesXY tmp = toolOffset[toolNum];
    if ((isnan(tmp.X)) || (isnan(tmp.Y)))
        return nullptr;

    return &toolOffset[toolNum];
}

void ParamsEEPROM::setOffsetX(double value)
{
    EEPROM.put(ADDRESS_OFFSET_X, value);
    offsetX = value;
}

void ParamsEEPROM::setOffsetY(double value)
{
    EEPROM.put(ADDRESS_OFFSET_Y, value);
    offsetY = value;
}

void ParamsEEPROM::setOffsetZ(double value)
{
    EEPROM.put(ADDRESS_OFFSET_Z, value);
    offsetZ = value;
}

void ParamsEEPROM::setRotation(double value)
{
    EEPROM.put(ADDRESS_ROTATION, value);
    rotation = value;
}

void ParamsEEPROM::setScaleX(double value)
{
    EEPROM.put(ADDRESS_SCALE_X, value);
    scaleX = value;
}

void ParamsEEPROM::setScaleY(double value)
{
    EEPROM.put(ADDRESS_SCALE_Y, value);
    scaleY = value;
}

void ParamsEEPROM::setPlazParams(double offsetX, double offsetY, double offsetZ, double rotation, double scaleX, double scaleY)
{
    setOffsetX(offsetX);
    setOffsetY(offsetY);
    setOffsetZ(offsetZ);
    setRotation(rotation);
    setScaleX(scaleX);
    setScaleY(scaleY);
}

void ParamsEEPROM::setHomeX(double value)
{
    EEPROM.put(ADDRESS_HOME_X, value);
    homeX = value;
}

void ParamsEEPROM::setHomeY(double value)
{
    EEPROM.put(ADDRESS_HOME_Y, value);
    homeY = value;
}

void ParamsEEPROM::setHome(double x, double y)
{
    setHomeX(x);
    setHomeY(y);
}

void ParamsEEPROM::setSerialNumber(uint16_t number)
{
    EEPROM.put(ADDRESS_SERIAL_NUMBER, number);

    serialNumber = number;
}

void ParamsEEPROM::setMagazineSlot(uint8_t slotNum, MagazineSlot slot)
{
    if ((slotNum >= SLOTS) || (slotNum < 0))
        return;

    EEPROM.put(ADDRESS_MAGAZINE_X1 + 24 * slotNum, slot.X);
    EEPROM.put(ADDRESS_MAGAZINE_Y1 + 24 * slotNum, slot.Y);
    EEPROM.put(ADDRESS_MAGAZINE_Z1 + 24 * slotNum, slot.Z);

    magazineSlots[slotNum] = slot;
}

void ParamsEEPROM::setMagazineSlot(uint8_t slotNum, double x, double y, double z)
{
    MagazineSlot data = {x, y, z};
    setMagazineSlot(slotNum, data);
}

void ParamsEEPROM::setToolOffset(uint8_t toolNum, ValuesXY tool)
{
    if ((toolNum >= SLOTS) || (toolNum < 0))
        return;

    EEPROM.put(ADDRESS_TOOL_OFFSET_X1 + 16 * toolNum, tool.X);
    EEPROM.put(ADDRESS_TOOL_OFFSET_Y1 + 16 * toolNum, tool.Y);

    toolOffset[toolNum] = tool;
}

void ParamsEEPROM::setToolOffset(uint8_t toolNum, double x, double y)
{
    ValuesXY data = {x, y};
    setToolOffset(toolNum, data);
}
