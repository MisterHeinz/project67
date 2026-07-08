#pragma once

enum struct ParamsStatus
{
	OK,
	INCORRECT_COMMAND,
	INVALID_PARAMS,
};

struct MoveParams
{
	ParamsStatus status;
	double units[4];
	double speed;
	double acceleration;
	double deceleration;
};

struct ValuesXY
{
    double X;
    double Y;
};

struct MagazineSlot
{
    double X;
    double Y;
    double Z;
};

enum struct StatusMain
{
	AUTO,
	READY,
	ERROR,
	NO_CONNECTION,
	EMERGENCY
};