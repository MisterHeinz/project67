 #define DEBUG // для отладки

#include "src/USBInfo.h"
#include "src/StepDirController/src/StepDirController.h"
#include "src/ParamsEEPROM.h"
#include "src/SignalingColumn.h"
#include "src/Struct.h"
#include "src/Utils.h"
// #include <Encoder.h>

const String HARDWARE_MODEL = "САРЖ-5Э";
const String FIRMWARE_VERSION = "2.0 stand";

const String RESULT_SUCCESS = "OK";
const String RESULT_ERROR_INVALID_PARAMS = "IP";
const String RESULT_ERROR_OUT_OF_LIMITS = "OL";
const String RESULT_ERROR_NO_DATA = "ND";
const String RESULT_ERROR_UNSUPPORTED_COMMAND = "UC";
const String RESULT_ERROR_INCORRECT_COMMAND = "IC";
const String RESULT_ERROR_NEED_CALIBRATION = "NC";
const String RESULT_ERROR_CALIBRATION_SENSOR = "CS";
const String RESULT_ERROR_LOW_AIR_PRESSURE = "AP";
const String RESULT_ERROR_OCCUPIED = "OC";
const String RESULT_ERROR_EMPTY = "EM";
const String RESULT_ERROR_MOVING_IN_PROCESS = "MP";
const String RESULT_ERROR_STOPED = "ST";
const String RESULT_ERROR_EMERGENCY_STOPED = "SE";
const String RESULT_ERROR_UNKNOWN = "EU";

const String COMMAND_HEARTBEAT = "HB";
const String COMMAND_HARDWARE_MODEL = "HM";
const String COMMAND_FIRMWARE_VERSION = "FV";
const String COMMAND_MOVE_HOME = "MH";
const String COMMAND_HOME_READ = "HR";
const String COMMAND_HOME_WRITE = "HW";
const String COMMAND_AREA_SIZE = "WA";
const String COMMAND_CALIBRATE_ALL = "CA";
const String COMMAND_CALIBRATION_READ = "CR";
const String COMMAND_PLAZ_PARAMS_READ = "PR";
const String COMMAND_PLAZ_PARAMS_WRITE = "PW";
const String COMMAND_PRESSURE_STATUS = "PS";
const String COMMAND_MAGAZINE_STATUS = "MS";
const String COMMAND_MAGAZINE_SLOT_COORDINATES_READ = "SR";
const String COMMAND_MAGAZINE_SLOT_COORDINATES_WRITE = "SW";
const String COMMAND_TOOL_NUMBER = "TN";
const String COMMAND_TOOL_IS_INSTALLED = "TI";
const String COMMAND_TOOL_IS_CLAMPED = "TC";
const String COMMAND_TOOL_CLAMP = "TF";
const String COMMAND_TOOL_OFFSET_READ = "TR";
const String COMMAND_TOOL_OFFSET_WRITE = "TW";
const String COMMAND_CHANGE_TOOL = "CT";
const String COMMAND_CURRENT_POSITION_GLOBAL = "CG";
const String COMMAND_CURRENT_POSITION_PLAZ = "CP";
const String COMMAND_EMERGENCY_STATUS = "ES";
const String COMMAND_CABLE_CUT = "CC";
const String COMMAND_MOVE_RELATIVE = "RM"; 							// движение по координатам плаза
const String COMMAND_MOVE_ABSOLUTE = "AM"; 							// движение по координатам плаза
const String COMMAND_MOVE_RELATIVE_SERVICE = "RS";					// движение по координатам станка
const String COMMAND_MOVE_ABSOLUTE_SERVICE = "AS";					// движение по координатам станка
// const String COMMAND_CALCULATE_DURATION_RELATIVE = "DR";
// const String COMMAND_CALCULATE_DURATION_ABSOLUTE = "DA";
const String COMMAND_BUZZER_DISABLE = "BD";
const String COMMAND_EMERGENCY_STOP = "SE";
const String COMMAND_STOP = "ST";
const String COMMAND_MODE_AUTO = "MA";
const String COMMAND_SET_SERIAL_NUMBER = "SS";

// пределы податчика
// const double MAX_SPEED = 425.0F;			// мм/сек; 		максимально допустимая скорость станка; 	значение по умолчанию 50, снижено до 47.5 из-за резонанса иглы
// const double MAX_ACCELERATION = 1900.0F; // мм/сек^2; 	максимально допустимое ускорение; 			значение по умолчанию 200

const double MAX_SPEED = 30.0F;			// мм/сек; 		максимально допустимая скорость станка; 	значение по умолчанию 50, снижено до 47.5 из-за резонанса иглы
const double MAX_ACCELERATION = 800.0F; // мм/сек^2; 	максимально допустимое ускорение; 			значение по умолчанию 200
// const double JERK = 0.375F;				// мм/сек; 		максимально допустимый рывок; 				значение по умолчанию 0.375
const double JERK = 0.5F;	

const bool ROOT_STATE_CUTTER = true;
const bool ROOT_STATE_CLAMPER = false;

const int PIN_CUTTER = 27;
const int PIN_CLAMPER = 28;
const int PIN_LIMIT_SWITCH[3] = {13, 14, 15}; // X, Y, Z
const int PIN_EMERGENCY = 41;
const int PIN_MAGAZINE[SLOTS] = {33, 34, 35, 36, 37}; // Слоты 1, 2, 3, 4, 5
const int PIN_TOOL_INSTALLED = 39;
const int PIN_PRESSURE = 40;

const uint8_t LEVEL_EMERGENCY = LOW; // кнопка аварийной остановки нажата

const uint8_t LEVEL_LIMIT_SWITCH[3] = {LOW, LOW, LOW}; // X, Y, Z
const uint8_t MODE_LIMIT_SWITCH[3] = {FALLING, FALLING, FALLING}; // X, Y, Z

const uint8_t LEVEL_MAGAZINE = LOW; // слот занят
const uint8_t LEVEL_TOOL_INSTALLED = HIGH; // головка установлена на станок
const uint8_t LEVEL_PRESSURE = HIGH; // давление в норме

// const uint8_t CALIBRATED = 7; // откалибровано 0b00000111 -  Z, Y, X (было при калибровке 3 осей)
const uint8_t CALIBRATED = 3; // откалибровано 0b00000011 - Y, X (калибровка оси Z отключена: используются только X и Y)

// направление движения к концевикам: 1 - в положительную; 0 - в отрицательную
const uint8_t CALIBRATION_DIR = 0b00000110; // Z, Y, X (бит Z больше не используется, т.к. калибровка оси Z отключена)
const double CALIBRATION_FINE_SPEED = 2.5;
const double CALIBRATION_OVER_MOVE = 0.1f; // дополнительное движение к датчикам для защиты от дребезга
const double CALIBRATION_FINE_OFFSET = 20.0F;
const double Z_LIMIT_CUT = 110.0F; // mm
const double Z_OFFSET_CHANGE_TOOL = 33.0F; // mm
const double X_LIMIT_MAGAZINE = 1170.0F; // mm 1147.0F
const double X_TOOL_SIZE = 100.0F; // mm

const uint32_t HEARTBEAT_INTERVAL = 1000; // интервал генератора сердцебиения в миллисекундах

const uint8_t MIN_SERIAL_AVIABLE_TIME = 10; // минимальное время через которое можно начинать взаимодействие с последовательным портом

volatile StatusMain statusMain;

volatile bool emergenсyStatus = false;
bool isConnectedToPC_old = false;
volatile bool pressureStatus = false;
volatile bool toolInstalled = false;

double calibrationRoughSpeed; // скорость грубой калибровки

int currentToolNum = -1; // текущий номер инструмента; -1 - не выбран
int targetToolNum = -1; // целевой номер инструмента, на который нужно поменять текущий (используется в процессе смены)

// volatile uint8_t calibration =7; // откалибровано 7 или 0b00000111 -  Z, Y, X отключена колибровка так надо
volatile uint8_t calibration = 3; // откалибровано 3 или 0b00000011 - Y, X (ось Z исключена из процесса калибровки)

bool isClamped = true;
bool isStoped = false;

String inData = "";			 // хранилище данных с последовательного порта
std::vector<String> outData; // очередь сообщений на отправку

elapsedMillis heartbeatWait;
elapsedMillis timeFromLastHeartbeatConfirm;
elapsedMillis serialAviableTime;

ParamsEEPROM storageParams;
MoveParams goHomeParams;

SignalingColumn signalingColumn(31, 32, 30, 29); // пины: красный, желтый, зеленый, звук

//TODO: внедрить данные с энкодеров в логику прошивки
// Encoder encoderX1(4, 5);	// pinA 4, pinB 5
// Encoder encoderX2(6, 7);	// pinA 6, pinB 7
// Encoder encoderY(8, 9);		// pinA 8, pinB 9
// Encoder encoderZ(10, 11);	// pinA 10, pinB 11

Axis X(16, 18,22);	 	// stepPin, dirPin
Axis Y(17, 0,1);	 	// stepPin, dirPin
Axis Z(5, 6);	 	// stepPin, dirPin
Axis A(7, 8, 9); // stepPin, dirPin, enablePin

MoveController mainController(2); // для движения и калибровки X
MoveController aux1Controller(2);  // для калибровки Y
MoveController aux2Controller(2);  // для калибровки Z

void setup()
{
	X.enable();
	Y.enable();
	const uint16_t* serialNumber = storageParams.getSerialNumber();
	initSerial(*serialNumber);

	inData.reserve(128);
	outData.reserve(128);
	for (auto& s : outData) {
    	s.reserve(128);
	}

	// тестирование световой колонны
	signalingColumn.setColor(SignalingColumn::RED);
	delay(500);
	signalingColumn.setColor(SignalingColumn::GREEN);
	delay(500);
	signalingColumn.setColor(SignalingColumn::YELLOW);
	signalingColumn.testBuzzer();

	pinMode(PIN_CUTTER, OUTPUT);
	pinMode(PIN_CLAMPER, OUTPUT);

	pinMode(PIN_EMERGENCY, INPUT);
	pinMode(PIN_PRESSURE, INPUT);
	pinMode(PIN_TOOL_INSTALLED, INPUT);

	for (uint8_t i = 0; i < 3; i++){
		pinMode(PIN_LIMIT_SWITCH[i], INPUT);
	}

	for (uint8_t i = 0; i < SLOTS; i++){
		pinMode(PIN_MAGAZINE[i], INPUT);
	}

	X.setStepsPerRevolution(1000)
		.setUnitsPerRevolution(8.0F)   // можно уточнить до 6.676, если хотите
  	.setInverseDirPin(false)
  	.enableLimits(-100000.0F, 100000.0F);

	Y.setStepsPerRevolution(1000)
		.setUnitsPerRevolution(7.8F)   // можно уточнить до 6.676, если хотите
  	.setInverseDirPin(true)
  	.enableLimits(-100000.0F, 100000.0F);

	Z.setStepsPerRevolution(400)
		.setUnitsPerRevolution(4.0F)
		.setInverseDirPin(true)
		.enableLimits(0.0F, 152.5); // 0...152.5 mm

	A.setStepsPerRevolution(1000) // 800 
		.setUnitsPerRevolution(40.376F)
		.setInverseDirPin(false)
		.setInverseEnablePin(true);

	setToolClamped(true);

	// isToolInstalled() ? A.enable() : A.disable();
	A.disable();

	pressureStatus = (digitalReadFast(PIN_PRESSURE) == LEVEL_PRESSURE);

	trySetStatus(StatusMain::READY);

	Serial.begin(115200);
	// Serial5.transmitterEnable(19);

	heartbeatWait = HEARTBEAT_INTERVAL;

	attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY), emergencyInterrupt, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(PIN_PRESSURE), []{pressureStatusCheck(true);}, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(PIN_TOOL_INSTALLED), []{isToolInstalledCheck(true);}, CHANGE);
	for (int i = 0; i < SLOTS; i++)
	{
		attachInterrupt(digitalPinToInterrupt(PIN_MAGAZINE[i]), magazineStatusCheck, CHANGE);
	}
	emergencyInterrupt();
}

void loop()
{
	if (receiveCommand())
		handleCommand();

	//heartbeat();
	sendData();
	signalingColumn.tick();
}

void addDataToOutQueue(String& data)
{
	noInterrupts();
	outData.push_back(data);
	interrupts();
}

bool isConnectedToPC()
{
	if (!Serial)
		return false;

	//#ifndef DEBUG
		//if (timeFromLastHeartbeatConfirm > HEARTBEAT_INTERVAL * 2)
			//return false;
	//#endif

	return true;
}

bool receiveCommand()
{
	char received = 0x00;

	noInterrupts();
	StatusMain status = statusMain;
	interrupts()

	if (!isConnectedToPC() && isConnectedToPC_old) {
		Serial.clear(); // очистка полученных, но не прочитанных данных
		inData = ""; // очистка прочитанных данных
		isConnectedToPC_old = false;
		trySetStatus(StatusMain::NO_CONNECTION);
	} else if (status == StatusMain::NO_CONNECTION)
		trySetStatus(StatusMain::READY);

	if (isConnectedToPC())
		isConnectedToPC_old = true;
		
	if (!Serial.available()) // TODO: Протестировать на наличие глюков со связью
		serialAviableTime = 0;	

	if (serialAviableTime >= MIN_SERIAL_AVIABLE_TIME)
	{
		received = Serial.read();
		if (received > 0)
			inData += received;
	}

	return received == '\n';
}

void handleCommand()
{
	inData.replace(' ', "");
	inData.replace('\n', "");
	inData.replace('\r', "");

	if (inData.length() < 2)
	{
		inData = "";
		return;
	}

	String function = inData.substring(0, 2);

	tryResetErrorStatus(function); // попытка сброса статуса ошибки при незначительных ошибках, если они есть 

	// Обработка запроса модели станка
	if (function == COMMAND_HARDWARE_MODEL)
	{
		String answer = COMMAND_HARDWARE_MODEL + RESULT_SUCCESS + HARDWARE_MODEL;
		addDataToOutQueue(answer);
	}

	// Обработка запроса версии прошивки
	else if (function == COMMAND_FIRMWARE_VERSION)
	{
		String answer = COMMAND_FIRMWARE_VERSION + RESULT_SUCCESS + FIRMWARE_VERSION;
		addDataToOutQueue(answer);
	}

	// Обработка подтверждения сердцебиения
	//else if (function == COMMAND_HEARTBEAT)
		//handleHeartbeatConfirm(inData);

	// Обработка запроса движения в домашнее положение
	else if (function == COMMAND_MOVE_HOME)
		handleMoveToHome(inData);

	// Обработка запроса чтения домашнего положения
	else if (function == COMMAND_HOME_READ)
	{
		String answer = COMMAND_HOME_READ + readHomePosition();
		addDataToOutQueue(answer);
	}

	// Обработка запроса записи домашнего положения
	else if (function == COMMAND_HOME_WRITE)
	{
		String answer = COMMAND_HOME_WRITE + writeHomePosition(inData);
		addDataToOutQueue(answer);
	}

	// Обработка запроса движения по относительным координатам
	else if (function == COMMAND_MOVE_RELATIVE)
		handleMoveRelative(inData);

	// Обработка запроса движения по абсолютным координатам
	else if (function == COMMAND_MOVE_ABSOLUTE)
		handleMoveAbsolute(inData);

	// Обработка запроса движения по относительным координатам
	else if (function == COMMAND_MOVE_RELATIVE_SERVICE)
		handleMoveRelativeService(inData);

	// Обработка запроса движения по абсолютным координатам
	else if (function == COMMAND_MOVE_ABSOLUTE_SERVICE)
		handleMoveAbsoluteService(inData);

	// Обработка запроса калибровки (поиск точек отсчета по концевикам)
	else if (function == COMMAND_CALIBRATE_ALL)
		// addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_SUCCESS + "111");
		handleCalibrateAll(inData);

	// Обработка запроса статуса калибровки
	else if (function == COMMAND_CALIBRATION_READ)
	{
		String answer = COMMAND_CALIBRATION_READ + RESULT_SUCCESS;
		answer += statusToString(calibration, 3);
		addDataToOutQueue(answer);
	}

	// Обработка запроса текущей позиции в координатах плаза
	else if (function == COMMAND_CURRENT_POSITION_GLOBAL)
	{
		String value = currentPositionGlobal();
		addDataToOutQueue(COMMAND_CURRENT_POSITION_GLOBAL + value);
	}

	// Обработка запроса текущей позиции в координатах плаза
	else if (function == COMMAND_CURRENT_POSITION_PLAZ)
	{
		String value = currentPositionPlaz();
		addDataToOutQueue(COMMAND_CURRENT_POSITION_PLAZ + value);
	}

	// Обработка запроса размера рабочей области
	else if (function == COMMAND_AREA_SIZE)
	{
		String value = workAreaSize();
		addDataToOutQueue(COMMAND_AREA_SIZE + RESULT_SUCCESS + value);
	}

	// Обработка запроса номера текущей головки
	else if (function == COMMAND_TOOL_NUMBER)
	{
		String answer = COMMAND_TOOL_NUMBER + RESULT_SUCCESS + String(currentToolNum + 1);
		addDataToOutQueue(answer);
	}

	// Обработка запроса наличия установленной головки
	else if (function == COMMAND_TOOL_IS_INSTALLED)
		isToolInstalledCheck(false);

	// Обработка запроса статуса фиксации податчика
	else if (function == COMMAND_TOOL_IS_CLAMPED)
	{
		String answer = COMMAND_TOOL_IS_CLAMPED + RESULT_SUCCESS + String((int)isClamped);
		addDataToOutQueue(answer);
	}

	// Обработка запроса управления фиксацией податчика
	else if (function == COMMAND_TOOL_CLAMP)
		handleClampTool(inData);

	// Обработка запроса чтения поправки раскрадочной головки
	else if (function == COMMAND_TOOL_OFFSET_READ)
		handleReadToolOffset(inData);

	// Обработка запроса записи поправки раскладочной головки
	else if (function == COMMAND_TOOL_OFFSET_WRITE)
		handleWriteToolOffset(inData);

	// Обработка запроса смены податчика
	else if (function == COMMAND_CHANGE_TOOL)
		handleChangeTool(inData);

	// Обработка запроса обрезки провода
	else if (function == COMMAND_CABLE_CUT)
	{
		String answer = COMMAND_CABLE_CUT + cableCut();
		addDataToOutQueue(answer);
	}

	// Обработка запроса чтения параметров плаза
	else if (function == COMMAND_PLAZ_PARAMS_READ)
	{
		String answer = COMMAND_PLAZ_PARAMS_READ + readPlazParams();
		addDataToOutQueue(answer);
	}

	// Обработка запроса записи параметров плаза
	else if (function == COMMAND_PLAZ_PARAMS_WRITE)
	{
		String answer = COMMAND_PLAZ_PARAMS_WRITE + writePlazParams(inData);
		addDataToOutQueue(answer);
	}

	// Обработка запроса состояния давления пневматической линии
	else if (function == COMMAND_PRESSURE_STATUS)
		pressureStatusCheck(false);

	// Обработка запроса статуса магазина
	else if (function == COMMAND_MAGAZINE_STATUS)
		magazineStatusCheck();

	// Обработка запроса чтения координат слота магазина
	else if (function == COMMAND_MAGAZINE_SLOT_COORDINATES_READ)
		handleReadMagazineSlotCoordinates(inData);

	// Обработка запроса записи координат слота магазина
	else if (function == COMMAND_MAGAZINE_SLOT_COORDINATES_WRITE)
		handleWriteMagazineSlotCoordinates(inData);

	// Обработка запроса состояния кнопки экстренной остановки
	else if (function == COMMAND_EMERGENCY_STATUS)
		handleEmergencyStatus();

	// Обработка запроса отключения звукового оповещения
	else if (function == COMMAND_BUZZER_DISABLE)
	{
		signalingColumn.disableBuzzer();
		addDataToOutQueue(COMMAND_BUZZER_DISABLE + RESULT_SUCCESS);
	}

	// Обработка запроса экстренной остановки
	else if (function == COMMAND_EMERGENCY_STOP)
	{
		emergency();
		addDataToOutQueue(COMMAND_EMERGENCY_STOP + RESULT_SUCCESS);
	}

	// Обработка запроса досрочной остановки
	else if (function == COMMAND_STOP)
		handleStop();

	// Обработка запроса включения/выключения автоматического режима
	else if (function == COMMAND_MODE_AUTO)
		handleModeAuto(inData);

	// Обработка запроса настройки серийного номера устройства
	else if ((function == COMMAND_SET_SERIAL_NUMBER) && (*(storageParams.getSerialNumber()) == 0xFFFF)) // если серийный номер уже настроен, не признаемся в существовании команды для его настройки
		handleSetSerialNumber(inData);

	// Обработка запроса неподдерживаемой команды
	else
	{
		String answer = function + RESULT_ERROR_UNSUPPORTED_COMMAND;
		addDataToOutQueue(answer);
	}
	inData = "";
}

//void heartbeat(){
	//if (heartbeatWait < HEARTBEAT_INTERVAL)
		//return;
	
	//heartbeatWait -= HEARTBEAT_INTERVAL;
	//String data = COMMAND_HEARTBEAT + RESULT_SUCCESS + String(HEARTBEAT_INTERVAL);
	//addDataToOutQueue(data);
//}

//void handleHeartbeatConfirm(String command)
//{
	//if (command.length() != 4) // если некорректная команда
		//return;

	//if (command.substring(2) != RESULT_SUCCESS) // если некоректный ответ
		//return;

	// if (timeFromLastHeartbeatConfirm < HEARTBEAT_INTERVAL * 0.5F) // если корректный ответ раньше времени
	// {
	// 	String data = COMMAND_HEARTBEAT + RESULT_ERROR_UNKNOWN;
	// 	addDataToOutQueue(data);
	// 	return;
	// }

	//timeFromLastHeartbeatConfirm = 0;
//}

void isToolInstalledCheck(bool isInterrupt)
{
	bool installed = isToolInstalled();
	if (isInterrupt && (toolInstalled == installed))
		return;

	toolInstalled = installed;

	// TODO: добавить enable/disable оси A?
	// TODO: выдавать ошибку, если станок в движениии?

	String result = COMMAND_TOOL_IS_INSTALLED + RESULT_SUCCESS;
	// result += String((int)toolInstalled);
	result += String((int)false);
	addDataToOutQueue(result);
}

void pressureStatusCheck(bool isInterrupt)
{
	int status = true;
	// int status = (digitalReadFast(PIN_PRESSURE) == LEVEL_PRESSURE);
	
	if (isInterrupt && (pressureStatus == status))
		return;

	if (isInterrupt)
		status ? trySetStatus(StatusMain::READY) : trySetStatus(StatusMain::ERROR);

		status = true;
	
	pressureStatus = status;
	String answer = COMMAND_PRESSURE_STATUS + RESULT_SUCCESS + String(pressureStatus);
	addDataToOutQueue(answer);
}

void magazineStatusCheck()
{
	uint8_t status = checkMagazineStatus();
	String answer = COMMAND_MAGAZINE_STATUS + RESULT_SUCCESS + statusToString(status, SLOTS);
	addDataToOutQueue(answer);
}

void emergencyInterrupt()
{
	noInterrupts();
	bool status = isEmergency();

	if (emergenсyStatus == status) // TODO: дребезг контактов, возможно нужно удалить
	{
		interrupts();
		return;
	}
	
	emergenсyStatus = status;

	if (status)
		emergency();
	else 
		trySetStatus(StatusMain::READY);

	interrupts();

	addDataToOutQueue(COMMAND_EMERGENCY_STATUS + RESULT_SUCCESS + String((int)status)); // TODO: ставить в очередь первым/вторым?
}

void emergency()
{
	mainController.emergencyStop();
	aux1Controller.emergencyStop();
	aux2Controller.emergencyStop();
	calibration = 0;

	trySetStatus(StatusMain::EMERGENCY);
}

uint8_t checkMagazineStatus()
{
	uint8_t magazineStatus = 0;
	for (int i = 0; i < SLOTS; i++){
		uint8_t result = (digitalReadFast(PIN_MAGAZINE[i]) == LEVEL_MAGAZINE);
		// magazineStatus &= ~(0x01 << i); // сбросить бит
		magazineStatus |= result << i; // установить бит, если занят
	}
	return magazineStatus;
}

String statusToString(uint8_t status, uint8_t size)
{
	String answer = "";
	for (int i = 0; i < size; i++)
	{
		answer += (status & (1 << i)) ? "1" : "0";
	}
	return answer;
}

void sendData()
{
	//noInterrupts();
	if (!isConnectedToPC())
	{
		noInterrupts();
		outData.clear();
		interrupts();
	}

	if (outData.size() == 0)
	{
		// interrupts();
		return;
	}

	noInterrupts();
	String data = outData.front();
	outData.erase(outData.begin());
	interrupts();

	Serial.println(data);
}

bool isCalibrated()
{
	return calibration == CALIBRATED; // Z, Y, X
	
}

MoveParams stringToMoveParams(String command)
{
	MoveParams params;

	int X_Index = command.indexOf("OX");
	int Y_Index = command.indexOf("OY");
	int Z_Index = command.indexOf("OZ");
	int A_Index = command.indexOf("OA");
	int speed_Index = command.indexOf("SP");
	int ACC_Index = command.indexOf("AC");
	int DCC_Index = command.indexOf("DC");

	if ((X_Index == -1) || (Y_Index == -1) || (Z_Index == -1) || (A_Index == -1) || (speed_Index == -1) || (ACC_Index == -1) || (DCC_Index == -1))
	{
		params.status = ParamsStatus::INCORRECT_COMMAND;
		return params;
	}

	params.units[0] = command.substring(X_Index + 2, Y_Index).toFloat();
	params.units[1] = command.substring(Y_Index + 2, Z_Index).toFloat();
	params.units[2] = command.substring(Z_Index + 2, A_Index).toFloat();
	params.units[3] = command.substring(A_Index + 2, speed_Index).toFloat();

	params.speed = command.substring(speed_Index + 2, ACC_Index).toFloat();
	params.acceleration = command.substring(ACC_Index + 2, DCC_Index).toFloat();
	params.deceleration = command.substring(DCC_Index + 2).toFloat();

	if ((params.speed <= 0.0F) || (params.acceleration <= 0.0F) || (params.deceleration <= 0.0F))
	{
		params.status = ParamsStatus::INVALID_PARAMS;
		return params;
	}

	double minSpeed = JERK / (double)MAX_SPEED * 100.0F;
	if (params.speed < minSpeed)
		params.speed = minSpeed;

	if (params.acceleration < 1.0F)
		params.acceleration = 1.0F;

	if (params.deceleration < 1.0F)
		params.deceleration = 1.0F;

	if (params.speed > 100.0F)
		params.speed = 100.0F;
	
	if (params.acceleration > 100.0F)
		params.acceleration = 100.0F;
	
	if (params.deceleration > 100.0F)
		params.deceleration = 100.0F;

	params.speed = MAX_SPEED * params.speed / 100.0F;
	params.acceleration = MAX_ACCELERATION * params.acceleration / 100.0F;
	params.deceleration = MAX_ACCELERATION * params.deceleration / 100.0 * -1.0F;

	params.status = ParamsStatus::OK;
	return params;
}

void handleStop()
{
	isStoped = true;
	sendIfStopAnswer();
	mainController.stopAsync();
	aux1Controller.stopAsync();
	aux2Controller.stopAsync();
	trySetStatus(StatusMain::READY);	
}

void handleEmergencyStatus()
{
	String answer = COMMAND_EMERGENCY_STATUS;
	answer += RESULT_SUCCESS;
	answer += String((int)isEmergency());

	addDataToOutQueue(answer);
}

void handleModeAuto(String command)
{
	String answer = COMMAND_MODE_AUTO;

	int modeIndex = command.indexOf(COMMAND_MODE_AUTO);

	bool enableAuto = command.substring(modeIndex + 2).toInt() > 0;
	StatusMain status = enableAuto ? StatusMain::AUTO : StatusMain::READY;
	StatusMain statusNew = trySetStatus(status);

	if (status == statusNew)
		answer += RESULT_SUCCESS;
	else if (statusNew == StatusMain::EMERGENCY)
		answer += RESULT_ERROR_EMERGENCY_STOPED;
	else
		answer += RESULT_ERROR_UNKNOWN;
	
	answer += statusNew == StatusMain::AUTO ? "1" : "0";
	addDataToOutQueue(answer);
}

void handleCalibrateAll(String command)
{
	String answer = COMMAND_CALIBRATE_ALL;

	if (isEmergency())
	{
		answer += RESULT_ERROR_EMERGENCY_STOPED;
		addDataToOutQueue(answer);
		return;
	}

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
	{
		answer += RESULT_ERROR_MOVING_IN_PROCESS;
		addDataToOutQueue(answer);
		return;
	}

	int speedIndex = command.indexOf("SP");
	if (speedIndex == -1)
	{
		answer += RESULT_ERROR_INCORRECT_COMMAND;
		addDataToOutQueue(answer);
		return;
	}

	double speed = command.substring(speedIndex + 2).toFloat();
	if (speed < 0){
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_INVALID_PARAMS);
		return;
	}
	
	if (speed < 1.0F)
		speed = 1.0F;
	
	if (speed > 30.0F)
		speed = 30.0F;

	calibrationRoughSpeed = MAX_SPEED * speed / 100.0F;
	calibration = 0;

	// calibratePhase1(); // грубая калибровка оси Z отключена (в нашей задаче нет оси Z, только X и Y)
	calibratePhase2(); // начинаем сразу с грубой калибровки оси X
}

/* --- Калибровка оси Z отключена: в нашей задаче используются только 2 двигателя (оси X и Y) ---
void calibratePhase1() // грубая калибровка оси Z
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	if (digitalReadFast(PIN_LIMIT_SWITCH[2]) == LEVEL_LIMIT_SWITCH[2]) // ось уже поднята к датчику, переходим к следующему этапу
	{
		calibratePhase2();
		return;
	}
	
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[2]), []{aux2Controller.stopAsync();}, MODE_LIMIT_SWITCH[2]);
	prepareMoveToLimit(&Z, 0b00000100);
	aux2Controller.setJerkSpeedUnits(JERK);
	aux2Controller.setRegularSpeedUnits(calibrationRoughSpeed);
	aux2Controller.setAccelerationUnits(MAX_ACCELERATION);
	aux2Controller.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	aux2Controller.setOnMoveFinished(calibratePhase1_add);
	aux2Controller.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	aux2Controller.moveAsync(Z);
}

void calibratePhase1_add() // защита от дребезга, двигает Z на 0,1 мм ближе к датчику
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	detachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[2]));
	MoveParams params;
	
	for (int i = 0; i < 4; i++){
		params.units[i] = 0.0F;
	}
	params.units[2] = CALIBRATION_OVER_MOVE;
	params.units[2] *= (CALIBRATION_DIR & 0b00000100) ? 1.0F : -1.0F;
	params.speed = MAX_SPEED * 0.01f; // скорость 1% 
	params.acceleration = MAX_ACCELERATION * 0.5;
	params.deceleration = MAX_ACCELERATION * -0.5F;
	params.status = ParamsStatus::OK;

	setPositionRelative(params);
	mainController.setOnMoveFinished(calibratePhase2);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	moveMotors(params);
}
--- конец отключенного блока калибровки оси Z --- */

void calibratePhase2() // грубая калибровка оси X
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	// проверка грубой калибровки оси Z отключена (ось Z не используется)
	// if (digitalReadFast(PIN_LIMIT_SWITCH[2]) != LEVEL_LIMIT_SWITCH[2]) // не выполнена грубая калибровка Z
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_CALIBRATION_SENSOR); 
	// 	return;
	// }

	postCalibration(&Y, 0b00000010);
	if (digitalReadFast(PIN_LIMIT_SWITCH[0]) == LEVEL_LIMIT_SWITCH[0]) // если ось X около датчика, попускаем этап
	{
		calibratePhase3();
		return;
	}

	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[0]), []{mainController.stopAsync();}, MODE_LIMIT_SWITCH[0]);
	prepareMoveToLimit(&X, 0b00000001);
	mainController.setJerkSpeedUnits(JERK);
	mainController.setRegularSpeedUnits(calibrationRoughSpeed);
	mainController.setAccelerationUnits(MAX_ACCELERATION);
	mainController.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	mainController.setOnMoveFinished(calibratePhase2_add);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	mainController.moveAsync(X);
}

void calibratePhase2_add() // защита от дребезга, двигает X на 0,1 мм ближе к датчику
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	detachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[0]));
	MoveParams params;

	for (int i = 0; i < 4; i++){
		params.units[i] = 0.0F;
	}

	params.units[0] = CALIBRATION_OVER_MOVE;
	params.units[0] *= (CALIBRATION_DIR & 0b00000001) ? 1.0F : -1.0F;

	params.speed = MAX_SPEED * 0.01f; // скорость 1% 
	params.acceleration = MAX_ACCELERATION * 0.5;
	params.deceleration = MAX_ACCELERATION * -0.5F;
	params.status = ParamsStatus::OK;

	setPositionRelative(params);
	mainController.setOnMoveFinished(calibratePhase3);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	moveMotors(params);
}

void calibratePhase3() // грубая калибровка оси Y
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	if (digitalReadFast(PIN_LIMIT_SWITCH[0]) != LEVEL_LIMIT_SWITCH[0]) // не выполнена грубая калибровка X
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_CALIBRATION_SENSOR); 
		return;
	}

	// postCalibration(&Z, 0b00000100); // финализация оси Z отключена (ось Z не используется)
	if (digitalReadFast(PIN_LIMIT_SWITCH[1]) == LEVEL_LIMIT_SWITCH[1]) // если ось Y около датчика, попускаем этап
	{
		calibratePhase4();
		return;
	}

	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[1]), []{aux1Controller.stopAsync();}, MODE_LIMIT_SWITCH[1]);
	prepareMoveToLimit(&Y, 0b00000010);
	aux1Controller.setJerkSpeedUnits(JERK);
	aux1Controller.setRegularSpeedUnits(calibrationRoughSpeed);
	aux1Controller.setAccelerationUnits(MAX_ACCELERATION);
	aux1Controller.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	aux1Controller.setOnMoveFinished(calibratePhase3_add);
	aux1Controller.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	aux1Controller.moveAsync(Y);
}

void calibratePhase3_add() // защита от дребезга, двигает Y на 0,1 мм ближе к датчику
{
	aux1Controller.setOnEmergensyStoped(nullptr);
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	detachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[1]));
	MoveParams params;

	for (int i = 0; i < 4; i++){
		params.units[i] = 0.0F;
	}

	params.units[1] = CALIBRATION_OVER_MOVE;
	params.units[1] *= (CALIBRATION_DIR & 0b00000010) ? 1.0F : -1.0F;

	params.speed = MAX_SPEED * 0.01f; // скорость 1% 
	params.acceleration = MAX_ACCELERATION * 0.5;
	params.deceleration = MAX_ACCELERATION * -0.5F;
	params.status = ParamsStatus::OK;

	setPositionRelative(params);
	mainController.setOnMoveFinished(calibratePhase4);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	moveMotors(params);
}

void calibratePhase4() // отъехать от концевиков для точной калибровки
{
	mainController.setOnEmergensyStoped(nullptr);
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	if ((digitalReadFast(PIN_LIMIT_SWITCH[1]) != LEVEL_LIMIT_SWITCH[1])) // не выполнена грубая калибровка Y
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_CALIBRATION_SENSOR);
		return;
	}

	postCalibration(&X, 0b00000001);

	MoveParams params;
	
	// цикл ограничен осями X и Y (i < 2); ось Z в смещении для точной калибровки не участвует
	for (int i = 0; i < 2; i++){
		params.units[i] = CALIBRATION_FINE_OFFSET;
		params.units[i] *= (CALIBRATION_DIR & (0x01 << i)) ? -1.0F : 1.0F;
	}
	params.units[2] = 0.0F; // ось Z не двигаем (калибровка Z отключена)
	params.units[3] = 0.0F;
	params.speed = MAX_SPEED * 0.02F; // скорость 2% 
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.status = ParamsStatus::OK;

	setPositionRelative(params);
	mainController.setOnMoveFinished(calibratePhase5);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	moveMotors(params);
}

void calibratePhase5() // точная калибровка осей X, Y, Z
{
	mainController.setOnEmergensyStoped(nullptr);
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		calibrateFinished();
		return;
	}

	
	if ((digitalReadFast(PIN_LIMIT_SWITCH[0]) == LEVEL_LIMIT_SWITCH[0]) || 
		(digitalReadFast(PIN_LIMIT_SWITCH[1]) == LEVEL_LIMIT_SWITCH[1])
		// || (digitalReadFast(PIN_LIMIT_SWITCH[2]) == LEVEL_LIMIT_SWITCH[2]) // проверка датчика оси Z отключена
		) // ошибка, одна или несколько осей не двигаются / один или несколько датчиков не работают
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_CALIBRATION_SENSOR);
		return;
	}

	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[0]), []{mainController.stopAsync();}, MODE_LIMIT_SWITCH[0]);
	prepareMoveToLimit(&X, 0b00000001);
	mainController.setJerkSpeedUnits(CALIBRATION_FINE_SPEED); // движение без разгона и торможения
	mainController.setRegularSpeedUnits(CALIBRATION_FINE_SPEED);
	mainController.setAccelerationUnits(MAX_ACCELERATION);
	mainController.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	mainController.setOnMoveFinished(calibratePhase5_add);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);

	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[1]), []{aux1Controller.stopAsync();}, MODE_LIMIT_SWITCH[1]);
	prepareMoveToLimit(&Y, 0b00000010);
	aux1Controller.setJerkSpeedUnits(CALIBRATION_FINE_SPEED); // движение без разгона и торможения
	aux1Controller.setRegularSpeedUnits(CALIBRATION_FINE_SPEED);
	aux1Controller.setAccelerationUnits(MAX_ACCELERATION);
	aux1Controller.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	aux1Controller.setOnMoveFinished(calibratePhase5_add);
	aux1Controller.setOnEmergensyStoped(calibrateAllEmergensyStoped);

	/* --- точная калибровка оси Z отключена (в нашей задаче ось Z не используется) ---
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[2]), []{aux2Controller.stopAsync();}, MODE_LIMIT_SWITCH[2]);
	prepareMoveToLimit(&Z, 0b00000100);
	aux2Controller.setJerkSpeedUnits(CALIBRATION_FINE_SPEED); // движение без разгона и торможения
	aux2Controller.setRegularSpeedUnits(CALIBRATION_FINE_SPEED);
	aux2Controller.setAccelerationUnits(MAX_ACCELERATION);
	aux2Controller.setDecelerationUnits(MAX_ACCELERATION * -1.0F);
	aux2Controller.setOnMoveFinished(calibratePhase5_add);
	aux2Controller.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	--- конец отключенного блока --- */

	mainController.moveAsync(X);
	aux1Controller.moveAsync(Y);
	// aux2Controller.moveAsync(Z); // ось Z в точной калибровке не участвует
}

void calibratePhase5_add() // защита от дребезга, двигает X, Y, Z на CALIBRATION_OVER_MOVE мм ближе к датчику
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving()) // еще не все оси завершили предыдущий этап
		return;

	aux1Controller.setOnEmergensyStoped(nullptr);
	aux2Controller.setOnEmergensyStoped(nullptr);
	mainController.setOnEmergensyStoped(nullptr);

	if(isStoped)
	{
		calibrateFinished();
		return;
	}
	
	MoveParams params;

	for (uint8_t i = 0; i < 2; i++)
	{
		detachInterrupt(digitalPinToInterrupt(PIN_LIMIT_SWITCH[i]));
		params.units[i] = CALIBRATION_OVER_MOVE;
		params.units[i] *= (CALIBRATION_DIR & (0x01 << i)) ? 1.0F : -1.0F;
	}
	params.units[2] = 0.0F;
	params.units[3] = 0.0F;

	params.speed = MAX_SPEED * 0.01f; // скорость 1% 
	params.acceleration = MAX_ACCELERATION * 0.5;
	params.deceleration = MAX_ACCELERATION * -0.5F;
	params.status = ParamsStatus::OK;

	setPositionRelative(params);
	mainController.setOnMoveFinished(calibrateFinished);
	mainController.setOnEmergensyStoped(calibrateAllEmergensyStoped);
	moveMotors(params);
}

void calibrateFinished() // результат калибровки
{
	mainController.setOnEmergensyStoped(nullptr);
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		addDataToOutQueue(COMMAND_CALIBRATE_ALL + RESULT_ERROR_STOPED);
		sendIfStopAnswer();
		return;
	}

	postCalibration(&X, 0b00000001);
	postCalibration(&Y, 0b00000010);
	// postCalibration(&Z, 0b00000100); // финализация оси Z отключена (ось Z не используется)

	// цикл ограничен осями X и Y (i < 2); бит оси Z в calibration больше не проверяется и не выставляется
	for (int i = 0; i < 2; i++){
		uint8_t result = (digitalReadFast(PIN_LIMIT_SWITCH[i]) == LEVEL_LIMIT_SWITCH[i]);
		calibration &= ~(0x01 << i); // сбросить бит
		calibration |= result << i; // установить бит, если откалибровано
	}

	if (!isCalibrated())
		trySetStatus(StatusMain::ERROR);

	String result = COMMAND_CALIBRATE_ALL;
	result += isCalibrated() ? RESULT_SUCCESS + statusToString(calibration, 3) : RESULT_ERROR_CALIBRATION_SENSOR;

	addDataToOutQueue(result);
}

String readMagazineSlotCoordinates(int slotNum)
{
	if ((slotNum < 0) || (slotNum >= SLOTS))
		return RESULT_ERROR_INVALID_PARAMS+ "SN" + String(slotNum + 1);

	const MagazineSlot* coordinates = storageParams.getMagazineSlot(slotNum);
	if (coordinates == nullptr)
		return RESULT_ERROR_NO_DATA + "SN" + String(slotNum + 1);

	return RESULT_SUCCESS + "SN" + String(slotNum + 1) + "OX" + String(coordinates->X, 5) + "OY" + String(coordinates->Y, 5) + "OZ" + String(coordinates->Z, 5);
}

void handleReadMagazineSlotCoordinates(String data)
{
	String answer = COMMAND_MAGAZINE_SLOT_COORDINATES_READ;
	int slotIndex = data.indexOf("SN");
	int slotNum = data.substring(slotIndex + 2).toInt() - 1;

	answer += readMagazineSlotCoordinates(slotNum);
	addDataToOutQueue(answer);
}

void handleWriteMagazineSlotCoordinates(String data)
{
	String answer = COMMAND_MAGAZINE_SLOT_COORDINATES_WRITE;

	int slotIndex = data.indexOf("SN");
	int X_Index = data.indexOf("OX");
	int Y_Index = data.indexOf("OY");
	int Z_Index = data.indexOf("OZ");

	if ((slotIndex == -1) || (X_Index == -1) || (Y_Index == -1) || (Z_Index == -1))
	{
		answer += RESULT_ERROR_INCORRECT_COMMAND;
		addDataToOutQueue(answer);
		return;
	}

	int slotNum = data.substring(slotIndex + 2, X_Index).toInt() - 1;

	if ((slotNum < 0) || (slotNum >= SLOTS))
	{
		answer += RESULT_ERROR_INVALID_PARAMS + "SN" + String(slotNum + 1);
		addDataToOutQueue(answer);
		return;
	}

	MagazineSlot coordinates;
	coordinates.X = data.substring(X_Index + 2, Y_Index).toFloat();
	coordinates.Y = data.substring(Y_Index + 2, Z_Index).toFloat();
	coordinates.Z = data.substring(Z_Index + 2).toFloat();

	if ((coordinates.X < X_LIMIT_MAGAZINE + X_TOOL_SIZE) || 
		(coordinates.Z < Z_OFFSET_CHANGE_TOOL) || 
		(coordinates.Z > Z.getMaxLimitUnits() - Z_OFFSET_CHANGE_TOOL / 2))
	{
		answer += RESULT_ERROR_OUT_OF_LIMITS + "SN" + String(slotNum + 1);
		addDataToOutQueue(answer);
		return;
	}

	// TODO: придумать проверку на конфликт с другими слотами?

	if (!isValidAbsoluteX(coordinates.X) || !isValidAbsoluteY(coordinates.Y) || !isValidAbsoluteZ(coordinates.Z))
	{
		answer += RESULT_ERROR_OUT_OF_LIMITS + "SN" + String(slotNum + 1);
		addDataToOutQueue(answer);
		return;
	}

	storageParams.setMagazineSlot(slotNum, coordinates);
	answer += readMagazineSlotCoordinates(slotNum);
	addDataToOutQueue(answer);
}

String readToolOffset(int toolNum)
{
	if ((toolNum < 0) || (toolNum >= SLOTS))
		return RESULT_ERROR_INVALID_PARAMS + "TN" + String(toolNum + 1);

	const ValuesXY* offset = storageParams.getToolOffset(toolNum);
	if (offset == nullptr)
		return RESULT_ERROR_NO_DATA + "TN" + String(toolNum + 1);

	return RESULT_SUCCESS + "TN" + String(toolNum + 1) + "OX" + String(offset->X, 5) + "OY" + String(offset->Y, 5);
}

void handleReadToolOffset(String data)
{
	String answer = COMMAND_TOOL_OFFSET_READ;
	int toolIndex = data.indexOf("TN");
	int toolNum = data.substring(toolIndex + 2).toInt() - 1;

	answer += readToolOffset(toolNum);
	addDataToOutQueue(answer);
}

void handleWriteToolOffset(String data)
{
	String answer = COMMAND_TOOL_OFFSET_WRITE;

	int toolIndex = data.indexOf("TN");
	int X_Index = data.indexOf("OX");
	int Y_Index = data.indexOf("OY");

	if ((toolIndex == -1) || (X_Index == -1) || (Y_Index == -1))
	{
		answer += RESULT_ERROR_INCORRECT_COMMAND;
		addDataToOutQueue(answer);
		return;
	}

	int toolNum = data.substring(toolIndex + 2, X_Index).toInt() - 1;

	if ((toolNum < 0) || (toolNum >= SLOTS))
	{
		answer += RESULT_ERROR_INVALID_PARAMS + "TN" + String(toolNum + 1);
		addDataToOutQueue(answer);
		return;
	}

	ValuesXY offset;
	offset.X = data.substring(X_Index + 2, Y_Index).toFloat();
	offset.Y = data.substring(Y_Index + 2).toFloat();

	storageParams.setToolOffset(toolNum, offset);
	answer += readToolOffset(toolNum);
	addDataToOutQueue(answer);
}

bool isEmergency()
{
	return false; // TODO: для отладки
	// return digitalReadFast(PIN_EMERGENCY) == LEVEL_EMERGENCY;
}

bool isToolInstalled()
{
	return false;
	// return digitalReadFast(PIN_TOOL_INSTALLED) == LEVEL_TOOL_INSTALLED;
}

void prepareMoveToLimit(Axis *axis, uint8_t mask)
{
	double position = 0.0F;
	double target = 0.0F;
	axis->disableLimits();
	position = (CALIBRATION_DIR & mask) ? axis->getMinLimitUnits() : axis->getMaxLimitUnits();
	axis->setCurrentPositionInUnits(position);
	target = (CALIBRATION_DIR & mask) ? axis->getMaxLimitUnits() : axis->getMinLimitUnits();
	axis->setTargetPositionAbsoluteInUnits(target);
}

void postCalibration(Axis *axis, uint8_t mask)
{
	double position = 0.0F;
	position = (CALIBRATION_DIR & mask) ? axis->getMaxLimitUnits() : axis->getMinLimitUnits();
	axis->setCurrentPositionInUnits(position);
	axis->enableLimits();
}

bool preMoveChecks(String command)
{
	String answer = command;
	if (isEmergency())
	{
		answer += RESULT_ERROR_EMERGENCY_STOPED;
		addDataToOutQueue(answer);
		return false;
	}

	if (!isCalibrated())
	{
		answer += RESULT_ERROR_NEED_CALIBRATION;
		addDataToOutQueue(answer);
		return false;
	}
	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
	{
		answer += RESULT_ERROR_MOVING_IN_PROCESS;
		addDataToOutQueue(answer);
		return false;
	}
	return true;
}

void handleMoveToHome(String data)
{
	String answer = COMMAND_MOVE_HOME;
	
	if (!preMoveChecks(answer))
		return;

	const double* homeX = storageParams.getHomeX();
	const double* homeY = storageParams.getHomeY();

	if ((homeX == nullptr) || (homeY == nullptr)) // если домашняя позиция не настроена
	{
		answer += RESULT_ERROR_NO_DATA;
		addDataToOutQueue(answer);
		return;
	}

	if (!isValidAbsoluteX(*homeX) || !isValidAbsoluteY(*homeY)) // если домашняя позиция настроена некорректно
	{
		answer += RESULT_ERROR_OUT_OF_LIMITS;
		addDataToOutQueue(answer);
		return;
	}

	int speed_Index = data.indexOf("SP");
	int ACC_Index = data.indexOf("AC");
	int DCC_Index = data.indexOf("DC");
	if ((speed_Index == -1) || (ACC_Index == -1) || (DCC_Index == -1)) // не все параметры получены
	{
		answer += RESULT_ERROR_INCORRECT_COMMAND;
		addDataToOutQueue(answer);
		return;
	}

	double speed = data.substring(speed_Index + 2, ACC_Index).toFloat();
	double acceleration = data.substring(ACC_Index + 2, DCC_Index).toFloat();
	double deceleration = data.substring(DCC_Index + 2).toFloat();

	if ((speed <= 0) || (acceleration <= 0) || (deceleration <= 0))
	{
		answer += RESULT_ERROR_INVALID_PARAMS;
		addDataToOutQueue(answer);
		return;
	}
	
	if (speed < 1.0F)
		speed = 1.0F;

	if (acceleration < 1.0F)
		acceleration = 1.0F;

	if (deceleration < 1.0F)
		deceleration = 1.0F;
	
	if (speed > 100.0F)
		speed = 100.0F;

	if (acceleration > 100.0F)
		acceleration = 100.0F;
	
	if (deceleration > 100.0F)
		deceleration = 100.0F;

	goHomeParams.speed = MAX_SPEED * speed / 100.0F;
	goHomeParams.acceleration = MAX_ACCELERATION * acceleration / 100.0F;
	goHomeParams.deceleration = MAX_ACCELERATION * deceleration / 100.0 * -1.0F;

	goHomeParams.units[0] = X.getPositionInUnits();
	goHomeParams.units[1] = Y.getPositionInUnits();
	goHomeParams.units[2] = Z.getMaxLimitUnits();
	goHomeParams.units[3] = 0.0F;
	goHomeParams.status = ParamsStatus::OK;

	if(!setPositionAbsolute(goHomeParams))
	{
		answer += RESULT_ERROR_OUT_OF_LIMITS;
		addDataToOutQueue(answer);
		return;
	}

	mainController.setOnMoveFinished(moveToHomeX);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_HOME);});
	moveMotors(goHomeParams); // поднимаем только ось Z
}

void moveToHomeX() // выводим X из зоны магазина
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_MOVE_HOME + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		moveFinished(COMMAND_MOVE_HOME, true);
		return;
	}

	if (X.getPositionInUnits() <= X_LIMIT_MAGAZINE)
	{
		moveToHomeXY();
		return;
	}

	goHomeParams.units[0] = X_LIMIT_MAGAZINE;
	if(!setPositionAbsolute(goHomeParams))
	{
		addDataToOutQueue(COMMAND_MOVE_HOME + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(moveToHomeXY);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_HOME);});
	moveMotors(goHomeParams);
}

void moveToHomeXY() // двигаем домой X и Y
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_MOVE_HOME + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		moveFinished(COMMAND_MOVE_HOME, true);
		return;
	}

	goHomeParams.units[0] = *storageParams.getHomeX();
	goHomeParams.units[1] = *storageParams.getHomeY();
	if(!setPositionAbsolute(goHomeParams))
	{
		addDataToOutQueue(COMMAND_MOVE_HOME + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished([]{moveFinished(COMMAND_MOVE_HOME, true);});
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_HOME);});
	moveMotors(goHomeParams);
}

void plazToGlobalCoordinates(double* x, double* y, double* z)
{
	if ((x != nullptr) && (y != nullptr))
	{
		double rotationRadians = *storageParams.getRotation() * PI / 180.0F;
		double oldX = *x * *storageParams.getScaleX(); 
		double oldY = *y * *storageParams.getScaleY();
		*x = oldX * cos(rotationRadians) - oldY * sin(rotationRadians) + *storageParams.getOffsetX();
		*y = oldX * sin(rotationRadians) + oldY * cos(rotationRadians) + *storageParams.getOffsetY();
	}

	if (z != nullptr)
		*z += *storageParams.getOffsetZ();
}

void globalToPlazCoordinates(double* x, double* y, double* z)
{
	if ((x != nullptr) && (y != nullptr))
	{
		double rotationRadians = *storageParams.getRotation() * PI / 180.0F;
		double oldX = *x - *storageParams.getOffsetX();
		double oldY = *y - *storageParams.getOffsetY();
		*x = oldX * cos(rotationRadians) + oldY * sin(rotationRadians);
		*y = oldY * cos(rotationRadians) - oldX * sin(rotationRadians);

		*x /= *storageParams.getScaleX();
		*y /= *storageParams.getScaleY();
	}

	if (z != nullptr)
		*z -= *storageParams.getOffsetZ();
}

void addToolOffset(int8_t toolNum, double* x, double* y)
{
	const ValuesXY *offset = storageParams.getToolOffset(toolNum);
	if (offset == nullptr)
		return;

	*x += offset->X;
	*y += offset->Y;
}

void removeToolOffset(int8_t toolNum, double* x, double* y)
{
	const ValuesXY *offset = storageParams.getToolOffset(toolNum);
	if (offset == nullptr)
		return;

	*x -= offset->X;
	*y -= offset->Y;
}

void handleMoveRelative(String data) // движение по координатам плаза
{
	String answer = COMMAND_MOVE_RELATIVE;

	if (!preMoveChecks(answer))
		return;

	MoveParams command = stringToMoveParams(data);
	double posX = X.getPositionInUnits();
	double posY = Y.getPositionInUnits();
	double posZ = Z.getPositionInUnits();
	switch (command.status)
	{
		case ParamsStatus::INCORRECT_COMMAND:
			answer += RESULT_ERROR_INCORRECT_COMMAND;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::INVALID_PARAMS:
			answer += RESULT_ERROR_INVALID_PARAMS;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::OK:
			globalToPlazCoordinates(&posX, &posY, &posZ);
			command.units[0] += posX;
			command.units[1] += posY;
			command.units[2] += posZ;
			plazToGlobalCoordinates(&command.units[0], &command.units[1], &command.units[2]);
			if ((X.getPositionInUnits() > X_LIMIT_MAGAZINE + 0.1f) || (command.units[0] > X_LIMIT_MAGAZINE + 0.1f))
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
				return;
			}

			if (setPositionAbsolute(command))
			{
				mainController.setOnMoveFinished([]{moveFinished(COMMAND_MOVE_RELATIVE, false);});
				mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_RELATIVE);});
				moveMotors(command);
			}
			else
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
			}
			break;

		default:
			answer += RESULT_ERROR_UNKNOWN;
			addDataToOutQueue(answer);
			break;
	}
}

void handleMoveAbsolute(String data) // движение по координатам плаза
{
	String answer = COMMAND_MOVE_ABSOLUTE;

	if (!preMoveChecks(answer))
		return;

	MoveParams command = stringToMoveParams(data);
	switch (command.status)
	{
		case ParamsStatus::INCORRECT_COMMAND:
			answer += RESULT_ERROR_INCORRECT_COMMAND;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::INVALID_PARAMS:
			answer += RESULT_ERROR_INVALID_PARAMS;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::OK:
			addToolOffset(currentToolNum, &command.units[0], &command.units[1]);
			plazToGlobalCoordinates(&command.units[0], &command.units[1], &command.units[2]);
			if ((X.getPositionInUnits() > X_LIMIT_MAGAZINE + 0.1f) || (command.units[0] > X_LIMIT_MAGAZINE + 0.1f))
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
				return;
			}

			if (setPositionAbsolute(command))
			{
				mainController.setOnMoveFinished([]{moveFinished(COMMAND_MOVE_ABSOLUTE, false);});
				mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_ABSOLUTE);});
				moveMotors(command);
			}
			else
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
			}
			break;

		default:
			answer += RESULT_ERROR_UNKNOWN;
			addDataToOutQueue(answer);
			break;
	}
}

void handleMoveRelativeService(String data) // движение по координатам станка
{
	String answer = COMMAND_MOVE_RELATIVE_SERVICE;
	
	if (!preMoveChecks(COMMAND_MOVE_RELATIVE_SERVICE))
		return;

	MoveParams command = stringToMoveParams(data);
	switch (command.status)
	{
		case ParamsStatus::INCORRECT_COMMAND:
			answer += RESULT_ERROR_INCORRECT_COMMAND;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::INVALID_PARAMS:
			answer += RESULT_ERROR_INVALID_PARAMS;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::OK:
			if (setPositionRelative(command))
			{
				mainController.setOnMoveFinished([]{moveFinished(COMMAND_MOVE_RELATIVE_SERVICE, true);});
				mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_RELATIVE_SERVICE);});
				moveMotors(command);
			}
			else
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
			}
			break;

		default:
			answer += RESULT_ERROR_UNKNOWN;
			addDataToOutQueue(answer);
			break;
	}
}

void handleMoveAbsoluteService(String data) // движение по координатам станка
{
	String answer = COMMAND_MOVE_ABSOLUTE_SERVICE;

	if (!preMoveChecks(COMMAND_MOVE_ABSOLUTE_SERVICE))
		return;

	MoveParams command = stringToMoveParams(data);
	switch (command.status)
	{
		case ParamsStatus::INCORRECT_COMMAND:
			answer += RESULT_ERROR_INCORRECT_COMMAND;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::INVALID_PARAMS:
			answer += RESULT_ERROR_INVALID_PARAMS;
			addDataToOutQueue(answer);
			break;

		case ParamsStatus::OK:
			if (setPositionAbsolute(command))
			{
				mainController.setOnMoveFinished([]{moveFinished(COMMAND_MOVE_ABSOLUTE_SERVICE, true);});
				mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_MOVE_ABSOLUTE_SERVICE);});
				moveMotors(command);
			}
			else
			{
				answer += RESULT_ERROR_OUT_OF_LIMITS;
				addDataToOutQueue(answer);
			}
			break;

		default:
			answer += RESULT_ERROR_UNKNOWN;
			addDataToOutQueue(answer);
			break;
	}
}

String cableCut()
{
	if (isEmergency())
		return RESULT_ERROR_EMERGENCY_STOPED;
	
	if (!isCalibrated())
		return RESULT_ERROR_NEED_CALIBRATION;

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
		return RESULT_ERROR_MOVING_IN_PROCESS;

	if (Z.getPositionInUnits() < Z_LIMIT_CUT)
		return RESULT_ERROR_OUT_OF_LIMITS;

	if (!pressureStatus)
		return RESULT_ERROR_LOW_AIR_PRESSURE;

	digitalWriteFast(PIN_CUTTER, ROOT_STATE_CUTTER);
	delay(250); // TODO: мешает сердцебиению - приведет к "аритмии" в момент обрезки))
	digitalWriteFast(PIN_CUTTER, !ROOT_STATE_CUTTER);

	return RESULT_SUCCESS;
}

void handleClampTool(String command)
{
	String answer = COMMAND_TOOL_CLAMP;
	if (isEmergency())
	{
		answer += RESULT_ERROR_EMERGENCY_STOPED;
		addDataToOutQueue(answer);
		return;
	}

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
	{
		answer += RESULT_ERROR_MOVING_IN_PROCESS;
		addDataToOutQueue(answer);
		return;
	}

	if (!pressureStatus)
	{
		answer += RESULT_ERROR_LOW_AIR_PRESSURE;
		addDataToOutQueue(answer);
		return;
	}
	
	int clampIndex = command.indexOf(COMMAND_TOOL_CLAMP);
	bool toolShouldClamp = command.substring(clampIndex + 2).toInt() > 0;
	answer += RESULT_SUCCESS;
	answer += setToolClamped(toolShouldClamp) ? "1" : "0";
	addDataToOutQueue(answer);
}

bool setToolClamped(bool shouldClamp)
{
	if (isEmergency())
		return isClamped;

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
		return isClamped;

	if (!pressureStatus)
		return isClamped;

	
	if (shouldClamp)
	{
		digitalWriteFast(PIN_CLAMPER, ROOT_STATE_CLAMPER == shouldClamp);
		delay(300); // ждем пока зафиксируется головка //TODO: избавиться от задержки?
		// if (isToolInstalled())
			A.enable();
	} else 
	{
		A.disable();
		digitalWriteFast(PIN_CLAMPER, ROOT_STATE_CLAMPER == shouldClamp);
	}

	isClamped = shouldClamp;
	return isClamped;
}

bool setPositionRelative(MoveParams params)
{
	bool result = true;
	result &= X.setTargetPositionRelativeInUnits(params.units[0]);
	result &= Y.setTargetPositionRelativeInUnits(params.units[1]);
	result &= Z.setTargetPositionRelativeInUnits(params.units[2]);
	A.setTargetPositionRelativeInUnits(params.units[3]);
	return result;
}

bool setPositionAbsolute(MoveParams params)
{
	bool result = true;
	result &= X.setTargetPositionAbsoluteInUnits(params.units[0]);
	result &= Y.setTargetPositionAbsoluteInUnits(params.units[1]);
	result &= Z.setTargetPositionAbsoluteInUnits(params.units[2]);
	A.setTargetPositionRelativeInUnits(params.units[3]);
	return result;
}

void moveMotors(MoveParams params)
{
	if (isEmergency())
		return;

	mainController.setJerkSpeedUnits(JERK);
	mainController.setRegularSpeedUnits(params.speed);
	mainController.setAccelerationUnits(params.acceleration);
	mainController.setDecelerationUnits(params.deceleration);

	mainController.moveAsync(X, Y, Z, A);
}

void moveFinished(String command, bool useGlobalCoordinates)
{
	String answer = command;
	String position = useGlobalCoordinates ? currentPositionGlobal() : currentPositionPlaz();
	answer += isStoped ? RESULT_ERROR_STOPED : position;
	addDataToOutQueue(answer);
	sendIfStopAnswer();
}

void emergensyStopedAnswer(String command)
{
	String answer = command + RESULT_ERROR_EMERGENCY_STOPED;
	addDataToOutQueue(answer);
}

void calibrateAllEmergensyStoped()
{
	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
		return;
	emergensyStopedAnswer(COMMAND_CALIBRATE_ALL);
}

uint8_t checkLimitSwitches()
{
	uint8_t result = 0;
	for (int i = 0; i < 3; i++)
	{
		result |= digitalReadFast(PIN_LIMIT_SWITCH[i]) << (i);
	}
	return result;
}

void sendIfStopAnswer(){
	if (!isStoped)
		return;

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
		return;

	String answer = COMMAND_STOP + RESULT_SUCCESS;
	addDataToOutQueue(answer);
	isStoped = false;
}

String currentPositionGlobal()
{
	if (!isCalibrated())
		return RESULT_ERROR_NEED_CALIBRATION;

	String position = RESULT_SUCCESS;
	position += "OX" + String(X.getPositionInUnits(), 5) + "OY" + String(Y.getPositionInUnits(), 5) + "OZ" + String(Z.getPositionInUnits(), 5) + "OA" + String(A.getPositionInUnits(), 5);
	return position;
}

String currentPositionPlaz()
{
	if (!isCalibrated())
		return RESULT_ERROR_NEED_CALIBRATION;

	String position = RESULT_SUCCESS;
	double posX = X.getPositionInUnits();
	double posY = Y.getPositionInUnits();
	double posZ = Z.getPositionInUnits();
	globalToPlazCoordinates(&posX, &posY, &posZ);
	removeToolOffset(currentToolNum, &posX, &posY);
	position += "OX" + String(posX, 5) + "OY" + String(posY, 5) + "OZ" + String(posZ, 5) + "OA" + String(A.getPositionInUnits(), 5);
	return position;
}

String workAreaSize()
{
	return "OX" + String(X.getMaxLimitUnits(), 5) + "OY" + String(Y.getMaxLimitUnits(), 5) + "OZ" + String(Z.getMaxLimitUnits(), 5);
}

void handleChangeTool(String command)
{
	String answer = COMMAND_CHANGE_TOOL;
	if (isEmergency())
	{
		answer += RESULT_ERROR_EMERGENCY_STOPED;
		addDataToOutQueue(answer);
		return;
	}

	if (!isCalibrated())
	{
		answer += RESULT_ERROR_NEED_CALIBRATION;
		addDataToOutQueue(answer);
		return;
	}

	if (mainController.isMoving() || aux1Controller.isMoving() || aux2Controller.isMoving())
	{
		answer += RESULT_ERROR_MOVING_IN_PROCESS;
		addDataToOutQueue(answer);
		return;
	}

	if (!pressureStatus)
	{
		answer += RESULT_ERROR_LOW_AIR_PRESSURE;
		addDataToOutQueue(answer);
		return;
	}

	int toolIndex = command.indexOf(COMMAND_CHANGE_TOOL);
	int toolNum = command.substring(toolIndex + 2).toInt() - 1;

	if ((toolNum < -1) || (toolNum >= SLOTS)) // получен некорректный номер инструмента
	{
		answer += RESULT_ERROR_INVALID_PARAMS;
		addDataToOutQueue(answer);
		return;
	}

	for (int i = 0; i < SLOTS; i++)
	{
		if (storageParams.getMagazineSlot(i) == nullptr)
		{
			trySetStatus(StatusMain::ERROR);
			answer += RESULT_ERROR_NO_DATA;
			addDataToOutQueue(answer);
			return;
		}
	}

	targetToolNum = toolNum;
	if (currentToolNum == targetToolNum) // инструмент уже установлен, действий не требуется
	{
		changeToolFinished();
		return;
	}

	dropCurrentToolPhase1();
}

void dropCurrentToolPhase1() // поднять Z
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	if (!pressureStatus)
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_LOW_AIR_PRESSURE);
		return;
	}

	setToolClamped(true);

	if ((currentToolNum < 0) && (X.getPositionInUnits() < X_LIMIT_MAGAZINE)) // нечего сбрасывать и не находимся в зоне магазина, пропускаем этапы сброса
	{
		pickupTargetToolPhase2();
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = X.getPositionInUnits();
	params.units[1] = Y.getPositionInUnits();
	params.units[2] = Z.getMaxLimitUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(dropCurrentToolPhase2);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void changeToolMoveOutMagazine(void (*callback)())
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	if (X.getPositionInUnits() <= X_LIMIT_MAGAZINE) // не находимся в зоне магазина, пропускаем этот этап
	{
		if (callback != nullptr)
			callback();
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = X_LIMIT_MAGAZINE;
	params.units[1] = Y.getPositionInUnits();
	params.units[2] = Z.getPositionInUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(callback);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void dropCurrentToolPhase2() // выехать из зоны магазина
{
	changeToolMoveOutMagazine(dropCurrentToolPhase3);
}

void dropCurrentToolPhase3() // подъехать в положение напротив слота, в который нужно сбросить податчик
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	// if ((currentToolNum < 0) && isToolInstalled()) // неизвестно, какая головка установлена / ошибка датчика
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_UNKNOWN);
	// 	return;
	// }

	// if ((currentToolNum > -1) && !isToolInstalled()) // изъяли головку / ошибка датчика
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_UNKNOWN);
	// 	return;
	// }

	if (currentToolNum < 0) // нечего сбрасывать, пропускаем этапы сброса
	{
		pickupTargetToolPhase2();
		return;
	}

	if (digitalReadFast(PIN_MAGAZINE[currentToolNum]) == LEVEL_MAGAZINE) // слот, в который нужно сбросить головку, уже занят
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OCCUPIED);
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = X_LIMIT_MAGAZINE;
	params.units[1] = storageParams.getMagazineSlot(currentToolNum)->Y;
	params.units[2] = Z.getMaxLimitUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(dropCurrentToolPhase4);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void dropCurrentToolPhase4() // заехать в слот
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(currentToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(currentToolNum)->Y;
	params.units[2] = Z.getMaxLimitUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(dropCurrentToolPhase5);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void dropCurrentToolPhase5() // зацепить податчик в магазин (опустить Z)
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(currentToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(currentToolNum)->Y;
	params.units[2] = storageParams.getMagazineSlot(currentToolNum)->Z;
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(dropCurrentToolPhase6);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void dropCurrentToolPhase6() // отсоединить податчик от станка (открыть зажим податчика и еще опустить Z)
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	if (!pressureStatus)
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_LOW_AIR_PRESSURE);
		return;
	}
	setToolClamped(false);
	delay(200); // ждем пока откроется зажим //TODO: подумать как убрать задержку

	double zPosition = storageParams.getMagazineSlot(currentToolNum)->Z - Z_OFFSET_CHANGE_TOOL;
	if(zPosition < 0.0F)
		zPosition = 0.0F;

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(currentToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(currentToolNum)->Y;
	params.units[2] = zPosition;
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(pickupTargetToolPhase1);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void pickupTargetToolPhase1() // выехать из зоны магазина
{
	// if (isToolInstalled()) // на станке не сброшена головка
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_UNKNOWN);
	// 	return;
	// }

	currentToolNum = -1;

	changeToolMoveOutMagazine(pickupTargetToolPhase2);
}

void pickupTargetToolPhase2() // подъехать в положение напротив слота, из которого нужно забрать податчик (опустив Z)
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	if (targetToolNum < 0) // задан только сброс головки, забирать нечего
	{
		setToolClamped(true);
		changeToolFinished();
		return;
	}

	double zPosition = storageParams.getMagazineSlot(targetToolNum)->Z - Z_OFFSET_CHANGE_TOOL;
	if(zPosition < 0.0F)
		zPosition = 0.0F;
	
	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = X_LIMIT_MAGAZINE;
	params.units[1] = storageParams.getMagazineSlot(targetToolNum)->Y;
	params.units[2] = zPosition;
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(pickupTargetToolPhase3);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void pickupTargetToolPhase3() // открыть зажим податчика, заехать в слот
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}
	if (!pressureStatus)
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_LOW_AIR_PRESSURE);
		return;
	}

	// if (isToolInstalled()) // Установлена головка, хотя ее быть не должно
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_UNKNOWN);
	// 	return;
	// }

	if (digitalReadFast(PIN_MAGAZINE[targetToolNum]) != LEVEL_MAGAZINE) // слот, из которого нужно забрать головку, свободен
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMPTY);
		return;
	}
	setToolClamped(false);

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(targetToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(targetToolNum)->Y;
	params.units[2] = Z.getPositionInUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(pickupTargetToolPhase4);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void pickupTargetToolPhase4() // поднять Z для подключения податчика к станку
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(targetToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(targetToolNum)->Y;
	params.units[2] = storageParams.getMagazineSlot(targetToolNum)->Z;
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(pickupTargetToolPhase5);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void pickupTargetToolPhase5() // закрыть зажим, вытащить податчик из магазина (поднять Z)
{
	if (isEmergency())
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_EMERGENCY_STOPED);
		return;
	}

	if(isStoped)
	{
		changeToolFinished();
		return;
	}

	if (!pressureStatus)
	{
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_LOW_AIR_PRESSURE);
		return;
	}
	setToolClamped(true);

	MoveParams params;
	params.speed = MAX_SPEED;
	params.acceleration = MAX_ACCELERATION;
	params.deceleration = MAX_ACCELERATION * -1.0F;
	params.units[0] = storageParams.getMagazineSlot(targetToolNum)->X;
	params.units[1] = storageParams.getMagazineSlot(targetToolNum)->Y;
	params.units[2] = Z.getMaxLimitUnits();
	params.units[3] = 0.0F;
	params.status = ParamsStatus::OK;
	if (!setPositionAbsolute(params))
	{
		trySetStatus(StatusMain::ERROR);
		addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_OUT_OF_LIMITS);
		return;
	}
	mainController.setOnMoveFinished(pickupTargetToolPhase6);
	mainController.setOnEmergensyStoped([]{emergensyStopedAnswer(COMMAND_CHANGE_TOOL);});
	moveMotors(params);
}

void pickupTargetToolPhase6() // выехать из зоны магазина
{
	// if (!isToolInstalled()) // на станке нет головки (ошибка датчика/ не зацепили головку на предыдущем этапе)
	// {
	// 	trySetStatus(StatusMain::ERROR);
	// 	addDataToOutQueue(COMMAND_CHANGE_TOOL + RESULT_ERROR_UNKNOWN);
	// 	return;
	// }

	changeToolMoveOutMagazine(changeToolFinished);
}

void changeToolFinished() // смена податчика завершена
{
	setToolClamped(true);
	String answer = COMMAND_CHANGE_TOOL;
	currentToolNum = isStoped ? currentToolNum : targetToolNum;
	answer += isStoped ? RESULT_ERROR_STOPED : RESULT_SUCCESS + String(currentToolNum + 1);
	addDataToOutQueue(answer);

	sendIfStopAnswer();
}

String readHomePosition()
{
	const double* homeX = storageParams.getHomeX();
	const double* homeY = storageParams.getHomeY();

	String answer = "";

	if ((homeX == nullptr) || (homeY == nullptr))
	{
		answer = RESULT_ERROR_NO_DATA;
		return answer;
	}

	answer = RESULT_SUCCESS;
	answer += "OX" + String(*homeX, 5) + "OY" + String(*homeY, 5);
	return answer;
}

String writeHomePosition(String command)
{
	int X_Index = command.indexOf("OX");
	int Y_Index = command.indexOf("OY");

	if ((X_Index == -1) || (Y_Index == -1))
		return RESULT_ERROR_INCORRECT_COMMAND;

	double homeX = command.substring(X_Index + 2, Y_Index).toFloat();
	double homeY = command.substring(Y_Index + 2).toFloat();

	if (!isValidAbsoluteX(homeX) || !isValidAbsoluteY(homeY))
		return RESULT_ERROR_OUT_OF_LIMITS;

	if (homeX > X_LIMIT_MAGAZINE) // конфликт с зоной магазина
		return RESULT_ERROR_OUT_OF_LIMITS;

	storageParams.setHome(homeX, homeY);
	return readHomePosition();
}

bool isValidAbsoluteX(double absoluteX)
{
	return (absoluteX < X.getMaxLimitUnits() + 0.0001) && (absoluteX > X.getMinLimitUnits() - 0.0001);
}

bool isValidAbsoluteY(double absoluteY)
{
	return (absoluteY < Y.getMaxLimitUnits() + 0.0001) && (absoluteY > Y.getMinLimitUnits() - 0.0001);
}

bool isValidAbsoluteZ(double absoluteZ)
{
	return (absoluteZ < Z.getMaxLimitUnits() + 0.00001) && (absoluteZ > Z.getMinLimitUnits() - 0.00001);
}

String readPlazParams()
{
	const double* offsetX = storageParams.getOffsetX();
	const double* offsetY = storageParams.getOffsetY();
	const double* offsetZ = storageParams.getOffsetZ();
	const double* rotation = storageParams.getRotation();
	const double* scaleX = storageParams.getScaleX();
	const double* scaleY = storageParams.getScaleY();

	String answer = "";

	if ((offsetX == nullptr) || (offsetY == nullptr) || (offsetZ == nullptr) || (rotation == nullptr) || (scaleX == nullptr) || (scaleY == nullptr))
	{
		answer = RESULT_ERROR_NO_DATA;
		return answer;
	}

	answer = RESULT_SUCCESS;
	// String умеет работать только с float, поэтому double преобразуем через toString()
	answer += "OX" + toString(*offsetX) + "OY" + toString(*offsetY) + "OZ" + toString(*offsetZ) + "OR" + toString(*rotation) + "SX" + toString(*scaleX) + "SY" + toString(*scaleY);
	return answer;
}

String writePlazParams(String command)
{
	int X_Index = command.indexOf("OX");
	int Y_Index = command.indexOf("OY");
	int Z_Index = command.indexOf("OZ");
	int rotation_Index = command.indexOf("OR");
	int scaleX_Index = command.indexOf("SX");
	int scaleY_Index = command.indexOf("SY");

	if ((X_Index == -1) || (Y_Index == -1) || (Z_Index == -1) || (rotation_Index == -1) || (scaleX_Index == -1) || (scaleY_Index == -1))
		return RESULT_ERROR_INCORRECT_COMMAND;

	double params[6];

	params[0] = toDouble(command.substring(X_Index + 2, Y_Index));
	params[1] = toDouble(command.substring(Y_Index + 2, Z_Index));
	params[2] = toDouble(command.substring(Z_Index + 2, rotation_Index));
	params[3] = toDouble(command.substring(rotation_Index + 2, scaleX_Index));
	params[4] = toDouble(command.substring(scaleX_Index + 2, scaleY_Index));
	params[5] = toDouble(command.substring(scaleY_Index + 2));

	//TODO: добавить ограничения?
	// for (int i = 0; i < 3; i++)
	// {
	// 	if (abs(params[i]) > 5) 
	// 		return RESULT_ERROR_INVALID_PARAMS;
	// }
	storageParams.setPlazParams(params[0], params[1], params[2], params[3], params[4], params[5]);
	return readPlazParams();
}

StatusMain trySetStatus(StatusMain newStatus)
{
	if (isEmergency())
		newStatus = StatusMain::EMERGENCY;
	// else if (	(!pressureStatus) || 
	// 			(isToolInstalled() && (currentToolNum < 0)) || 
	// 			(!isToolInstalled() && (currentToolNum > -1))	
	// 		) 
	else if (	(!pressureStatus)	
			)
			newStatus = StatusMain::ERROR;
		
	else if (!isConnectedToPC())
		newStatus = StatusMain::NO_CONNECTION;


	if (newStatus == statusMain)
		return statusMain;

	switch (newStatus)
	{
		case StatusMain::EMERGENCY:
			signalingColumn.setColor(SignalingColumn::RED);
			signalingColumn.enableBuzzer();
			if (statusMain == StatusMain::AUTO)
				addDataToOutQueue(COMMAND_MODE_AUTO + RESULT_SUCCESS + "0");
			break;

		case StatusMain::ERROR:
			if (statusMain == StatusMain::AUTO)
				addDataToOutQueue(COMMAND_MODE_AUTO + RESULT_SUCCESS + "0");
			signalingColumn.setColor(SignalingColumn::RED);
			signalingColumn.disableBuzzer();
			break;

		case StatusMain::NO_CONNECTION:
			signalingColumn.setColor(SignalingColumn::RED);
			signalingColumn.disableBuzzer();
			break;

		case StatusMain::READY:
			signalingColumn.setColor(SignalingColumn::YELLOW);
			signalingColumn.disableBuzzer();
			break;

		case StatusMain::AUTO:
			signalingColumn.setColor(SignalingColumn::GREEN);
			signalingColumn.disableBuzzer();
			break;
		
		default:
			break;
	}

	statusMain = newStatus;

	return newStatus;
}

void tryResetErrorStatus(String function)
{
	if (statusMain != StatusMain::ERROR)
		return;

	if ((function == COMMAND_MOVE_ABSOLUTE) || 
		(function == COMMAND_MOVE_RELATIVE) || 
		(function == COMMAND_MOVE_ABSOLUTE_SERVICE) || 
		(function == COMMAND_MOVE_RELATIVE_SERVICE) || 
		(function == COMMAND_CALIBRATE_ALL) || 
		(function == COMMAND_MOVE_HOME))
		trySetStatus(StatusMain::READY);
}

void handleSetSerialNumber(String command)
{
	String answer = COMMAND_SET_SERIAL_NUMBER;

	int numberIndex = command.indexOf(COMMAND_SET_SERIAL_NUMBER);

	int newNumber = command.substring(numberIndex + 2).toInt();

	if ((newNumber < 0) || (newNumber >= 0xFFFF))
	{
		answer += RESULT_ERROR_OUT_OF_LIMITS;
		addDataToOutQueue(answer);
		return;
	}

	storageParams.setSerialNumber(newNumber); // значение применится после перезапуска устройства
	answer += RESULT_SUCCESS + String(newNumber);
	addDataToOutQueue(answer);
}