#include <ev3.h>
#include <processes.h>
#include <array>
#include <memory>
#include <map>

#include "Move.h"
#include "Graph.h"
#include "Grabber.h"
#include "Crane.h"

using namespace ev3;

enum class BarrelState {
	unknown,
	empty,
	barrel
};

const int NUMBER_OF_BARRELS = 6;
const int power = 100;

const std::vector<int> colors = {
		0, // красный
		60, // жёлтый
		120, // зелёный
		240 // синий
};
// перевод из индекса цвета в номер ячейки на складе
const std::map<int, int> colorToCell = {
		{-1, 0}, // чёрный
		{-2, 1}, // белый
		{0, 2}, // красный
		{1, 3}, // жёлтый
		{2, 4}, // зелёный
		{3, 5}, // синий
};

std::shared_ptr<EV3> eva;
std::shared_ptr<Move> move;

std::shared_ptr<RawReflectedLightSensor> leftLight;
std::shared_ptr<RawReflectedLightSensor> rightLight;
std::shared_ptr<ColorSensor> colorSensor;
std::shared_ptr<Sensor> distSensor;

std::shared_ptr<Motor> leftMotor;
std::shared_ptr<Motor> rightMotor;
std::shared_ptr<Motor> craneMotor;
std::shared_ptr<Motor> grabMotor;

std::shared_ptr<Grabber> grabber;
std::shared_ptr<Crane> crane;

// бочки нумеруются снизу
std::array<BarrelState, 9> barrels;
Node currentPosition = {0, 0};
Direction currentDirection = Direction::LEFT;

void calibration();
void debugSomething();

void initBarrels();
void setupSensors();
void setupMotors();

void goToStartPosition();
void findFirstBarrel();
std::pair<Node, bool> grabFirstBarrel();

int main()
{
	eva = std::make_shared<EV3>();

	initBarrels();
	setupSensors();
	setupMotors();

	grabber = std::make_shared<Grabber>(grabMotor);
	crane = std::make_shared<Crane>(craneMotor);
	move = std::make_shared<Move>(eva, leftMotor, rightMotor, leftLight, rightLight);
	move->setPower(power);

//	calibration();
	debugSomething();

	goToStartPosition();
	findFirstBarrel();
	auto target = grabFirstBarrel();
//	findPathToStore();
//	goToStore();
//	putBarrel();
//
//	for (int i = 1; i < NUMBER_OF_BARRELS; ++i) {
//		int barrel = findNearestBarrel();
//		goToBarrel(barrel);
//		grabNextBarrel();
//		findPathToStore();
//		goToStore();
//		putBarrel();
//	}

	return 0;
}

// MARK: Calibration

void calibration() {
	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/calibration.txt");
	// записываем показания датчиков линии
	eva->runProcess(MoveByEncoderOnArcProcess(leftMotor, rightMotor, 1000, 1000, 100)
			>> (StopProcess(leftMotor) | StopProcess(rightMotor)));
	// записываем показания датчика цвета
	eva->wait(2);
	eva.reset();
	exit(0);
}

void debugSomething() {
	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/log.txt");

	eva->runProcess(grabber->initialize() >> grabber->open());
//	eva->runProcess(grabber->close());
//	goToStartPosition();
//	findFirstBarrel();
//	grabFirstBarrel();

	eva.reset();
	exit(0);
}

// MARK: Initialization

void initBarrels() {
	for (auto& barrel : barrels) {
		barrel = BarrelState::unknown;
	}
}

void setupSensors() {
	eva->initSensors(Sensor::Mode::COLOR_REFLECT_RAW, Sensor::Mode::COLOR_REFLECT_RAW, Sensor::Mode::COLOR_RGB, Sensor::Mode::INFRARED_PROXIMITY);

	leftLight = eva->getRawReflectedLightSensor(Sensor::Port::P1);
	rightLight = eva->getRawReflectedLightSensor(Sensor::Port::P2);
	leftLight->setMinValue(18);
	leftLight->setMaxValue(122);
	rightLight->setMinValue(18);
	rightLight->setMaxValue(122);

	colorSensor = eva->getColorSensor(Sensor::Port::P3);
	colorSensor->setMinRValue(0);
	colorSensor->setMaxRValue(255);
	colorSensor->setMinGValue(0);
	colorSensor->setMaxGValue(255);
	colorSensor->setMinBValue(0);
	colorSensor->setMaxBValue(255);

	distSensor = eva->getSensor(Sensor::Port::P4);
}

void setupMotors() {
	leftMotor = eva->getMotor(Motor::Port::B);
	rightMotor = eva->getMotor(Motor::Port::A);
	craneMotor = eva->getMotor(Motor::Port::C);
	grabMotor = eva->getMotor(Motor::Port::D);

	// сброс моторов
	leftMotor->setPower(0);
	rightMotor->setPower(0);
	craneMotor->setPower(0);
	grabMotor->setPower(0);

	// калибровка
	leftMotor->setDirection(Motor::Direction::BACKWARD);
	rightMotor->setDirection(Motor::Direction::FORWARD);
	craneMotor->setDirection(Motor::Direction::FORWARD);
	grabMotor->setDirection(Motor::Direction::FORWARD);

	leftMotor->resetEncoder();
	rightMotor->resetEncoder();
	craneMotor->resetEncoder();
	grabMotor->resetEncoder();

	leftMotor->setEncoderScale(1);
	rightMotor->setEncoderScale(1);
	craneMotor->setEncoderScale(1);
	grabMotor->setEncoderScale(1);

	leftMotor->setMaxAccelleration(50000.0f);
	rightMotor->setMaxAccelleration(50000.0f);
	craneMotor->setMaxAccelleration(500000.0f);
	grabMotor->setMaxAccelleration(500000.0f);
}

// MARK: Strategy

void goToStartPosition() {
	eva->runProcess((move->moveOnLine(200, false) >> move->moveOnLineToCross(110, true) >> StopByEncoderOnArcProcess(leftMotor, rightMotor, -140, 140, power))
			| (grabber->initialize() >> grabber->halfOpen()));
}

void findFirstBarrel() {
	// поворачиваемся от 0 бочки к 2 и записываем показания датчика расстояния
	// (только переходы через пороговое значение)
	int startEncoder = leftMotor->getEncoder() + 40;
	std::vector<int> flipEncoders;
	flipEncoders.reserve(12);
	auto recordBarrels = std::make_shared<LambdaProcess>([&flipEncoders](ev3::time_t timestamp) {
		const int DIST_THRESHOLD = 30;
		int dist = distSensor->getValue();
		if (flipEncoders.size() % 2 == 0 && dist < DIST_THRESHOLD) {
			flipEncoders.push_back(leftMotor->getEncoder());
		}
		if (flipEncoders.size() % 2 == 1 && dist > DIST_THRESHOLD) {
			flipEncoders.push_back(leftMotor->getEncoder());
		}
		return true;
	});

	eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, 20, -20, 30) >>
			(StopByEncoderOnArcProcess(leftMotor, rightMotor, 220, -220, 30) & recordBarrels));
	flipEncoders.push_back(leftMotor->getEncoder());

	// находим бочки по записанным значениям
	auto sumSegment = [&flipEncoders](int from, int to) {
		int result = 0;
		bool isBarrel = false;
		int prevEncoder = flipEncoders[0];
		for (auto& encoder : flipEncoders) {
			if (isBarrel) {
				if (prevEncoder >= to) {
					break;
				}
				if (encoder >= from && encoder <= to) {
					if (prevEncoder >= from) {
						result += encoder - prevEncoder;
					} else {
						result += encoder - from;
					}
				}
			}
			isBarrel = !isBarrel;
			prevEncoder = encoder;
		}
		return result;
	};

	// сохраняем найденные проверенные бочки в barrels
	int firstBarrel = 3;
	for (int i = 0; i < 3; ++i) {
		int w = sumSegment(startEncoder + 220 * i / 3, startEncoder + 220 * (i + 1) / 3);
		if (w > 5) {
			barrels[i] = BarrelState::barrel;
			if (firstBarrel == 3) {
				firstBarrel = i;
			}
		} else {
			barrels[i] = BarrelState::empty;
		}
	}

	if (firstBarrel == 3) {
		for (size_t i = 3; i < barrels.size(); ++i) {
			barrels[i] = BarrelState::barrel;
		}
	}
}

std::pair<Node, bool> grabFirstBarrel() {
	int firstBarrel = 0;
	while(barrels[firstBarrel] != BarrelState::barrel) firstBarrel++;
	barrels[firstBarrel] = BarrelState::empty;

	// поворачиваемся к нужной бочке и открываем захват
	int degreesToRotate = (firstBarrel - 2) * 110;
	std::shared_ptr<Process> prepareProcess = grabber->open();
	if (degreesToRotate != 0) {
		prepareProcess = prepareProcess | std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, degreesToRotate, -degreesToRotate, 30);
	}
	eva->runProcess(prepareProcess);

	// подъезжаем к нужной бочке и закрываем захват
	int distToMove = abs((firstBarrel - 1) * 100);
	std::shared_ptr<Process> moveToBarrel = MoveByEncoderOnArcProcess(leftMotor, rightMotor, 70, 70, 50)
			>> (StopByEncoderOnArcProcess(leftMotor, rightMotor, 50, 50, 50) | SetPowerProcess(grabMotor, 10));
	if (firstBarrel != 2) {
		moveToBarrel = move->moveByEncoder(distToMove, distToMove, false) >> moveToBarrel;
	}
	eva->runProcess(moveToBarrel >> grabber->close());

	auto getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.2f);
	// возвращаемся к перекрёстку
	eva->runProcess(crane->freeToMove() | move->moveByEncoder(-120 - distToMove, -120 - distToMove, true) | getColorProcess);

	// устанавливаем стартовую позицию в графе
	currentPosition = {0, 0};
	currentDirection = Direction::LEFT;

	int color = getColorProcess->getColor();
	if (color < -2 || color > 3) {
		getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.5f);
		eva->runProcess(getColorProcess);
		color = getColorProcess->getColor();
		if (color < -2 || color > 3) {
			color = -1;
		}
	}

	// устанавливаем точку назначения
	int destCell = colorToCell.find(color)->second;
	Node destPosition = {3, destCell % 3};
	return std::make_pair<Node, bool>(std::move(destPosition), destCell / 3 == 1);
}
