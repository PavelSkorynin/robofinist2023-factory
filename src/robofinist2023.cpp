#include <ev3.h>
#include <processes.h>
#include <array>
#include <memory>
#include <map>
#include <fstream>

#include "Move.h"
#include "Graph.h"
#include "Grabber.h"
#include "Crane.h"

#include "DebugFunctions.h"

using namespace ev3;

enum class BarrelState {
	unknown,
	empty,
	barrel
};

const int NUMBER_OF_BARRELS = 6;
const int power = 100;
const int ONE_BARREL_ANGLE = 80;
const bool USE_CHECK = false;
const bool USE_DEBUG_WAIT = false;

const std::vector<int> colors = {
		0, // красный
		60, // жёлтый
		120, // зелёный
		240 // синий
};
// перевод из индекса цвета в номер ячейки на складе
const std::map<int, int> colorToCell = {
		{-1, 4}, // чёрный
		{-2, 1}, // белый
		{0, 0}, // красный
		{1, 5}, // жёлтый
		{2, 3}, // зелёный
		{3, 2}, // синий
};

std::shared_ptr<Graph> graph = std::make_shared<Graph>();

std::shared_ptr<EV3> eva;
std::shared_ptr<Move> move;
std::shared_ptr<Grabber> grabber;
std::shared_ptr<Crane> crane;

std::shared_ptr<RawReflectedLightSensor> leftLight;
std::shared_ptr<RawReflectedLightSensor> rightLight;
std::shared_ptr<ColorSensor> colorSensor;
std::shared_ptr<Sensor> distSensor;

std::shared_ptr<Motor> leftMotor;
std::shared_ptr<Motor> rightMotor;
std::shared_ptr<Motor> craneMotor;
std::shared_ptr<Motor> grabMotor;

// бочки нумеруются снизу
std::array<BarrelState, 9> barrels;
Node currentPosition = {1, 0};
Direction currentDirection = Direction::LEFT;
int barrelsDone = 0;

//void calibration();
void debugSomething();
void debugWait(ev3::time_t waitTime);

void initBarrels();
void setupSensors();
void setupMotors();

//void goToStartPosition();
//void findFirstBarrel();
//std::pair<Node, bool> grabFirstBarrel();

int findNearestBarrel();
std::vector<Action> findPathToBarrel(Node position);
int findNextBarrel(int barrel);
std::pair<Node, bool> grabNextBarrel(int distance);
void checkNextBarrel(int barrel);
std::vector<Action> findPathToStore(Node position);
void goToNode(const std::vector<Action>& actions, bool upperShelf, bool barrel);
void putBarrel();
void outputBarrels();

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
//	debugSomething();

	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/run.txt");

	eva->runProcess(grabber->initialize() >> grabber->halfOpen());

	for (int i = 0; i < NUMBER_OF_BARRELS; ++i) {
		eva->wait(1);
		int barrel = findNearestBarrel();
//		eva->lcdPrintf(Color::BLACK, "%d ", barrel);
		Node barrelNode = {0, barrel / 3};
		auto actions = findPathToBarrel(barrelNode);

//		eva->lcdPrintf(Color::BLACK, "goB ");
		goToNode(actions, false, true);
//		eva->lcdPrintf(Color::BLACK, "findB ");
		eva->runProcess(crane->down());
		currentPosition = barrelNode;
		currentDirection = Direction::LEFT;
		int distanceToBarrel = findNextBarrel(barrel);
		auto target = grabNextBarrel(distanceToBarrel);

		if (USE_CHECK) {
			int barrelToGrab = 0;
			for(; barrelToGrab < (int)barrels.size(); ++barrelToGrab) {
				if (barrels[barrelToGrab] == BarrelState::barrel) {
					barrels[barrelToGrab] = BarrelState::empty;
					break;
				}
			}
			checkNextBarrel(barrelToGrab);
		}
		outputBarrels();

//		eva->lcdPrintf(Color::BLACK, "findS ");
		actions = findPathToStore(target.first);
//		eva->lcdPrintf(Color::BLACK, "doS ");
		goToNode(actions, target.second, false);
		currentPosition = target.first;
		currentDirection = Direction::LEFT;
		putBarrel();
		barrelsDone++;
	}

	return 0;
}

// MARK: Calibration

//void calibration() {
//	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/calibration.txt");
//	// записываем показания датчиков линии
//	eva->runProcess(MoveByEncoderOnArcProcess(leftMotor, rightMotor, 1000, 1000, 100)
//			>> (StopProcess(leftMotor) | StopProcess(rightMotor)));
//	// записываем показания датчика цвета
//	eva->wait(2);
//	eva.reset();
//	exit(0);
//}

// MARK: Debug

void debugSomething() {
	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/log.txt");

	eva->runProcess(move->moveOnLine(1000, true) | crane->freeToMove() | grabber->open());

//	eva->runProcess(move->moveOnLineToCross(500, true));
//	eva->wait(1);
//	eva->runProcess(move->moveOnLineToCross(500, true));
//	eva->wait(1);
//	eva->runProcess(move->moveOnLineToCross(500, true));

//	debugColors(eva, colorSensor, colors);

	eva.reset();
	exit(0);
}

void debugWait(ev3::time_t waitTime) {
	if (USE_DEBUG_WAIT) {
		eva->wait(waitTime);
	}
}

// MARK: Initialization

void initBarrels() {
	for (auto& barrel : barrels) {
		barrel = BarrelState::unknown;
	}
}

void setupSensors() {
	eva->initSensors(Sensor::Mode::COLOR_REFLECT_RAW, Sensor::Mode::COLOR_REFLECT_RAW, Sensor::Mode::COLOR_RGB, Sensor::Mode::INFRARED_PROXIMITY);

	// читаем калибровочные значения из файла
	int minLeft, maxLeft, minRight, maxRight, maxR, maxG, maxB;
	FILE* fIn = fopen("/home/root/lms2012/prjs/robofinist2023/calibration.txt", "r");
	fscanf(fIn, "%d %d %d %d %d %d %d", &minLeft, &maxLeft, &minRight, &maxRight, &maxR, &maxG, &maxB);
	fclose(fIn);

//	eva->lcdPrintf(Color::BLACK, "%d %d  %d %d  %d %d %d", minLeft, maxLeft, minRight, maxRight, maxR, maxG, maxB);
//	eva->wait(5);

	leftLight = eva->getRawReflectedLightSensor(Sensor::Port::P1);
	rightLight = eva->getRawReflectedLightSensor(Sensor::Port::P2);
	leftLight->setMinValue(minLeft);
	leftLight->setMaxValue(maxLeft);
	rightLight->setMinValue(minRight);
	rightLight->setMaxValue(maxRight);

	colorSensor = eva->getColorSensor(Sensor::Port::P3);
	colorSensor->setMinRValue(0); // 10
	colorSensor->setMaxRValue(maxR);
	colorSensor->setMinGValue(0); // 14
	colorSensor->setMaxGValue(maxG);
	colorSensor->setMinBValue(0); // 6
	colorSensor->setMaxBValue(maxB);

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

std::vector<Action> findPathToStore(Node position) {
	eva->lcdPrintf(Color::BLACK, "pos %d %d, %d %d", currentPosition.x, currentPosition.y, position.x, position.y);
	auto path = graph->pathFromNodeToNode(currentPosition, currentDirection, position);
	eva->lcdPrintf(Color::BLACK, " path ");
	for (auto action : path.first) {
		eva->lcdPrintf(Color::BLACK, "%d ", (int)action);
	}
	debugWait(5);

	auto actions = path.first;
	if (path.second == Direction::UP) {
		actions.push_back(Action::TURN_RIGHT);
	}
	if (path.second == Direction::DOWN) {
		actions.push_back(Action::TURN_LEFT);
	}
	if (path.second == Direction::LEFT) {
		actions.push_back(Action::TURN_AROUND);
	}
	return actions;
}

std::vector<Action> findPathToBarrel(Node position) {
	Node preNode = {1, position.y};
	auto path = graph->pathFromNodeToNode(currentPosition, currentDirection, preNode);
	auto actions = path.first;
	if (path.second == Direction::UP) {
		actions.push_back(Action::TURN_LEFT);
	}
	if (path.second == Direction::DOWN) {
		actions.push_back(Action::TURN_RIGHT);
	}
	if (path.second == Direction::RIGHT) {
		actions.push_back(Action::TURN_AROUND);
	}
	actions.push_back(Action::FORWARD);
	return actions;
}

void goToNode(const std::vector<Action>& actions, bool upperShelf, bool barrel) {
	std::shared_ptr<Process> moveProcess;
	for (size_t i = 0; i < actions.size(); ++i) {
		std::shared_ptr<Process> nextMove;
		bool stop = i == actions.size() - 1 || actions[i] != actions[i + 1];
		switch (actions[i]) {
		case Action::FORWARD:
			nextMove = move->moveOnLine(500, false) >> move->moveOnLineToCross(155, stop);
			break;
		case Action::TURN_LEFT:
			nextMove = move->rotateToLineLeft(50, stop);
			break;
		case Action::TURN_RIGHT:
			nextMove = move->rotateToLineRight(50, stop);
			break;
		case Action::TURN_AROUND:
			nextMove = move->rotateToLineRight(50, false) >> move->rotateToLineRight(200, stop);
			break;
		}
		moveProcess = i == 0 ? nextMove : (moveProcess >> nextMove);
	}

	std::shared_ptr<Process> craneProcess = upperShelf
			? crane->up() : (barrel ? (grabber->open() >> WaitTimeProcess(0.2f) >> crane->down()) : crane->freeToMove());
	if (moveProcess == nullptr) {
		eva->runProcess(craneProcess);
	} else {
		eva->runProcess(moveProcess | craneProcess);
	}
}

void putBarrel() {
	eva->runProcess(move->moveOnLine(680, true) & std::make_shared<WaitTimeProcess>(3.0f));
	eva->runProcess(grabber->open());
	eva->runProcess(move->moveOnLine(100, true) & std::make_shared<WaitTimeProcess>(1.0f));
	eva->runProcess(move->moveByEncoder(-150, -150, true));
	eva->runProcess(move->rotateToLineRight(200, true)
			>> move->moveOnLine(680, true));
}

int findNearestBarrel() {
	int nextBarrel = 0;
	while(nextBarrel < (int)barrels.size() - 1 && barrels[nextBarrel] == BarrelState::empty) nextBarrel++;
	return nextBarrel;
}

int findNextBarrel(int firstBarrel) {
	if (USE_CHECK) {
		int angle = ((firstBarrel % 3) - 1) * ONE_BARREL_ANGLE;
	//	eva->lcdPrintf(Color::BLACK, " fr");
	//	eva->wait(3);
		eva->runProcess((std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, angle, -angle, power / 2)
				>> grabber->halfOpen())
				>> WaitTimeProcess(0.5f));
		if (barrels[firstBarrel] != BarrelState::barrel) {
			int startEncoder = leftMotor->getEncoder();
	//		eva->lcdPrintf(Color::BLACK, " ff");
	//		eva->wait(3);
			auto maxAngle = (3 - (firstBarrel % 3)) * ONE_BARREL_ANGLE + ONE_BARREL_ANGLE / 2;
			eva->runProcess((StopByEncoderOnArcProcess(leftMotor, rightMotor, maxAngle, -maxAngle, 30)
					& LambdaProcess([&](ev3::time_t timestamp) {
				return distSensor->getValue() > 60;
			})) >> (StopProcess(leftMotor) | StopProcess(rightMotor)));
			int distanceToBarrel = distSensor->getValue() * 2.5f;
			int delta = leftMotor->getEncoder() - startEncoder;
	//		eva->lcdPrintf(Color::BLACK, " coB");
	//		eva->wait(3);
			eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, -ONE_BARREL_ANGLE / 4, ONE_BARREL_ANGLE / 4, 30));
			int deltaBarrel = (delta) / ONE_BARREL_ANGLE;
			if (deltaBarrel < 0) {
				deltaBarrel = 0;
			}
			if (deltaBarrel > 3) {
				deltaBarrel = 3;
			}
			for (int i = 0; i < deltaBarrel; ++i) {
				barrels[firstBarrel + i] = BarrelState::empty;
			}
			barrels[firstBarrel + deltaBarrel] = BarrelState::barrel;
			return distanceToBarrel;
		}
		return firstBarrel;
	} else {
		eva->runProcess((std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, -ONE_BARREL_ANGLE, ONE_BARREL_ANGLE, power / 2)
				>> grabber->halfOpen())
				>> WaitTimeProcess(0.5f));
		eva->runProcess((SetPowerProcess(leftMotor, 30) | SetPowerProcess(rightMotor, -30))
				>> (WaitLineProcess(rightLight) & LambdaProcess([&](ev3::time_t timestamp) {
			return distSensor->getValue() > 50;
		})));
		eva->runProcess(StopProcess(leftMotor) | StopProcess(rightMotor));

		// довернули до линии - нужно переехать к следующему перекрёстку
		if (rightLight->getValue() < 50) {
			// последний перекрёсток - дальше идти некуда
			if (currentPosition.y == 2) {
				eva.reset();
				exit(0);
			}

			for (int i = 0; i < 3; ++i) {
				barrels[currentPosition.y * 3 + i] = BarrelState::empty;
			}

			eva->runProcess(move->alignToLine(true)
					>> move->moveOnLine(500, false)
					>> move->moveOnLineToCross(155, true)
					>> move->rotateLeft(true)
					);
			currentPosition.y++;
			return findNextBarrel(currentPosition.y * 3);
		} else {
			int distanceToBarrel = distSensor->getValue() * 2.7f;
//			eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, -ONE_BARREL_ANGLE / 4, ONE_BARREL_ANGLE / 4, 30));
			return distanceToBarrel;
		}
	}
}

std::pair<Node, bool> grabNextBarrel(int distToMove) {
//	if (USE_CHECK) {
//		barrels[nextBarrel] = BarrelState::empty;
//	}

	// открываем захват
	eva->runProcess(grabber->open());

//	eva->lcdPrintf(Color::BLACK, "%d ", nextBarrel);
//	eva->wait(5);

	if (distToMove < 10) {
		distToMove = 0;
	}
	// подъезжаем к нужной бочке и закрываем захват
	std::shared_ptr<Process> moveToBarrel = MoveByEncoderOnArcProcess(leftMotor, rightMotor, distToMove + 150, distToMove + 150, 50)
			>> (std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, 50, 50, 50) | grabber->close());
	eva->runProcess(moveToBarrel >> grabber->close());

//	eva->lcdPrintf(Color::BLACK, " closed");
//	eva->wait(3);

	auto getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.2f);
	// возвращаемся к перекрёстку
	eva->runProcess(std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, -200 - distToMove, -200 - distToMove, 50)
			| getColorProcess);

//	eva->lcdPrintf(Color::BLACK, " ret");
//	eva->wait(3);

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

void checkNextBarrel(int lastBarrel) {
	int numOfEmpty = 0;
	for (auto& state : barrels) {
		if (state == BarrelState::empty) {
			numOfEmpty++;
		}
	}
	if (numOfEmpty - barrelsDone >= 3) {
		for (size_t barrel = lastBarrel + 1; barrel < barrels.size(); ++barrel) {
			barrels[barrel] = BarrelState::barrel;
		}
		return;
	}
	// не делаем проверку следующих, если забрали последнюю банку в тройке или следующую за ней
	if ((lastBarrel >= 2 && currentPosition.y == 0)
			|| (lastBarrel >= 5 && currentPosition.y == 1)) {
		return;
	}

	int degreesToRotate = (2 - (lastBarrel % 3)) * ONE_BARREL_ANGLE;
	// если мы последней забрали третью бочку по текущему направлению, то дальше не проверяем
	if (degreesToRotate == 0) {
		return;
	}

	ev3::time_t prevTimestamp = eva->timestamp();
	int prevDistance = 51;
	int startEncoder = leftMotor->getEncoder();

	eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, degreesToRotate + ONE_BARREL_ANGLE / 2, -degreesToRotate - ONE_BARREL_ANGLE / 2, 30)
					& LambdaProcess([&](ev3::time_t timestamp) {
		if (timestamp - prevTimestamp > 0.05f) {
			int distance = distSensor->getValue();
			if (prevDistance < 60 && distance < 60) {
				int delta = leftMotor->getEncoder() - startEncoder;
				int deltaBarrel = (delta + ONE_BARREL_ANGLE / 2) / ONE_BARREL_ANGLE;
				if (deltaBarrel <= 0) {
					deltaBarrel = 1;
				}
				if (deltaBarrel > 2) {
					deltaBarrel = 2;
				}
				barrels[lastBarrel + deltaBarrel] = BarrelState::barrel;
			}
		}
		return true;
	}));
	eva->runProcess(crane->freeToMove());
}

void outputBarrels() {
//	eva->lcdClean();
//	for (auto barrel : barrels) {
//		eva->lcdPrintf(Color::BLACK, "%d ", (int)barrel);
//	}
}

