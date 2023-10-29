#include <ev3.h>
#include <processes.h>
#include <array>
#include <memory>
#include <map>

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
const int ONE_BARREL_ANGLE = 110;

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
Node currentPosition = {0, 0};
Direction currentDirection = Direction::LEFT;

void calibration();
void debugSomething();

void initBarrels();
void setupSensors();
void setupMotors();

//void goToStartPosition();
//void findFirstBarrel();
//std::pair<Node, bool> grabFirstBarrel();

int findNearestBarrel();
std::vector<Action> findPathToBarrel(Node position);
int findNextBarrel(int barrel);
std::pair<Node, bool> grabNextBarrel(int barrel);
void checkNextBarrel(int barrel);
std::vector<Action> findPathToStore(Node position);
void goToNode(const std::vector<Action>& actions, bool upperShelf);
void putBarrel();

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

	eva->runProcess(grabber->initialize());

//	goToStartPosition();
//	findFirstBarrel();
//	auto target = grabFirstBarrel();
//	auto actions = findPathToStore(target.first);
//	goToNode(actions, target.second);
//	currentPosition = target.second;
//	currentDirection = Direction::RIGHT;
//	putBarrel();

	for (int i = 0; i < NUMBER_OF_BARRELS; ++i) {
		int barrel = findNearestBarrel();
		Node barrelNode = {0, barrel / 3};
		auto actions = findPathToBarrel(barrelNode);
		goToNode(actions, false);
		currentPosition = barrelNode;
		currentDirection = Direction::LEFT;
		int barrelToGrab = findNextBarrel(barrel);
		auto target = grabNextBarrel(barrelToGrab);
		checkNextBarrel(barrelToGrab);

		actions = findPathToStore(target.first);
		goToNode(actions, target.second);
		currentPosition = target.first;
		currentDirection = Direction::RIGHT;
		putBarrel();
	}

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

// MARK: Debug

void debugSomething() {
	eva->setupLogger("/home/root/lms2012/prjs/robofinist2023/log.txt");

	debugColors(eva, colorSensor, colors);

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
	colorSensor->setMinRValue(0); // 10
	colorSensor->setMaxRValue(107);
	colorSensor->setMinGValue(0); // 14
	colorSensor->setMaxGValue(152);
	colorSensor->setMinBValue(0); // 6
	colorSensor->setMaxBValue(65);

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

//void goToStartPosition() {
//	eva->runProcess((move->moveOnLine(200, false) >> move->moveOnLineToCross(110, true) >> StopByEncoderOnArcProcess(leftMotor, rightMotor, -140, 140, power))
//			| (grabber->initialize() >> grabber->halfOpen()));
//}
//
//void findFirstBarrel() {
//	// поворачиваемся от 0 бочки к 2 и записываем показания датчика расстояния
//	// (только переходы через пороговое значение)
//	int startEncoder = leftMotor->getEncoder() + 40;
//	std::vector<int> flipEncoders;
//	flipEncoders.reserve(12);
//	auto recordBarrels = std::make_shared<LambdaProcess>([&flipEncoders](ev3::time_t timestamp) {
//		const int DIST_THRESHOLD = 30;
//		int dist = distSensor->getValue();
//		if (flipEncoders.size() % 2 == 0 && dist < DIST_THRESHOLD) {
//			flipEncoders.push_back(leftMotor->getEncoder());
//		}
//		if (flipEncoders.size() % 2 == 1 && dist > DIST_THRESHOLD) {
//			flipEncoders.push_back(leftMotor->getEncoder());
//		}
//		return true;
//	});
//
//	eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, 20, -20, 30) >>
//			(StopByEncoderOnArcProcess(leftMotor, rightMotor, 220, -220, 30) & recordBarrels));
//	flipEncoders.push_back(leftMotor->getEncoder());
//
//	// находим бочки по записанным значениям
//	auto sumSegment = [&flipEncoders](int from, int to) {
//		int result = 0;
//		bool isBarrel = false;
//		int prevEncoder = flipEncoders[0];
//		for (auto& encoder : flipEncoders) {
//			if (isBarrel) {
//				if (prevEncoder >= to) {
//					break;
//				}
//				if (encoder >= from && encoder <= to) {
//					if (prevEncoder >= from) {
//						result += encoder - prevEncoder;
//					} else {
//						result += encoder - from;
//					}
//				}
//			}
//			isBarrel = !isBarrel;
//			prevEncoder = encoder;
//		}
//		return result;
//	};
//
//	// сохраняем найденные проверенные бочки в barrels
//	int firstBarrel = 3;
//	for (int i = 0; i < 3; ++i) {
//		int w = sumSegment(startEncoder + 220 * i / 3, startEncoder + 220 * (i + 1) / 3);
//		if (w > 5) {
//			barrels[i] = BarrelState::barrel;
//			if (firstBarrel == 3) {
//				firstBarrel = i;
//			}
//		} else {
//			barrels[i] = BarrelState::empty;
//		}
//	}
//
//	if (firstBarrel == 3) {
//		for (size_t i = 3; i < barrels.size(); ++i) {
//			barrels[i] = BarrelState::barrel;
//		}
//	}
//}
//
//std::pair<Node, bool> grabFirstBarrel() {
//	int firstBarrel = 0;
//	while(barrels[firstBarrel] != BarrelState::barrel) firstBarrel++;
//	barrels[firstBarrel] = BarrelState::empty;
//
//	// поворачиваемся к нужной бочке и открываем захват
//	int degreesToRotate = (firstBarrel - 2) * 110;
//	std::shared_ptr<Process> prepareProcess = grabber->open();
//	if (degreesToRotate != 0) {
//		prepareProcess = prepareProcess | std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, degreesToRotate, -degreesToRotate, 30);
//	}
//	eva->runProcess(prepareProcess);
//
//	// подъезжаем к нужной бочке и закрываем захват
//	int distToMove = abs((firstBarrel - 1) * 100);
//	std::shared_ptr<Process> moveToBarrel = MoveByEncoderOnArcProcess(leftMotor, rightMotor, 70, 70, 50)
//			>> (StopByEncoderOnArcProcess(leftMotor, rightMotor, 50, 50, 50) | SetPowerProcess(grabMotor, 10));
//	if (firstBarrel != 2) {
//		moveToBarrel = move->moveByEncoder(distToMove, distToMove, false) >> moveToBarrel;
//	}
//	eva->runProcess(moveToBarrel >> grabber->close());
//
//	auto getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.2f);
//	// возвращаемся к перекрёстку
//	eva->runProcess(crane->freeToMove() | move->moveByEncoder(-120 - distToMove, -120 - distToMove, true) | getColorProcess);
//
//	// устанавливаем стартовую позицию в графе
//	currentPosition = {0, 0};
//	currentDirection = Direction::LEFT;
//
//	int color = getColorProcess->getColor();
//	if (color < -2 || color > 3) {
//		getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.5f);
//		eva->runProcess(getColorProcess);
//		color = getColorProcess->getColor();
//		if (color < -2 || color > 3) {
//			color = -1;
//		}
//	}
//
//	// устанавливаем точку назначения
//	int destCell = colorToCell.find(color)->second;
//	Node destPosition = {3, destCell % 3};
//	return std::make_pair<Node, bool>(std::move(destPosition), destCell / 3 == 1);
//}

std::vector<Action> findPathToStore(Node position) {
	auto path = graph->pathFromNodeToNode(currentPosition, currentDirection, position);
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

void goToNode(const std::vector<Action>& actions, bool upperShelf) {
	std::shared_ptr<Process> moveProcess;
	for (size_t i = 0; i < actions.size(); ++i) {
		std::shared_ptr<Process> nextMove;
		bool stop = i == actions.size() - 1 || actions[i] != actions[i + 1];
		switch (actions[i]) {
		case Action::FORWARD:
			nextMove = move->moveOnLine(500, stop);
			break;
		case Action::TURN_LEFT:
			nextMove = move->rotateToLineLeft(100, stop);
			break;
		case Action::TURN_RIGHT:
			nextMove = move->rotateToLineRight(100, stop);
			break;
		case Action::TURN_AROUND:
			nextMove = move->rotateToLineRight(100, false);
			nextMove = move->rotateToLineRight(100, stop);
			break;
		}
		moveProcess = i == 0 ? nextMove : (moveProcess >> nextMove);
	}
	std::shared_ptr<Process> craneProcess = upperShelf
			? crane->up() : crane->freeToMove();
	eva->runProcess(moveProcess | craneProcess);
}

void putBarrel() {
	eva->runProcess(move->moveOnLine(700, true));
	eva->runProcess(grabber->open());
	eva->runProcess(move->moveByEncoder(-100, 100, true));
	eva->runProcess(((move->rotateToLineRight(200, true) | grabber->halfOpen())
			>> move->moveOnLine(700, true))
			& crane->down());
}

int findNearestBarrel() {
	int nextBarrel = 0;
	while(barrels[nextBarrel] == BarrelState::empty) nextBarrel++;
	return nextBarrel;
}

std::vector<Action> findPathToBarrel(Node position) {
	auto path = graph->pathFromNodeToNode(currentPosition, currentDirection, position);
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
	return actions;
}

int findNextBarrel(int firstBarrel) {
	int angle = ((firstBarrel % 3) - 1) * ONE_BARREL_ANGLE;
	eva->runProcess((std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, -angle, angle, power)
			| grabber->halfOpen())
			>> WaitTimeProcess(0.5f));
	if (barrels[firstBarrel] != BarrelState::barrel) {
		int startEncoder = leftMotor->getEncoder();
		eva->runProcess((StopByEncoderOnArcProcess(leftMotor, rightMotor, ONE_BARREL_ANGLE * 4, -ONE_BARREL_ANGLE * 4, 60)
				& LambdaProcess([&](ev3::time_t timestamp) {
			return distSensor->getValue() > 50;
		})) >> (StopProcess(leftMotor) | StopProcess(rightMotor)));
		int delta = leftMotor->getEncoder() - startEncoder;
		int deltaBarrel = (delta + ONE_BARREL_ANGLE / 2) / ONE_BARREL_ANGLE;
		for (int i = 0; i < deltaBarrel; ++i) {
			barrels[firstBarrel + i] = BarrelState::empty;
		}
		barrels[firstBarrel + deltaBarrel] = BarrelState::barrel;
		return firstBarrel + deltaBarrel;
	}
	return firstBarrel;
}

std::pair<Node, bool> grabNextBarrel(int nextBarrel) {
	barrels[nextBarrel] = BarrelState::empty;

	// открываем захват
	eva->runProcess(grabber->open());

	// подъезжаем к нужной бочке и закрываем захват
	int distToMove = abs(((nextBarrel % 3) - 1) * 100);
	std::shared_ptr<Process> moveToBarrel = move->moveByEncoder(distToMove, distToMove, false)
			>> MoveByEncoderOnArcProcess(leftMotor, rightMotor, 70, 70, 50)
			>> (std::make_shared<StopByEncoderOnArcProcess>(leftMotor, rightMotor, 50, 50, 50) | grabber->close());
	eva->runProcess(moveToBarrel >> grabber->close());

	auto getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors, 0.2f);
	// возвращаемся к перекрёстку
	eva->runProcess((crane->freeToMove() >> move->moveByEncoder(-120 - distToMove, -120 - distToMove, true)) | getColorProcess);

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
	if (numOfEmpty >= 3) {
		for (size_t barrel = lastBarrel + 1; barrel < barrels.size(); ++barrel) {
			barrels[barrel] = BarrelState::barrel;
		}
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

	eva->runProcess(StopByEncoderOnArcProcess(leftMotor, rightMotor, degreesToRotate + ONE_BARREL_ANGLE / 2, -degreesToRotate - ONE_BARREL_ANGLE / 2, 60)
					| LambdaProcess([&](ev3::time_t timestamp) {
		if (timestamp - prevTimestamp > 0.05f) {
			int distance = distSensor->getValue();
			if (prevDistance < 50 && distance < 50) {
				int delta = leftMotor->getEncoder() - startEncoder;
				int deltaBarrel = (delta + ONE_BARREL_ANGLE / 2) / ONE_BARREL_ANGLE;
				if (deltaBarrel == 0) {
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
}

