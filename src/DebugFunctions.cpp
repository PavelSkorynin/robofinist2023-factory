#include "DebugFunctions.h"

#include <processes.h>

void debugGrabber(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Grabber> grabber) {
	eva->runProcess(grabber->initialize() >> grabber->halfOpen());
	eva->runProcess(ev3::WaitTimeProcess(2) >> grabber->close());
}

void debugCrane(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Crane> crane) {
	eva->runProcess(crane->freeToMove());
	eva->wait(2);
	eva->runProcess(crane->up());
	eva->wait(2);
	eva->runProcess(crane->down());
}

void debugRotations(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Move> move) {
	eva->runProcess(move->rotateToLineLeft(100, true));
	eva->wait(1);
	eva->runProcess(move->rotateToLineLeft(100, false));
	eva->runProcess(move->rotateToLineLeft(100, true));
	eva->wait(1);
	eva->runProcess(move->rotateToLineRight(100, false));
	eva->runProcess(move->rotateToLineRight(100, true));
	eva->wait(1);
}

void debugColors(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<ev3::ColorSensor> colorSensor, std::vector<int> colors) {
	for (int i = 0; i < 6; ++i) {
		auto getColorProcess = std::make_shared<GetColorProcess>(colorSensor, colors);
		eva->runProcess(getColorProcess);
		eva->lcdClean();
		eva->lcdPrintf(Color::BLACK, "%d %d", i, getColorProcess->getColor());
		eva->wait(2);
	}
}
