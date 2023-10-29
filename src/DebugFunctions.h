#pragma once

#include <EV3.h>
#include <memory>
#include <vector>

#include "Move.h"
#include "Crane.h"
#include "Grabber.h"

void debugGrabber(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Grabber> grabber);
void debugCrane(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Crane> crane);
void debugRotations(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<Move> move);
void debugColors(std::shared_ptr<ev3::EV3> eva, std::shared_ptr<ev3::ColorSensor> colorSensor, std::vector<int> colors);
