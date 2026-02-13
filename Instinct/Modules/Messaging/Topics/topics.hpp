#pragma once

//Include the PubSub Library
#include "topic.hpp"

//Include the Message Definitions
#include "sensors.hpp"

//Include the Message/Topic IDs
#include "topicIDs.hpp"

//Sensor Topics
extern Topic<ImuMsg> topicImu;
extern Topic<MagMsg> topicMag;
extern Topic<BaroMsg> topicBaro;