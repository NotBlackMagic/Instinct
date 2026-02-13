#include "topics.hpp"

Topic<ImuMsg> topicImu("imu", static_cast<uint8_t>(TopicID::Imu));
Topic<MagMsg> topicMag("mag", static_cast<uint8_t>(TopicID::Mag));
Topic<BaroMsg> topicBaro("baro", static_cast<uint8_t>(TopicID::Baro));