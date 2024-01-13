#include "InnerLoopTopics.hpp"

Topic<TimestampedData<float>> EncoderDataTopic(-1, "EncoderData");

Topic<float> speedSetpointTopic(-1, "SetPoint (Reactionwheel)");