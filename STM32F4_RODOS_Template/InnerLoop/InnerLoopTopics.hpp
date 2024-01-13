#ifndef FLOATSAT_INNERLOOP_INNERLOOPTOPICS_HPP_
#define FLOATSAT_INNERLOOP_INNERLOOPTOPICS_HPP_

#include "rodos.h"
#include "../timestamp.hpp"


extern Topic<float> speedSetpointTopic;

extern Topic<TimestampedData<float>> EncoderDataTopic;


#endif