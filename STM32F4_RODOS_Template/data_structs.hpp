#ifndef FLOATSAT_DATASTRUCTS_HPP_
#define FLOATSAT_DATASTRUCTS_HPP_


#include "rodos.h"

/**
 * @brief This header contains general data structures used by multiple applications
 */


/**
 * @brief This template is used to create data structure containing a timestamp
*/
template <typename TYPE>
struct TimestampedData {
    uint64_t timestamp;
    TYPE data;
};

#endif /* FLOATSAT_DATASTRUCTURES_HPP_ */