#ifndef FLOATSAT_TIMESTAMP_
#define FLOATSAT_TIMESTAMP_


/**
 * @brief Struct to contain data with a timestamp
 */
template <typename T>
struct TimestampedData {
    int64_t timestamp;
    T data;
};


#endif
