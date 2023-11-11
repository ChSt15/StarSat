#ifndef FLOATSAT_TIMESTAMP_HPP
#define FLOATSAT_TIMESTAMP_HPP


/**
 * @brief Struct to contain data with a timestamp
 */
template <typename T>
struct TimestampedData {
    /// @brief Timestamp in nanoseconds
    int64_t timestamp;
    T data;
};


#endif
