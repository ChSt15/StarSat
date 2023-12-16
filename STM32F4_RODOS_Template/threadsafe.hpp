#ifndef FLOATSAT_THREADSAFE_HPP
#define FLOATSAT_THREADSAFE_HPP


#include "rodos.h"


/**
 * @brief Contains data to be protected against race conditions. Use for safe and easy access to data from or between multiple threads.
 * @example imu.calibData.set(...) or imu.calibData = ... to safely set imu calib params from another thread. IMU can then use imu.calibData.get() or ... = imu.calibData to get the data safely.
 */
template <typename TYPE>
struct Threadsafe {

private:
    /// @brief Mutex to protect data
    RODOS::Semaphore sem;
    /// @brief Data to be protected
    TYPE data;
    /// @brief Flag to indicate if data is new. Will be reset by get()
    bool newData = false;

public:

    /**
     * @brief Construct a new Threadsafe object
     * @param data Data to be protected
    */
    Threadsafe(const TYPE& data = TYPE()) 
    {
        set(data);
    }
    
    /**
     * @brief Copies the given data into the protected data
    */
    void set(const TYPE& dataToCopy) {
        //sem.enter();
        newData = true;
        data = dataToCopy;
        //sem.leave();
    }

    /**
     * @brief Returns a copy the protected data
     * @return protected data
    */
    TYPE get() {
        TYPE dataRet;
        sem.enter();
        newData = false;
        dataRet = data;
        sem.leave();
        return dataRet;
    }

    bool isNew() {
        bool newDataRet;
        sem.enter();
        newDataRet = newData;
        sem.leave();
        return newDataRet;
    }

    /**
     * @brief Copies the given data into the protected data
    */
    void operator=(const TYPE& data) {
        set(data);
    }

    /**
     * @brief Returns a copy the protected data
     * @return protected data
    */
    operator TYPE() {
        return get();
    }

};


#endif
