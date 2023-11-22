#include "CalibrationIMU.hpp"

#include "rodos.h"


void CalibrationIMU::init(){

}


void CalibrationIMU::run(){

    /**
     * Rough idea
    */

   while(true){
        suspendCallerUntil(NOW() + this->modePeriod * MILLISECONDS);

        // Update this->calibMode

        switch (this->calibMode)
        {
        case CalibAccel:
        {
            const IMUCalib calibDataAccel = calibrateAccel();
            imu.setAccelCalib(calibDataAccel);
            break;
        }
        case CalibGyro:
        {
            const IMUCalib calibDataGyro = calibrateGyro();
            imu.setGyroCalib(calibDataGyro);
            break;
        }
        case CalibMag:
        {
            const IMUCalib calibDataMag = calibrateMag();
            imu.setMagCalib(calibDataMag);
            break;
        }
        case DoNothing:
            break;
        }
   }

}


IMUCalib CalibrationIMU::calibrateAccel(){

}


IMUCalib CalibrationIMU::calibrateMag(){

}


IMUCalib CalibrationIMU::calibrateGyro(){

}
