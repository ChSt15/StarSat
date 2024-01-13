#ifndef FLOATSAT_THREADS_MODES_HPP_
#define FLOATSAT_THREADS_MODES_HPP_

#include "rodos.h"


enum modes
{
	Idle = 0,
	Calib_Gyro, Calib_Accel, Calib_Mag, Calib_Arm,
	Control_Speed, Control_Pos, Control_Vel, 
	Mission_Locate, Mission_Point, Mission_Dock_initial, Mission_Dock_final
};


void setMode(modes mode);

modes getMode();


extern modes mode;

#endif