#include "Modes.hpp"

#include "rodos.h"

static Semaphore semaphore;

static bool init_complete = config::skip_init;

static modes mode = Electrical_Startup;

/// @brief Sets mode to newmode (protected by semaphore)
void setMode(modes newmode)
{
	if (!init_complete && (newmode == Standby || newmode == Idle))
	{
		// Start up Routine
		modes currmode;
		PROTECT_WITH_SEMAPHORE(semaphore) currmode = mode;
		switch (currmode)
		{
		case Electrical_Startup:
			newmode = Calib_Gyro;
			break;;
		case Calib_Gyro: 
			newmode = Reactionwheel_Spinup;
			break;
		case Reactionwheel_Spinup:
			init_complete = true;
			newmode = Standby;
			break;
		default:
			break;
		}
	}
	PROTECT_WITH_SEMAPHORE(semaphore) mode = newmode;
}

/// @brief Gets mode (protected by semaphore)
modes getMode()
{
	modes currentmode;
	PROTECT_WITH_SEMAPHORE(semaphore) currentmode = mode;
	return currentmode;
}