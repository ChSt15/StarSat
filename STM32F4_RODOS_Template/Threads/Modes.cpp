#include "Modes.hpp"

#include "rodos.h"

Semaphore semaphore;


/// @brief Sets mode to newmode (protected by semaphore)
void setMode(modes newmode)
{
	PROTECT_WITH_SEMAPHORE(semaphore) mode = newmode;
}

/// @brief Gets mode (protected by semaphore)
modes getMode()
{
	modes currentmode;
	PROTECT_WITH_SEMAPHORE(semaphore) currentmode = mode;
	return currentmode;
}


modes mode = Idle;