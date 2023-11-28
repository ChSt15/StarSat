#include "Modes.hpp"

#include "rodos.h"

Semaphore semaphore;


/// @brief Sets mode to newmode (protected by semaphore)
void setMode(modes newmode)
{
	semaphore.enter();
	mode = newmode;
	semaphore.leave();
}

/// @brief Gets mode (protected by semaphore)
modes getMode()
{
	semaphore.enter();
	modes currentmode = mode;
	semaphore.leave();
	
	return currentmode;
}


modes mode = Idle;