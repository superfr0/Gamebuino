/*
	This file is a wrapper for include different display drivers
*/

#include "Display.h"

#ifdef USE_DISPLAY_NONE

/* Maybe we make a stub display that doesn't go anywhere */

#else

/* Fall back to PCD8544 if no alternate display is set in settings */
#include "displays/PCD8544.cpp"

#endif