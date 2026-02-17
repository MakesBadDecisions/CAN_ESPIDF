/**
 * @file device.h
 * @brief Device Header Selector
 * 
 * Includes the correct device header based on build flags.
 * Add -DDEVICE_<name> to platformio.ini build_flags to select.
 * 
 * Available devices:
 *   -DDEVICE_DIS06043H  -> 4.3" CrowPanel (recommended - has UART1 on GPIO17/18)
 *   -DDEVICE_DIS07050   -> 5.0" CrowPanel  
 *   -DDEVICE_DIS08070H  -> 7.0" CrowPanel
 */

#pragma once

#if defined(DEVICE_DIS06043H)
    #include "dis06043h.h"
#elif defined(DEVICE_DIS07050)
    #include "dis07050.h"
#elif defined(DEVICE_DIS08070H)
    #include "dis08070h.h"
#else
    // Default to 4.3" if no device specified
    #warning "No device specified, defaulting to DIS06043H (4.3 inch)"
    #include "dis06043h.h"
#endif
