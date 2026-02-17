/**
 * @file wifi_ap.h
 * @brief WiFi AP - DEPRECATED (moved to Display Node)
 * 
 * This is a stub for build compatibility. WiFi AP functionality
 * is implemented in the Display Node's wifi_manager component.
 */

#pragma once

#include "esp_err.h"

// Stub - WiFi AP is on Display Node only
static inline esp_err_t wifi_ap_init(void) { return ESP_OK; }

