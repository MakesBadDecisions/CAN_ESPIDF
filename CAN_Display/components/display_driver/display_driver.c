/**
 * @file display_driver.c
 * @brief Display Hardware Abstraction Layer Implementation
 */

#include "display_driver.h"
#include "system.h"

static const char *TAG = "display";

esp_err_t display_init(void)
{
    SYS_LOGW("display_init: not implemented");
    return ESP_OK;
}

void display_clear(uint16_t color)
{
    (void)color;
    SYS_LOGW("display_clear: not implemented");
}

void display_flush(void)
{
    SYS_LOGW("display_flush: not implemented");
}

