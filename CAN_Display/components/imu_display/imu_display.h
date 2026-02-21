/**
 * @file imu_display.h
 * @brief IMU Visualization for Gauge Slot Panels
 *
 * Creates a 2D G-force / tilt bubble + readout labels inside any
 * gauge panel.  The bubble position represents lateral and longitudinal
 * forces (G-Load mode) or pitch/roll inclination (Tilt mode).
 *
 * Designed to render between the PID and Unit dropdowns that occupy
 * the top and bottom ~20% of each gauge panel.  Only one gauge slot
 * can host the IMU at a time (single physical sensor).
 *
 * Data comes from the QMI8658 background task via qmi8658_get_orientation().
 * Must be called with the display lock held (runs in LVGL context).
 *
 * This component is intentionally outside components/ui/ so SquareLine
 * Studio exports do not overwrite it.
 */

#pragma once

#include "lvgl.h"
#include "gauge_engine.h"   /* imu_display_mode_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Attach IMU visualization to a gauge slot panel
 *
 * Creates crosshair, bubble dot, and acceleration labels inside the
 * given parent panel.  Starts a 10 Hz LVGL timer for live updates.
 * If already attached elsewhere, detaches from the old panel first.
 *
 * The visualization is constrained to the center ~60% of the panel
 * to avoid overlapping PID/Unit dropdowns at top/bottom.
 *
 * @param parent  The gauge panel object (e.g. ui_gauge1)
 */
void imu_display_attach(lv_obj_t *parent);

/**
 * @brief Detach IMU visualization — delete all widgets and stop timer
 *
 * Safe to call even if not currently attached.
 */
void imu_display_detach(void);

/**
 * @brief Check if IMU display is currently attached to a panel
 * @return true if rendering
 */
bool imu_display_is_attached(void);

/**
 * @brief Get the panel the IMU display is currently attached to
 * @return Parent panel pointer, or NULL if detached
 */
lv_obj_t *imu_display_get_parent(void);

/**
 * @brief Set visualization mode (G-Load or Tilt)
 *
 * G-Load: bubble maps lateral/longitudinal acceleration (±1.5G).
 * Tilt:   bubble maps pitch/roll angles (±30°).
 *
 * @param mode  IMU_MODE_G_LOAD or IMU_MODE_TILT
 */
void imu_display_set_mode(imu_display_mode_t mode);

// ---------- Legacy API (backward compat, calls attach/detach) ----------

/** @deprecated Use imu_display_attach() */
void imu_display_init(lv_obj_t *parent);

/** @deprecated Use imu_display_detach() */
void imu_display_deinit(void);

#ifdef __cplusplus
}
#endif
