/**
 * @file pid_types.h
 * @brief PID Data Type Definitions (Shared)
 * 
 * Common PID value types used for inter-node transport.
 * Both CAN_Interface and CAN_Display include this header.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// PID Formula Types (shared enum for both nodes)
// ============================================================================

typedef enum {
    PID_FORMULA_TYPE_LINEAR,
    PID_FORMULA_TYPE_BITFIELD,
    PID_FORMULA_TYPE_LOOKUP,
    PID_FORMULA_TYPE_CUSTOM,
    PID_FORMULA_TYPE_IDENTITY,
} pid_formula_type_t;

// ============================================================================
// Units (shared enum — canonical for both nodes)
// ============================================================================

typedef enum {
    PID_UNIT_NONE = 0,
    PID_UNIT_RPM,
    PID_UNIT_KMH,
    PID_UNIT_MPH,
    PID_UNIT_CELSIUS,
    PID_UNIT_FAHRENHEIT,
    PID_UNIT_PERCENT,
    PID_UNIT_VOLT,
    PID_UNIT_KPA,
    PID_UNIT_PA,
    PID_UNIT_PSI,
    PID_UNIT_GS,           // grams per second
    PID_UNIT_DEGREE,
    PID_UNIT_MA,            // milliamps
    PID_UNIT_SECONDS,
    PID_UNIT_MS,            // milliseconds
    PID_UNIT_MINUTES,
    PID_UNIT_RATIO,
    PID_UNIT_COUNT,
    PID_UNIT_KM,
    PID_UNIT_NM,            // Newton-meters (torque)
    PID_UNIT_LPH,           // Liters per hour
    PID_UNIT_MG_STROKE,     // Milligrams per stroke
    PID_UNIT_MAX            // sentinel — keep last
} pid_unit_t;

// ============================================================================
// Convertible Unit Pairs
// A "base unit" can be converted to one or more "alt units".
// ============================================================================

typedef struct {
    pid_unit_t  base;       // native/base unit (e.g. CELSIUS)
    pid_unit_t  alt;        // alternative unit  (e.g. FAHRENHEIT)
} pid_unit_pair_t;

// ============================================================================
// PID Value - Canonical decoded PID reading
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t pid_id;        // OBD-II PID number
    float    value;         // Decoded numeric value
    uint8_t  unit;          // pid_unit_t
    uint32_t timestamp_ms;  // Milliseconds since node boot
} pid_value_t;

// ============================================================================
// Shared API — implemented in pid_types.c
// ============================================================================

/**
 * @brief Get display string for a unit enum value
 * @return Short string like "RPM", "kPa", "C", "F", etc.
 */
const char *pid_unit_str(pid_unit_t unit);

/**
 * @brief Convert a value between compatible units
 * @param value    Input value
 * @param from     Source unit
 * @param to       Target unit
 * @param[out] out Converted value (untouched on failure)
 * @return true if conversion succeeded, false if pair is not supported
 */
bool pid_unit_convert(float value, pid_unit_t from, pid_unit_t to, float *out);

/**
 * @brief Get list of alternative display units for a given base unit
 * @param base      The native/base unit of a PID
 * @param[out] alts Caller array to receive alt unit enum values
 * @param max_alts  Size of alts array
 * @return Number of alternatives written (0 = no conversions available)
 */
int pid_unit_get_alts(pid_unit_t base, pid_unit_t *alts, int max_alts);

