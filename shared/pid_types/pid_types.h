/**
 * @file pid_types.h
 * @brief PID Data Type Definitions (Shared)
 * 
 * Common PID value types used for inter-node transport.
 */

#pragma once

#include <stdint.h>

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
// Units (shared enum)
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
    PID_UNIT_PSI,
    PID_UNIT_GS,
    PID_UNIT_DEGREE,
    PID_UNIT_SECONDS,
    PID_UNIT_RATIO,
    PID_UNIT_COUNT,
} pid_unit_t;

// ============================================================================
// PID Value - Canonical decoded PID reading
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t pid_id;        // OBD-II PID number
    float    value;         // Decoded numeric value
    uint8_t  unit;          // pid_unit_t
    uint32_t timestamp_ms;  // Milliseconds since node boot
} pid_value_t;

