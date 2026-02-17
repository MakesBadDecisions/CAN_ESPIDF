/**
 * @file pid_entry.h
 * @brief PID Database Entry Definitions
 * 
 * Defines the structure for PID decode entries and formula types.
 * All PID tables use these definitions.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Formula Types - How to decode raw OBD-II bytes
// ============================================================================

typedef enum {
    PID_FORMULA_RAW_BYTE,              // A (no conversion)
    PID_FORMULA_A_MINUS_OFFSET,        // A - offset
    PID_FORMULA_A_PERCENT_255,         // A * 100 / 255
    PID_FORMULA_A_PERCENT_128,         // A * 100 / 128
    PID_FORMULA_A_SIGNED_PERCENT,      // (A - 128) * 100 / 128
    PID_FORMULA_A_TIMES_N,             // A * multiplier
    PID_FORMULA_A_DIV_200,             // A / 200
    PID_FORMULA_A_HALF_MINUS_OFFSET,   // A / 2 - offset
    PID_FORMULA_A_MINUS_64,            // A - 64
    PID_FORMULA_A_TIMES_40,            // A * 40
    PID_FORMULA_AB_RAW,                // A * 256 + B
    PID_FORMULA_AB_DIV_N,              // (A * 256 + B) / divisor
    PID_FORMULA_AB_SIGNED_DIV_N,       // (signed)(A * 256 + B) / divisor
    PID_FORMULA_AB_MULT_N,             // (A * 256 + B) * multiplier
    PID_FORMULA_AB_DIV_N_MINUS_OFFSET, // (A * 256 + B) / divisor - offset
    PID_FORMULA_AB_RATIO_32768,        // (A * 256 + B) / 32768
    PID_FORMULA_TORQUE_AB_DIV4,        // (A * 256 + B) / 4
    PID_FORMULA_BITFIELD_4BYTE,        // 4-byte bitfield
    PID_FORMULA_STATUS,                // Special status byte handling
    PID_FORMULA_ENUM_LOOKUP,           // Enum table lookup
    PID_FORMULA_STRING,                // ASCII string decode
} pid_formula_t;

// ============================================================================
// PID Types - What kind of data the PID carries
// ============================================================================

typedef enum {
    PID_TYPE_FORMULA,   // Numeric value with formula decode
    PID_TYPE_ENUM,      // Discrete named states
    PID_TYPE_STATUS,    // Boolean/on-off status
    PID_TYPE_BITFIELD,  // Bit flags
    PID_TYPE_STRING,    // ASCII text
} pid_type_t;

// ============================================================================
// Units - Physical units of the decoded value
// ============================================================================

typedef enum {
    UNIT_NONE,
    UNIT_RPM,
    UNIT_KMH,
    UNIT_MPH,
    UNIT_CELSIUS,
    UNIT_FAHRENHEIT,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_KPA,
    UNIT_PA,
    UNIT_PSI,
    UNIT_GS,           // grams per second
    UNIT_DEGREE,
    UNIT_MA,
    UNIT_SECONDS,
    UNIT_MS,           // Milliseconds
    UNIT_MINUTES,
    UNIT_RATIO,
    UNIT_COUNT,
    UNIT_KM,
    UNIT_NM,           // Newton-meters (torque)
    UNIT_LPH,          // Liters per hour
    UNIT_MG_STROKE,    // Milligrams per stroke
} unit_t;

// ============================================================================
// Formula Parameters
// ============================================================================

typedef struct {
    float offset;
    float multiplier;
    float divisor;
} pid_param_t;

// ============================================================================
// PID Entry - One row in the PID lookup table
// ============================================================================

typedef struct {
    uint16_t      pid;          // PID code (0x00-0xFFFF)
    const char   *name;         // Human-readable name
    pid_type_t    type;         // Type of data
    unit_t        unit;         // Native unit
    pid_formula_t formula;      // Decode formula
    pid_param_t   param;        // Formula parameters
    uint8_t       data_bytes;   // Expected response bytes (1, 2, or 4)
    uint8_t       obd_service;  // OBD service (0x01, 0x22, etc.)
} pid_entry_t;

// ============================================================================
// PID Database API
// ============================================================================

/**
 * @brief Initialize PID database
 * @return ESP_OK on success
 */
int pid_db_init(void);

/**
 * @brief Look up a PID entry by number
 * @param pid PID number to find
 * @return Pointer to entry or NULL if not found
 */
const pid_entry_t* pid_db_lookup(uint16_t pid);

/**
 * @brief Decode raw OBD-II response bytes using entry's formula
 * @param entry PID entry from lookup
 * @param data Raw response bytes (typically A, B, C, D)
 * @param len Number of bytes in data
 * @return Decoded float value
 */
float pid_db_decode(const pid_entry_t *entry, const uint8_t *data, uint8_t len);

/**
 * @brief Get OBD-II Mode 0x01 PID table
 * @return Pointer to table array
 */
const pid_entry_t *pid_db_get_obd2_table(void);

/**
 * @brief Get OBD-II Mode 0x01 PID count
 */
size_t pid_db_get_obd2_count(void);

/**
 * @brief Get GM Mode 0x22 PID table
 * @return Pointer to table array
 * @note GM PIDs are unverified - validate against vehicle before use
 */
const pid_entry_t *pid_db_get_gm_table(void);

/**
 * @brief Get GM Mode 0x22 PID count
 */
size_t pid_db_get_gm_count(void);

