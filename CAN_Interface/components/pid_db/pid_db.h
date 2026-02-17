/**
 * @file pid_db.h
 * @brief PID Database API
 * 
 * Lookup and management of PID definitions. Supports multi-source
 * tables (OBD-II standard, GM extended, etc.).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "pid_entry.h"

// ============================================================================
// Table Access Functions (from pid_table_*.c)
// ============================================================================

const pid_entry_t *pid_db_get_obd2_table(void);
size_t pid_db_get_obd2_count(void);

const pid_entry_t *pid_db_get_gm_table(void);
size_t pid_db_get_gm_count(void);

