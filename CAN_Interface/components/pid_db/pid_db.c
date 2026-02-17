/**
 * @file pid_db.c
 * @brief PID Database Implementation
 */

#include "pid_db.h"
#include "system.h"
#include <string.h>

static const char *TAG = "pid_db";

// ============================================================================
// API Implementation
// ============================================================================

int pid_db_init(void)
{
    SYS_LOGI(TAG, "PID DB init: OBD2=%zu entries, GM=%zu entries",
             pid_db_get_obd2_count(), pid_db_get_gm_count());
    return 0;  // ESP_OK
}

const pid_entry_t *pid_db_lookup(uint16_t pid)
{
    // Search OBD-II table first
    const pid_entry_t *table = pid_db_get_obd2_table();
    size_t count = pid_db_get_obd2_count();
    
    for (size_t i = 0; i < count; i++) {
        if (table[i].pid == pid) {
            return &table[i];
        }
    }
    
    // Then GM extended
    table = pid_db_get_gm_table();
    count = pid_db_get_gm_count();
    
    for (size_t i = 0; i < count; i++) {
        if (table[i].pid == pid) {
            return &table[i];
        }
    }
    
    return NULL;
}

float pid_db_decode(const pid_entry_t *entry, const uint8_t *data, uint8_t len)
{
    if (!entry || !data || len == 0) {
        return 0.0f;
    }

    uint8_t A = data[0];
    uint8_t B = (len > 1) ? data[1] : 0;
    uint16_t AB = (A << 8) | B;
    float result = 0.0f;

    switch (entry->formula) {
        case PID_FORMULA_RAW_BYTE:
            result = (float)A;
            break;
            
        case PID_FORMULA_A_MINUS_OFFSET:
            result = (float)A - entry->param.offset;
            break;
            
        case PID_FORMULA_A_PERCENT_255:
            result = (float)A * 100.0f / 255.0f;
            break;
            
        case PID_FORMULA_A_PERCENT_128:
            result = (float)A * 100.0f / 128.0f;
            break;
            
        case PID_FORMULA_A_SIGNED_PERCENT:
            result = ((float)A - 128.0f) * 100.0f / 128.0f;
            break;
            
        case PID_FORMULA_A_TIMES_N:
            result = (float)A * entry->param.multiplier;
            break;
            
        case PID_FORMULA_A_DIV_200:
            result = (float)A / 200.0f;
            break;
            
        case PID_FORMULA_A_HALF_MINUS_OFFSET:
            result = (float)A / 2.0f - entry->param.offset;
            break;
            
        case PID_FORMULA_A_MINUS_64:
            result = (float)A - 64.0f;
            break;
            
        case PID_FORMULA_A_TIMES_40:
            result = (float)A * 40.0f;
            break;
            
        case PID_FORMULA_AB_RAW:
            result = (float)AB;
            break;
            
        case PID_FORMULA_AB_DIV_N:
            result = (entry->param.divisor != 0) ? 
                     (float)AB / entry->param.divisor : (float)AB;
            break;
            
        case PID_FORMULA_AB_SIGNED_DIV_N: {
            int16_t signed_AB = (int16_t)AB;
            result = (entry->param.divisor != 0) ? 
                     (float)signed_AB / entry->param.divisor : (float)signed_AB;
            break;
        }
            
        case PID_FORMULA_AB_MULT_N:
            result = (float)AB * entry->param.multiplier;
            break;
            
        case PID_FORMULA_AB_DIV_N_MINUS_OFFSET:
            result = (entry->param.divisor != 0) ?
                     (float)AB / entry->param.divisor - entry->param.offset :
                     (float)AB - entry->param.offset;
            break;
            
        case PID_FORMULA_AB_RATIO_32768:
            result = (float)AB / 32768.0f;
            break;
            
        case PID_FORMULA_TORQUE_AB_DIV4:
            result = (float)AB / 4.0f;
            break;
            
        case PID_FORMULA_BITFIELD_4BYTE:
        case PID_FORMULA_STATUS:
        case PID_FORMULA_ENUM_LOOKUP:
        case PID_FORMULA_STRING:
            // Non-numeric, return raw
            result = (float)AB;
            break;
            
        default:
            SYS_LOGW(TAG, "Unknown formula type: %d", entry->formula);
            result = (float)A;
            break;
    }
    
    SYS_LOGD(TAG, "Decode PID 0x%04X (%s): %.2f %s", 
             entry->pid, entry->name, result,
             entry->unit <= UNIT_MG_STROKE ? "" : "?");
    
    return result;
}

