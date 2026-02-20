/**
 * @file pid_types.c
 * @brief PID Data Types Implementation — unit strings, conversions
 */

#include "pid_types.h"
#include <stddef.h>

// ============================================================================
// Unit Display Strings
// ============================================================================

const char *pid_unit_str(pid_unit_t unit)
{
    switch (unit) {
        case PID_UNIT_RPM:        return "RPM";
        case PID_UNIT_KMH:        return "km/h";
        case PID_UNIT_MPH:        return "mph";
        case PID_UNIT_CELSIUS:    return "\xC2\xB0""C";   // °C
        case PID_UNIT_FAHRENHEIT: return "\xC2\xB0""F";   // °F
        case PID_UNIT_PERCENT:    return "%";
        case PID_UNIT_VOLT:       return "V";
        case PID_UNIT_KPA:        return "kPa";
        case PID_UNIT_PA:         return "Pa";
        case PID_UNIT_PSI:        return "psi";
        case PID_UNIT_GS:         return "g/s";
        case PID_UNIT_DEGREE:     return "\xC2\xB0";      // °
        case PID_UNIT_MA:         return "mA";
        case PID_UNIT_SECONDS:    return "s";
        case PID_UNIT_MS:         return "ms";
        case PID_UNIT_MINUTES:    return "min";
        case PID_UNIT_RATIO:      return ":1";
        case PID_UNIT_COUNT:      return "";
        case PID_UNIT_KM:         return "km";
        case PID_UNIT_NM:         return "Nm";
        case PID_UNIT_LPH:        return "L/h";
        case PID_UNIT_MG_STROKE:  return "mg/st";
        case PID_UNIT_NONE:
        default:                  return "";
    }
}

// ============================================================================
// Unit Conversion Pairs
// ============================================================================

static const pid_unit_pair_t s_conversions[] = {
    { PID_UNIT_CELSIUS,  PID_UNIT_FAHRENHEIT },
    { PID_UNIT_KMH,      PID_UNIT_MPH        },
    { PID_UNIT_KPA,      PID_UNIT_PSI        },
    { PID_UNIT_KM,       PID_UNIT_NONE       },  // placeholder — miles not yet in enum
    { PID_UNIT_PA,       PID_UNIT_KPA        },
};
static const int s_conversion_count = sizeof(s_conversions) / sizeof(s_conversions[0]);

bool pid_unit_convert(float value, pid_unit_t from, pid_unit_t to, float *out)
{
    if (from == to) {
        *out = value;
        return true;
    }

    // C -> F
    if (from == PID_UNIT_CELSIUS && to == PID_UNIT_FAHRENHEIT) {
        *out = value * 9.0f / 5.0f + 32.0f;
        return true;
    }
    // F -> C
    if (from == PID_UNIT_FAHRENHEIT && to == PID_UNIT_CELSIUS) {
        *out = (value - 32.0f) * 5.0f / 9.0f;
        return true;
    }
    // km/h -> mph
    if (from == PID_UNIT_KMH && to == PID_UNIT_MPH) {
        *out = value * 0.621371f;
        return true;
    }
    // mph -> km/h
    if (from == PID_UNIT_MPH && to == PID_UNIT_KMH) {
        *out = value / 0.621371f;
        return true;
    }
    // kPa -> PSI
    if (from == PID_UNIT_KPA && to == PID_UNIT_PSI) {
        *out = value * 0.145038f;
        return true;
    }
    // PSI -> kPa
    if (from == PID_UNIT_PSI && to == PID_UNIT_KPA) {
        *out = value / 0.145038f;
        return true;
    }
    // Pa -> kPa
    if (from == PID_UNIT_PA && to == PID_UNIT_KPA) {
        *out = value / 1000.0f;
        return true;
    }
    // kPa -> Pa
    if (from == PID_UNIT_KPA && to == PID_UNIT_PA) {
        *out = value * 1000.0f;
        return true;
    }

    return false;  // unsupported pair
}

int pid_unit_get_alts(pid_unit_t base, pid_unit_t *alts, int max_alts)
{
    int count = 0;
    for (int i = 0; i < s_conversion_count && count < max_alts; i++) {
        if (s_conversions[i].base == base) {
            alts[count++] = s_conversions[i].alt;
        } else if (s_conversions[i].alt == base) {
            alts[count++] = s_conversions[i].base;
        }
    }
    return count;
}

