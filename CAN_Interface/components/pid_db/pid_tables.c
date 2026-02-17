/**
 * @file pid_tables.c
 * @brief Unified PID Database Tables
 * 
 * Contains all PID definitions for:
 * - Standard OBD-II Mode 0x01 PIDs (0x00-0xA4)
 * - GM Extended Mode 0x22 PIDs
 * 
 * Format: { pid, "name", type, unit, formula, {offset,mult,div}, bytes, service }
 * 
 * NOTE: GM PID addresses are UNVERIFIED and should be validated against
 * actual vehicle responses. Some may need correction.
 */

#include "pid_entry.h"
#include <stddef.h>

/* Shorthand macros for cleaner table format */
#define P(o, m, d)  { .offset = (o), .multiplier = (m), .divisor = (d) }
#define P0          { 0, 0, 0 }

/* ============================================================================
 * STANDARD OBD-II PIDs (Mode 0x01)
 * ============================================================================ */

static const pid_entry_t s_obd2_pids[] = {
    /* PID Number, Name,                                      Type,           Unit,      Formula,                     Parameters, Data Bytes,  Service */
    /* ---- Supported-PIDs bitmaps -------------------------------------------- */
    { 0x00, "Supported PIDs [01-20]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0x20, "Supported PIDs [21-40]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0x40, "Supported PIDs [41-60]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0x60, "Supported PIDs [61-80]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0x80, "Supported PIDs [81-A0]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0xA0, "Supported PIDs [A1-C0]",                  PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },

    /* ---- Engine Core (0x01-0x1F) ------------------------------------------- */
    { 0x01, "Monitor status since DTCs cleared",       PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              4, 0x01 },
    { 0x02, "Freeze frame DTC",                        PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              2, 0x01 },
    { 0x03, "Fuel system status",                      PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              2, 0x01 },
    { 0x04, "Calculated engine load",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x05, "Engine coolant temperature",              PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x01 },
    { 0x06, "Short term fuel trim - Bank 1",           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x01 },
    { 0x07, "Long term fuel trim - Bank 1",            PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x01 },
    { 0x08, "Short term fuel trim - Bank 2",           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x01 },
    { 0x09, "Long term fuel trim - Bank 2",            PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x01 },
    { 0x0A, "Fuel pressure",                           PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_A_TIMES_N,             P(0, 3, 0),      1, 0x01 },
    { 0x0B, "Intake manifold pressure",                PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x0C, "Engine RPM",                              PID_TYPE_FORMULA,  UNIT_RPM,     PID_FORMULA_AB_DIV_N,              P(0, 0, 4),      2, 0x01 },
    { 0x0D, "Vehicle speed",                           PID_TYPE_FORMULA,  UNIT_KMH,     PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x0E, "Timing advance",                          PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_HALF_MINUS_OFFSET,   P(64, 0, 0),     1, 0x01 },
    { 0x0F, "Intake air temperature",                  PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x01 },
    { 0x10, "MAF air flow rate",                       PID_TYPE_FORMULA,  UNIT_GS,      PID_FORMULA_AB_DIV_N,              P(0, 0, 100),    2, 0x01 },
    { 0x11, "Throttle position",                       PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x12, "Secondary air status",                    PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_ENUM_LOOKUP,           P0,              1, 0x01 },
    { 0x13, "O2 sensors present (2 banks)",            PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x14, "O2 Sensor 1 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x15, "O2 Sensor 2 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x16, "O2 Sensor 3 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x17, "O2 Sensor 4 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x18, "O2 Sensor 5 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x19, "O2 Sensor 6 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x1A, "O2 Sensor 7 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x1B, "O2 Sensor 8 Voltage",                     PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_A_DIV_200,             P0,              2, 0x01 },
    { 0x1C, "OBD standards compliance",                PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_ENUM_LOOKUP,           P0,              1, 0x01 },
    { 0x1D, "O2 sensors present (4 banks)",            PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x1E, "Auxiliary input status",                  PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x1F, "Run time since engine start",             PID_TYPE_FORMULA,  UNIT_SECONDS, PID_FORMULA_AB_RAW,                P0,              2, 0x01 },

    /* ---- Distance / MIL (0x21-0x3F) ---------------------------------------- */
    { 0x21, "Distance with MIL on",                    PID_TYPE_FORMULA,  UNIT_KM,      PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x22, "Fuel rail pressure (relative)",           PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_MULT_N,             P(0, 0.079f, 0), 2, 0x01 },
    { 0x23, "Fuel rail pressure (GDI)",                PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_MULT_N,             P(0, 10, 0),     2, 0x01 },
    { 0x24, "O2 Sensor 1 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x25, "O2 Sensor 2 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x26, "O2 Sensor 3 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x27, "O2 Sensor 4 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x28, "O2 Sensor 5 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x29, "O2 Sensor 6 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x2A, "O2 Sensor 7 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x2B, "O2 Sensor 8 EQ ratio",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x2C, "Commanded EGR",                           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x2D, "EGR error",                               PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x01 },
    { 0x2E, "Commanded evap purge",                    PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x2F, "Fuel tank level",                         PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x30, "Warm-ups since codes cleared",            PID_TYPE_FORMULA,  UNIT_COUNT,   PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x31, "Distance since codes cleared",            PID_TYPE_FORMULA,  UNIT_KM,      PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x32, "Evap vapor pressure",                     PID_TYPE_FORMULA,  UNIT_PA,      PID_FORMULA_AB_DIV_N,              P(0, 0, 4),      2, 0x01 },
    { 0x33, "Barometric pressure",                     PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_RAW_BYTE,              P0,              1, 0x01 },
    { 0x34, "O2 Sensor 1 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x35, "O2 Sensor 2 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x36, "O2 Sensor 3 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x37, "O2 Sensor 4 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x38, "O2 Sensor 5 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x39, "O2 Sensor 6 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x3A, "O2 Sensor 7 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x3B, "O2 Sensor 8 EQ / current",                PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_RATIO_32768,        P0,              4, 0x01 },
    { 0x3C, "Catalyst temp B1S1",                      PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_AB_DIV_N_MINUS_OFFSET, P(40, 0, 10),    2, 0x01 },
    { 0x3D, "Catalyst temp B2S1",                      PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_AB_DIV_N_MINUS_OFFSET, P(40, 0, 10),    2, 0x01 },
    { 0x3E, "Catalyst temp B1S2",                      PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_AB_DIV_N_MINUS_OFFSET, P(40, 0, 10),    2, 0x01 },
    { 0x3F, "Catalyst temp B2S2",                      PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_AB_DIV_N_MINUS_OFFSET, P(40, 0, 10),    2, 0x01 },

    /* ---- Control Module / Throttle (0x41-0x5F) ----------------------------- */
    { 0x41, "Monitor status this drive",               PID_TYPE_BITFIELD, UNIT_NONE,    PID_FORMULA_BITFIELD_4BYTE,        P0,              4, 0x01 },
    { 0x42, "Control module voltage",                  PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_DIV_N,              P(0, 0, 1000),   2, 0x01 },
    { 0x43, "Absolute load value",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_AB_MULT_N,             P(0, 0.392f, 0), 2, 0x01 },
    { 0x44, "Commanded EQ ratio",                      PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_DIV_N,              P(0, 0, 32768),  2, 0x01 },
    { 0x45, "Relative throttle position",              PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x46, "Ambient air temperature",                 PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x01 },
    { 0x47, "Throttle position B",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x48, "Throttle position C",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x49, "Accel pedal position D",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x4A, "Accel pedal position E",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x4B, "Throttle actuator cmd",                   PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x4C, "Time with MIL on",                        PID_TYPE_FORMULA,  UNIT_MINUTES, PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x4D, "Time since codes cleared",                PID_TYPE_FORMULA,  UNIT_MINUTES, PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x4E, "Time since codes cleared",                PID_TYPE_FORMULA,  UNIT_MINUTES, PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x4F, "Max EQ/O2V/O2C/MAP",                      PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              4, 0x01 },
    { 0x50, "Max MAF rate",                            PID_TYPE_FORMULA,  UNIT_GS,      PID_FORMULA_A_TIMES_N,             P(0, 10, 0),     4, 0x01 },
    { 0x51, "Fuel type",                               PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_ENUM_LOOKUP,           P0,              1, 0x01 },
    { 0x52, "Ethanol %",                               PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x53, "Abs evap vapor pressure",                 PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_DIV_N,              P(0, 0, 200),    2, 0x01 },
    { 0x54, "Evap vapor pressure",                     PID_TYPE_FORMULA,  UNIT_PA,      PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x55, "ST O2 trim B1B3",                         PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              2, 0x01 },
    { 0x56, "LT O2 trim B1B3",                         PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              2, 0x01 },
    { 0x57, "ST O2 trim B2B4",                         PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              2, 0x01 },
    { 0x58, "LT O2 trim B2B4",                         PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              2, 0x01 },
    { 0x59, "Fuel rail pressure (abs)",                PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_MULT_N,             P(0, 10, 0),     2, 0x01 },
    { 0x5A, "Rel accel pedal pos",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x5B, "Hybrid battery life",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x5C, "Engine oil temperature",                  PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x01 },
    { 0x5D, "Fuel injection timing",                   PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_AB_DIV_N_MINUS_OFFSET, P(210, 0, 128),  2, 0x01 },
    { 0x5E, "Engine fuel rate",                        PID_TYPE_FORMULA,  UNIT_LPH,     PID_FORMULA_AB_MULT_N,             P(0, 0.05f, 0),  2, 0x01 },
    { 0x5F, "Emission standard",                       PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_ENUM_LOOKUP,           P0,              1, 0x01 },

    /* ---- Torque (0x61-0x64) ------------------------------------------------ */
    { 0x61, "Driver demand torque %",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_MINUS_OFFSET,        P(125, 0, 0),    1, 0x01 },
    { 0x62, "Actual engine torque %",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_MINUS_OFFSET,        P(125, 0, 0),    1, 0x01 },
    { 0x63, "Engine reference torque",                 PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_AB_RAW,                P0,              2, 0x01 },
    { 0x64, "Engine torque data",                      PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              5, 0x01 },

    /* ---- Extended Vehicle Data (0x65-0xA4) --------------------------------- */
    /* These are mostly multi-byte status fields - decode as needed */
    { 0x65, "Aux I/O supported",                       PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              2, 0x01 },
    { 0x66, "MAF sensor",                              PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              5, 0x01 },
    { 0x67, "Engine coolant temp",                     PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              3, 0x01 },
    { 0x68, "Intake air temp sensor",                  PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              7, 0x01 },
    { 0x84, "Manifold surface temp",                   PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x01 },
    { 0x8D, "Throttle position G",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x01 },
    { 0x8E, "Engine friction torque %",                PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_MINUS_OFFSET,        P(125, 0, 0),    1, 0x01 },
    { 0xA2, "Cylinder fuel rate",                      PID_TYPE_FORMULA,  UNIT_MG_STROKE, PID_FORMULA_AB_MULT_N,           P(0, 0.005f, 0), 2, 0x01 },
    { 0xA4, "Trans gear ratio",                        PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_DIV_N,              P(0, 0, 1000),   2, 0x01 },
};

/* ============================================================================
 * GM EXTENDED PIDs (Mode 0x22)
 * 
 * WARNING: These addresses are from Arduino project and may need verification
 * against actual vehicle responses. Some may be incorrect or vehicle-specific.
 * ============================================================================ */

static const pid_entry_t s_gm_pids[] = {

    /* ---- Throttle / Airflow ------------------------------------------------ */
    { 0x083F, "Throttle Position",                     PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },
    { 0x0841, "Throttle Desired",                      PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },
    { 0x0843, "Accel Pedal Position",                  PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },
    { 0x0845, "Accel Pedal Position 2",                PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },
    { 0x0885, "MAF Frequency",                         PID_TYPE_FORMULA,  UNIT_COUNT,   PID_FORMULA_AB_RAW,                P0,              2, 0x22 },
    { 0x0886, "Volumetric Efficiency",                 PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_128,         P0,              1, 0x22 },
    { 0x0888, "MAF g/s",                               PID_TYPE_FORMULA,  UNIT_GS,      PID_FORMULA_AB_MULT_N,             P(0, 0.01f, 0),  2, 0x22 },
    { 0x088A, "Cylinder Airmass",                      PID_TYPE_FORMULA,  UNIT_MG_STROKE, PID_FORMULA_AB_MULT_N,           P(0, 0.01f, 0),  2, 0x22 },
    { 0x088C, "Dynamic Airflow",                       PID_TYPE_FORMULA,  UNIT_GS,      PID_FORMULA_AB_MULT_N,             P(0, 0.01f, 0),  2, 0x22 },
    { 0x08A2, "Idle Desired RPM",                      PID_TYPE_FORMULA,  UNIT_RPM,     PID_FORMULA_A_TIMES_40,            P(0, 40, 0),     1, 0x22 },

    /* ---- Spark / Timing ---------------------------------------------------- */
    { 0x09C4, "Spark Advance",                         PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09C6, "Spark Adv Cyl 1",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09C8, "Spark Adv Cyl 2",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09CA, "Spark Adv Cyl 3",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09CC, "Spark Adv Cyl 4",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09CE, "Spark Adv Cyl 5",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09D0, "Spark Adv Cyl 6",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09D2, "Spark Adv Cyl 7",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09D4, "Spark Adv Cyl 8",                       PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09D6, "Knock Retard Cyl 1-4",                  PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09D8, "Knock Retard Cyl 5-8",                  PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09DA, "Spark Coolant Corr",                    PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09DC, "Spark IAT Corr",                        PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },
    { 0x09DE, "Spark EGR Corr",                        PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_A_MINUS_64,            P(64, 0, 0),     1, 0x22 },

    /* ---- Torque ------------------------------------------------------------ */
    { 0x0A8B, "Engine Torque",                         PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0A8D, "Driver Demand Torque",                  PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0A8F, "Axle Torque Predicted",                 PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0A91, "Axle Torque Immediate",                 PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0A93, "Axle Torque Actual",                    PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0AB0, "Peak Torque",                           PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },
    { 0x0AB2, "Max Torque",                            PID_TYPE_FORMULA,  UNIT_NM,      PID_FORMULA_TORQUE_AB_DIV4,        P(0, 0, 4),      2, 0x22 },

    /* ---- Fuel / Injection -------------------------------------------------- */
    { 0x0B0B, "Injector PW Bank 1",                    PID_TYPE_FORMULA,  UNIT_MS,      PID_FORMULA_AB_MULT_N,             P(0, 0.01f, 0),  2, 0x22 },
    { 0x0B0D, "Injector PW Bank 2",                    PID_TYPE_FORMULA,  UNIT_MS,      PID_FORMULA_AB_MULT_N,             P(0, 0.01f, 0),  2, 0x22 },
    { 0x0B10, "Fuel Trim Cell",                        PID_TYPE_FORMULA,  UNIT_COUNT,   PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x0B12, "STFT Bank 1",                           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x22 },
    { 0x0B14, "STFT Bank 2",                           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x22 },
    { 0x0B16, "LTFT Bank 1",                           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x22 },
    { 0x0B18, "LTFT Bank 2",                           PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_SIGNED_PERCENT,      P0,              1, 0x22 },
    { 0x0B20, "Start of Injection",                    PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_AB_SIGNED_DIV_N,       P(0, 0, 100),    2, 0x22 },
    { 0x0B22, "End of Injection",                      PID_TYPE_FORMULA,  UNIT_DEGREE,  PID_FORMULA_AB_SIGNED_DIV_N,       P(0, 0, 100),    2, 0x22 },
    { 0x0B24, "EQ Ratio Commanded",                    PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_MULT_N,             P(0, 0.0001f, 0),2, 0x22 },
    { 0x0B26, "EQ Ratio Actual",                       PID_TYPE_FORMULA,  UNIT_RATIO,   PID_FORMULA_AB_MULT_N,             P(0, 0.0001f, 0),2, 0x22 },
    { 0x0B50, "Fuel Rail Pressure",                    PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_MULT_N,             P(0, 0.1f, 0),   2, 0x22 },
    { 0x0B60, "Fuel System Status",                    PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_STATUS,                P0,              1, 0x22 },

    /* ---- O2 Sensors -------------------------------------------------------- */
    { 0x0C00, "O2 B1S1 Voltage",                       PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_MULT_N,             P(0, 0.001f, 0), 2, 0x22 },
    { 0x0C02, "O2 B1S2 Voltage",                       PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_MULT_N,             P(0, 0.001f, 0), 2, 0x22 },
    { 0x0C04, "O2 B2S1 Voltage",                       PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_MULT_N,             P(0, 0.001f, 0), 2, 0x22 },
    { 0x0C06, "O2 B2S2 Voltage",                       PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_MULT_N,             P(0, 0.001f, 0), 2, 0x22 },

    /* ---- Engine State / Flags ---------------------------------------------- */
    { 0x1300, "PE Active",                             PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1302, "DFCO Active",                           PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1304, "COT Active",                            PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1306, "AC Clutch Request",                     PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1308, "Fan Relay Cmd",                         PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x130A, "MIL Status",                            PID_TYPE_STATUS,   UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },

    /* ---- Temperatures ------------------------------------------------------ */
    { 0x1340, "Coolant Temp 2",                        PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x22 },
    { 0x1342, "Intake Air Temp 2",                     PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x22 },
    { 0x1344, "Oil Temp",                              PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x22 },
    { 0x1360, "Oil Life %",                            PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },

    /* ---- Pressure / MAP ---------------------------------------------------- */
    { 0x1400, "MAP Sensor",                            PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_AB_MULT_N,             P(0, 0.1f, 0),   2, 0x22 },
    { 0x1402, "Barometric Pressure",                   PID_TYPE_FORMULA,  UNIT_KPA,     PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },

    /* ---- Transmission ------------------------------------------------------ */
    { 0x1A02, "Trans Fluid Temp",                      PID_TYPE_FORMULA,  UNIT_CELSIUS, PID_FORMULA_A_MINUS_OFFSET,        P(40, 0, 0),     1, 0x22 },
    { 0x1A04, "Current Gear",                          PID_TYPE_FORMULA,  UNIT_COUNT,   PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1A06, "Slip RPM",                              PID_TYPE_FORMULA,  UNIT_RPM,     PID_FORMULA_AB_SIGNED_DIV_N,       P(0, 0, 1),      2, 0x22 },
    { 0x1A08, "TCC Slip Speed",                        PID_TYPE_FORMULA,  UNIT_RPM,     PID_FORMULA_AB_SIGNED_DIV_N,       P(0, 0, 1),      2, 0x22 },
    { 0x1A0A, "TCC Duty Cycle",                        PID_TYPE_FORMULA,  UNIT_PERCENT, PID_FORMULA_A_PERCENT_255,         P0,              1, 0x22 },
    { 0x1A80, "Shift Mode",                            PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },
    { 0x1A82, "PRNDL Position",                        PID_TYPE_ENUM,     UNIT_NONE,    PID_FORMULA_RAW_BYTE,              P0,              1, 0x22 },

    /* ---- Misc -------------------------------------------------------------- */
    { 0x4268, "Battery Voltage",                       PID_TYPE_FORMULA,  UNIT_VOLT,    PID_FORMULA_AB_DIV_N,              P(0, 0, 100),    2, 0x22 },
};

#undef P
#undef P0

/* ============================================================================
 * Table Access Functions
 * ============================================================================ */

static const size_t s_obd2_pid_count = sizeof(s_obd2_pids) / sizeof(s_obd2_pids[0]);
static const size_t s_gm_pid_count = sizeof(s_gm_pids) / sizeof(s_gm_pids[0]);

const pid_entry_t *pid_db_get_obd2_table(void) { return s_obd2_pids; }
size_t pid_db_get_obd2_count(void) { return s_obd2_pid_count; }

const pid_entry_t *pid_db_get_gm_table(void) { return s_gm_pids; }
size_t pid_db_get_gm_count(void) { return s_gm_pid_count; }
