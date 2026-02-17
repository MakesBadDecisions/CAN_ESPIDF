# pid_db - PID Database & Unit Conversions

## Purpose

Contains the complete PID decode table and unit conversion system. This is the core data asset of the project, migrated from the verified Arduino codebase and restructured as pure C const data for ESP-IDF.

## What Changed From Arduino

The Arduino version used `std::function<>` lambda decoders for each PID entry. Each lambda allocated heap memory. With 165 entries, this consumed significant RAM.

The ESP-IDF version uses a **formula-type enum** with parameters. The entire table lives in flash (ROM) as `const` data - zero heap usage.

### Arduino (old)
```cpp
{ 0x05, "Engine Coolant Temp", PIDType::FORMULA, Unit::CELSIUS,
  [](const uint8_t* d) { return DecodedValue{d[0] - 40.0f, "C"}; }
}
```

### ESP-IDF (new)
```c
{ .pid = 0x05, .name = "Engine Coolant Temp",
  .type = PID_TYPE_FORMULA, .unit = UNIT_CELSIUS,
  .formula = PID_FORMULA_A_MINUS_OFFSET, .param = {.offset = 40.0f}
}
```

## Formula Types

Most OBD-II PIDs use a small set of decode formulas. Instead of unique functions, we enumerate them:

| Formula Type | Formula | Example PIDs |
|-------------|---------|-------------|
| `A_MINUS_OFFSET` | `A - offset` | 0x05 (Coolant), 0x0F (IAT), 0x46 (Ambient) |
| `A_TIMES_N` | `A * multiplier` | 0x0A (Fuel Pressure: A*3) |
| `A_PERCENT_255` | `A * 100 / 255` | 0x04 (Load), 0x11 (Throttle), 0x2F (Fuel Level) |
| `A_SIGNED_PERCENT` | `(A-128) * 100 / 128` | 0x06-0x09 (Fuel Trims) |
| `AB_DIV_N` | `(A*256+B) / divisor` | 0x0C (RPM: /4), 0x10 (MAF: /100) |
| `AB_RAW` | `A*256 + B` | 0x0D (Speed), 0x1F (Runtime), 0x21 (Distance) |
| `AB_MULT_N` | `(A*256+B) * multiplier` | 0x22 (Rail Pres: *0.079), 0x5E (Fuel Rate: *0.05) |
| `AB_DIV_N_MINUS_OFFSET` | `(A*256+B)/div - offset` | 0x3C (Cat Temp: /10-40), 0x5D (Inj Timing: /128-210) |
| `AB_RATIO_32768` | `(A*256+B) / 32768` | 0x24-0x2B (O2 EQ Ratios) |
| `A_DIV_200` | `A / 200` | 0x14-0x1B (O2 Voltages) |
| `A_HALF_MINUS_OFFSET` | `A/2 - offset` | 0x0E (Timing Advance: /2-64) |
| `BITFIELD_4BYTE` | `(A<<24)\|(B<<16)\|(C<<8)\|D` | 0x00, 0x20, 0x40 (Supported PIDs) |
| `RAW_BYTE` | `A` (no conversion) | 0x1C (OBD Standard), 0x30 (Warm-ups) |
| `TORQUE_AB_DIV4` | `(A*256+B) / 4` | GM torque PIDs (0xA91, 0xAB0, etc.) |
| `A_MINUS_64` | `A - 64` | GM spark advance PIDs (0x9CE-0x9DE) |
| `A_TIMES_40` | `A * 40` | 0x8A2 (Idle Desired RPM) |
| `STATUS` | Special handling | 0x01 (Monitor Status), 0x03 (Fuel Sys) |
| `STRING` | ASCII decode | VIN, ECU Name |
| `ENUM_LOOKUP` | Enum table index | 0x51 (Fuel Type), 0x1C (OBD Standard) |

## PID Entry Structure

```c
typedef struct {
    uint16_t         pid;           // PID code (0x00-0xFFFF)
    const char      *name;          // Human-readable name
    pid_type_t       type;          // FORMULA, ENUM, STATUS, BITFIELD, STRING
    unit_t           unit;          // Native unit (before conversion)
    pid_formula_t    formula;       // Decode formula type
    pid_param_t      param;         // Formula parameters (offset, divisor, multiplier)
    uint8_t          data_bytes;    // Expected response data bytes (1, 2, or 4)
} pid_entry_t;
```

## PID Coverage

### Standard OBD-II (Service 0x01) - 97 entries
PIDs 0x00 through 0xA4 covering:
- Engine: RPM, load, coolant temp, timing, MAF, throttle
- Fuel: trims (short/long, bank 1/2), pressure, rate, level, type
- O2 Sensors: voltage (8 sensors), EQ ratio (8 sensors), current (8 sensors)
- Emissions: EGR, EVAP purge, catalyst temps, DPF, NOx, SCR, DEF
- Turbo: speed, boost pressure, wastegate, charge air temp, EGT
- Diagnostics: MIL status, DTC count, supported PIDs bitmaps
- Vehicle: speed, distance, runtime, barometric pressure, ambient temp

### GM Extended (Service 0x22) - 68 entries
PIDs 0x83F through 0x4268 covering:
- Throttle: position, desired position, desired area, desired MAP
- Airflow: MAF hi-res, volumetric efficiency, cylinder airmass, dynamic airflow
- Spark: base advance, coolant/EGR/IAT corrections, torque management, knock retard
- Torque: engine, axle (predicted/immediate/actual), driver demand, peak/max
- Fuel: injector pulse width (bank 1/2), SOI/EOI, trim cell, rail pressure
- Transmission: fluid temp, current gear, slip RPM, TCC slip/pressure, shift mode
- State: PE active, DFCO active, COT active, PRNDL, shift mode, fuel system status

## Unit Conversion System

Unit conversions are handled by a separate utility within this component. Conversions are applied AFTER decode, on the float result.

| Category | Conversions |
|----------|------------|
| Temperature | C to F: `v*9/5+32`, C to K: `v+273.15` |
| Speed | km/h to mph: `v*0.621371`, km/h to knots: `v*0.539957` |
| Pressure | kPa to PSI: `v*0.145038`, kPa to bar: `v*0.01`, kPa to Pa: `v*1000` |
| Mass flow | g/s to lb/hr: `v*7.936641` |
| Voltage | V to mV: `v*1000` |
| Volume flow | L/h to gal/h: `v*0.264172` |
| Distance | km to miles: `v*0.621371` |
| Time | s to min: `v/60`, s to hr: `v/3600` |
| Torque | Nm to ft-lb: `v*0.737562`, Nm to in-lb: `v*8.85075` |

## Poll Configuration

Each PID has a default polling configuration with:
- **Interval (ms):** How often to request (50ms - 12000ms)
- **Priority (1-5):** 1 = highest (RPM, torque), 5 = lowest (baro, misfires)

These are stored as a separate const table and can be overridden at runtime via NVS config.

## Files

```
pid_db/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── pid_db.h             # Public API: pid_db_lookup(), pid_db_decode()
│   ├── pid_entry.h          # PID entry struct and formula types
│   ├── pid_units.h          # Unit enum and conversion functions
│   └── pid_poll_config.h    # Default poll intervals and priorities
└── src/
    ├── pid_table_obd2.c     # Standard OBD-II PID table (const)
    ├── pid_table_gm.c       # GM extended PID table (const)
    ├── pid_decode.c          # Formula-based decode dispatcher
    ├── pid_units.c           # Unit conversion implementations
    └── pid_poll_defaults.c   # Default poll config table (const)
```

## Dependencies

- `shared/pid_types` (shared PID value types for inter-node transport)

## Usage

```c
// Look up a PID entry
const pid_entry_t *entry = pid_db_lookup(0x0C);  // Engine RPM

// Decode raw OBD-II response data
float value = pid_db_decode(entry, response_data, data_len);

// Convert units
float mph = pid_unit_convert(value, UNIT_KMH, UNIT_MPH);
```
