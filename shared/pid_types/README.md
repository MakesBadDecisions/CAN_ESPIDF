# pid_types -- PID Data Type Definitions

## Overview

This component defines the data types used to represent decoded OBD-II
Parameter ID (PID) values throughout the CAN_ESPIDF system. The CAN Interface
node decodes raw CAN frames into these types; the Display node receives them
and uses the same types to interpret and render the data. Both nodes link
against this component so they always agree on layouts, enum values, and
struct packing.

---

## Enums

### PIDFormulaType

Describes how raw OBD-II response bytes (conventionally labeled A, B, C, D)
are converted into a meaningful value. The CAN Interface node selects the
appropriate formula when decoding; the Display node may reference the formula
type for formatting or range-checking.

| Enumerator              | Description                                      | Typical Example PID        |
|-------------------------|--------------------------------------------------|----------------------------|
| `A_MINUS_40`            | `A - 40`                                         | Coolant temp (0x05)        |
| `AB_DIV_N`              | `((A * 256) + B) / N` for a given divisor N      | RPM (0x0C, N=4)           |
| `A_TIMES_100_DIV_255`   | `A * 100 / 255`                                  | Throttle position (0x11)   |
| `A_DIV_2`               | `A / 2`                                          | Vehicle speed (0x0D) (*)   |
| `A_MINUS_128`           | `A - 128`                                        | Fuel trim (0x06, 0x07)     |
| `AB_MINUS_OFFSET`       | `((A * 256) + B) - offset`                       | Timing advance variants    |
| `A_DIV_N`               | `A / N`                                          | Various percentage PIDs    |
| `AB_TIMES_N`            | `((A * 256) + B) * N`                            | MAF rate (0x10)            |
| `BITFIELD_1BYTE`        | Interpret A as a 1-byte bitfield                 | Status flags               |
| `BITFIELD_2BYTE`        | Interpret A,B as a 2-byte bitfield               | Readiness monitors         |
| `BITFIELD_4BYTE`        | Interpret A,B,C,D as a 4-byte bitfield           | Extended status             |
| `CUSTOM`                | Non-standard formula; requires dedicated handler  | Manufacturer-specific PIDs |
| `IDENTITY`              | Value = A (no transformation)                    | Simple status bytes        |

(*) Some references use `A` directly for speed; the divisor depends on the
specific PID definition table in use.

### Unit (pid_unit_t)

Physical or logical unit of the decoded value. Used by both nodes for display
labels, formatting, and unit conversions. The canonical definition is the
`pid_unit_t` enum in `pid_types.h`. The CAN Interface node's `pid_entry.h`
provides backward-compatible `unit_t` typedef and `UNIT_*` macros.

| Enumerator       | Meaning                        |
|------------------|--------------------------------|
| `PID_UNIT_NONE`  | No unit / not applicable       |
| `PID_UNIT_RPM`   | Revolutions per minute         |
| `PID_UNIT_KMH`   | Kilometers per hour            |
| `PID_UNIT_MPH`   | Miles per hour                 |
| `PID_UNIT_CELSIUS`| Degrees Celsius               |
| `PID_UNIT_FAHRENHEIT`| Degrees Fahrenheit         |
| `PID_UNIT_PERCENT`| Percentage (0--100)           |
| `PID_UNIT_VOLT`  | Volts                          |
| `PID_UNIT_KPA`   | Kilopascals                    |
| `PID_UNIT_PSI`   | Pounds per square inch         |
| `PID_UNIT_PA`    | Pascals                        |
| `PID_UNIT_GS`    | Grams per second               |
| `PID_UNIT_DEGREE`| Degrees (e.g., timing advance) |
| `PID_UNIT_MA`    | Milliamps                      |
| `PID_UNIT_SECONDS`| Seconds                       |
| `PID_UNIT_MS`    | Milliseconds                   |
| `PID_UNIT_MINUTES`| Minutes                       |
| `PID_UNIT_KM`    | Kilometers                     |
| `PID_UNIT_RATIO` | Dimensionless ratio            |
| `PID_UNIT_COUNT` | Unitless count                 |
| `PID_UNIT_NM`    | Newton-meters (torque)         |
| `PID_UNIT_LPH`   | Liters per hour               |
| `PID_UNIT_MG_STROKE`| Milligrams per stroke       |

### Unit Conversion API

The shared `pid_types.c` provides runtime unit conversion:

- **`pid_unit_str(unit)`** -- returns display string (e.g., `"°C"`, `"rpm"`, `"psi"`)
- **`pid_unit_convert(value, from, to, &result)`** -- bidirectional conversion
  between compatible units (C↔F, km/h↔mph, kPa↔PSI, Pa↔kPa)
- **`pid_unit_get_alts(base, alts[], max)`** -- returns convertible alternatives
  for a given base unit (e.g., `PID_UNIT_CELSIUS` → `PID_UNIT_FAHRENHEIT`)

The Display node uses these to populate the unit dropdown and convert gauge
values on-the-fly without needing the CAN Interface node to re-send data.

### PIDType

Categorizes the kind of data a PID carries. Determines how the value should
be displayed and, in some cases, how it should be decoded.

| Enumerator  | Description                                              |
|-------------|----------------------------------------------------------|
| `FORMULA`   | Numeric value obtained by applying a `PIDFormulaType`    |
| `ENUM`      | Value maps to a discrete set of named states             |
| `STATUS`    | Boolean or on/off status indicator                       |
| `BITFIELD`  | One or more bit flags packed into the value               |
| `STRING`    | ASCII or UTF-8 text (e.g., VIN segments)                 |

---

## Structs

### PIDValue

The canonical representation of a single decoded PID reading. This is the
structure that travels over the UART link (inside a `comm_protocol` message
payload) and is consumed by the Display node.

```c
typedef struct __attribute__((packed)) {
    uint16_t pid_id;      // OBD-II PID number (e.g., 0x0C for RPM)
    float    value;        // Decoded numeric value
    uint8_t  unit;         // Unit enum value
    uint32_t timestamp;    // Milliseconds since node boot (from esp_timer)
} PIDValue;
```

**Size:** 11 bytes (packed).

| Field       | Type       | Notes                                            |
|-------------|------------|--------------------------------------------------|
| `pid_id`    | `uint16_t` | Supports standard (Mode 01) and extended PIDs    |
| `value`     | `float`    | IEEE 754 single-precision; sufficient for all OBD-II numeric ranges |
| `unit`      | `uint8_t`  | Cast from `Unit` enum                            |
| `timestamp` | `uint32_t` | Wraps after ~49 days; adequate for runtime use   |

For non-numeric PID types (`ENUM`, `STATUS`, `BITFIELD`), the `value` field
holds the raw integer value cast to `float`. For `STRING` type PIDs, the data
is carried in a separate payload field rather than in `PIDValue.value`.

---

## Why These Types Are Shared

The CAN Interface node populates `PIDValue` structs after decoding raw CAN
frames. The Display node unpacks the same structs from UART messages. If
either side used a different struct layout, enum ordering, or packing, the
data would be silently corrupt. Sharing a single definition prevents this
class of bug entirely.
