# CAN_ESPIDF Shared Components

## Purpose

The `shared/` directory contains code that is common to **both** ESP32-S3 nodes
in the CAN_ESPIDF system:

- **CAN Interface Node** -- reads OBD-II PID data from the vehicle CAN bus and
  transmits it over UART to the Display Node via USB-C cable.
- **Display Node** -- receives PID data over UART and renders it on screen.

Both nodes must agree on data representations, message formats, and utility
behavior. Duplicating these definitions would invite version drift and subtle
bugs. Keeping a single source of truth in `shared/` eliminates that risk.

---

## How Shared Code Is Included

Each node's `CMakeLists.txt` adds the shared directory via ESP-IDF's
`EXTRA_COMPONENT_DIRS` mechanism:

```cmake
# In CAN_Interface/CMakeLists.txt or CAN_Display/CMakeLists.txt
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)
```

Every subdirectory under `shared/` that contains a `CMakeLists.txt` with an
`idf_component_register(...)` call is automatically discovered as a component.
The node then declares a dependency on the shared component(s) it needs in its
own component's `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES pid_types comm_protocol utils
)
```

No copy step is needed. Both nodes reference the same physical files.

---

## Sub-Modules

### pid_types

Data type definitions for OBD-II Parameter IDs (PIDs). Includes enums for
formula types, measurement units (`pid_unit_t` with 22 unit values), PID
categories, and the `pid_value_t` struct that is the canonical representation
of a decoded PID reading. Also provides a shared unit conversion API:
`pid_unit_str()` for display strings, `pid_unit_convert()` for bidirectional
conversions (°C↔°F, km/h↔mph, kPa↔PSI, Pa↔kPa), and `pid_unit_get_alts()`
for discovering convertible alternatives. Both nodes link against this
component so they always agree on layouts, enum values, and conversions.

See [`pid_types/README.md`](pid_types/README.md).

### comm_protocol

Inter-node message format definitions. Covers the message header layout, all
message type codes, and the payload structures for each message type. Both
nodes serialize and deserialize these structures over the UART link, so the
definitions must be identical on each side.

See [`comm_protocol/README.md`](comm_protocol/README.md).

### utils

Common utility functions: CRC calculations for message integrity, byte
packing/unpacking helpers for portable serialization, and timestamp utilities
for consistent timekeeping across nodes.

See [`utils/README.md`](utils/README.md).
