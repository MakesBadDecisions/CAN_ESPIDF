# diagnostics - DTC/UDS/Vehicle Info

## Purpose

Handles all diagnostic operations: reading and clearing Diagnostic Trouble Codes (DTCs), UDS extended diagnostics, and vehicle identification (VIN, CVN, ECU name).

## Capabilities

### DTC Reading
- **Mode 03** - Stored (confirmed) DTCs
- **Mode 07** - Pending DTCs
- **Mode 0x19** - UDS extended DTCs (3-byte codes with detailed status)
- **DTC format:** P/C/B/U prefix + 4 hex digits (e.g., P0101, C1234)

### DTC Clearing
- **Mode 04** - Clear all stored DTCs and reset MIL

### Vehicle Information (Mode 09)
- **Sub 0x02** - VIN (17 ASCII chars, ISO-TP multi-frame)
- **Sub 0x04** - CVN (4 bytes hex, calibration verification)
- **Sub 0x0A** - ECU Name (variable length ASCII)

### UDS Services (Mode 0x19)
- **Sub 0x02** - Report DTC by status mask
- 3-byte DTC codes with full status byte interpretation:
  - Test failed, test failed this cycle, pending, confirmed
  - Test not completed since clear, not completed this cycle
  - Warning indicator requested

## DTC Storage

```c
typedef struct {
    char     code[6];       // "P0101\0"
    char     type[10];      // "Stored", "Pending", "Permanent"
    uint8_t  status;        // Raw status byte (UDS)
    bool     mil_on;        // MIL indicator for this DTC
    int64_t  timestamp_us;  // When discovered
} dtc_record_t;

#define DTC_MAX_STORED 64
```

## Controller Scanner

Discovers ECUs on the bus using the MPVI-style scan sequence from the Arduino project:

1. **Ping controllers** - Send to 0x7E0-0x7E7, record which respond
2. **Scan supported PIDs** - Request PID ranges 0x00, 0x20, 0x40, 0x60, 0x80, 0xA0
3. **Request VIN** - Mode 09, Sub 0x02
4. **Request CVN** - Mode 09, Sub 0x04
5. **Request ECU Name** - Mode 09, Sub 0x0A

Results are stored and available for the comm_link to send to the display node.

## Files

```
diagnostics/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── diagnostics.h        # Public API
│   ├── dtc_manager.h        # DTC storage and management
│   ├── uds.h                # UDS protocol types and helpers
│   └── vehicle_info.h       # VIN/CVN/ECU name storage
└── src/
    ├── diagnostics.c         # Top-level diagnostic coordinator
    ├── dtc_manager.c         # DTC storage, format, status decode
    ├── uds.c                 # UDS response parsing
    ├── vehicle_info.c        # VIN/CVN/ECU retrieval and storage
    └── controller_scanner.c  # ECU discovery and PID range scanning
```

## Dependencies

- `obd2` (sending diagnostic requests, receiving responses)
- `can_driver` (bus access via obd2)
- `system` (logging, timing)
