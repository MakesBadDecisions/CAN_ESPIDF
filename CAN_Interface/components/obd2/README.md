# obd2 - OBD-II Protocol Stack

## Purpose

Implements the OBD-II communication protocol on top of the CAN driver HAL. Handles request building, response parsing, ISO-TP multi-frame transport, and flow control.

This component does NOT know about specific PIDs or their meanings - it deals only with the transport layer. PID decoding is handled by `pid_db`.

## Architecture

```
┌─────────────────────────────────────────┐
│     poll_engine / diagnostics           │
│  "Send Mode 01 PID 0x0C on bus 0"      │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│           obd2 API                      │
│  - obd2_request_pid()                   │
│  - obd2_request_dtc()                   │
│  - obd2_request_vehicle_info()          │
│  - obd2_process_response()              │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│        ISO-TP Transport                 │
│  - Single frame (<= 7 bytes)           │
│  - Multi-frame (First + Consecutive)    │
│  - Flow control (CTS, Wait, Overflow)   │
└──────────────┬──────────────────────────┘
               │
┌──────────────┴──────────────────────────┐
│        can_driver HAL                   │
│  - can_driver_send()                    │
│  - can_driver_receive()                 │
└─────────────────────────────────────────┘
```

## OBD-II Services Supported

| Service | Name | Request | Response |
|---------|------|---------|----------|
| 0x01 | Current Data | `[02][01][PID]` | `[03+][41][PID][data...]` |
| 0x02 | Freeze Frame | `[02][02][PID]` | `[03+][42][PID][data...]` |
| 0x03 | Stored DTCs | `[01][03]` | `[N*2+1][43][DTC_H][DTC_L]...` |
| 0x04 | Clear DTCs | `[01][04]` | `[01][44]` |
| 0x07 | Pending DTCs | `[01][07]` | `[N*2+1][47][DTC_H][DTC_L]...` |
| 0x09 | Vehicle Info | `[02][09][InfoType]` | ISO-TP multi-frame |
| 0x0A | Permanent DTCs | `[01][0A]` | `[N*2+1][4A][DTC_H][DTC_L]...` |
| 0x19 | UDS Read DTC | `[03][19][SubFunc][Mask]` | ISO-TP multi-frame |
| 0x22 | Read Data By ID | `[03][22][DID_H][DID_L]` | `[03+][62][DID_H][DID_L][data...]` |

## CAN ID Conventions

| Purpose | CAN ID | Notes |
|---------|--------|-------|
| OBD-II Broadcast Request | 0x7DF | All ECUs listen |
| ECU-Specific Request | 0x7E0-0x7E7 | Specific controller |
| ECU Response | 0x7E8-0x7EF | Response ID = Request ID + 8 |

## ISO-TP Transport

For responses larger than 7 bytes (VIN, DTC lists, multi-byte data), ISO-TP (ISO 15765-2) handles segmentation:

### Frame Types

| PCI Type | Byte 0 | Structure | Max Data |
|----------|--------|-----------|----------|
| Single Frame | `0x0N` | `[0N][data...]` | 7 bytes |
| First Frame | `0x1N` | `[1N][LL][data...]` | 6 bytes (+ total length) |
| Consecutive | `0x2N` | `[2N][data...]` | 7 bytes per frame |
| Flow Control | `0x3S` | `[3S][BS][STmin]` | Control only |

### Multi-Frame Sequence

```
ECU Request:  [02][09][02]                    → Request VIN
ECU Response: [10][14][49][02][01][57][41][55] → First Frame (20 bytes total)
Host:         [30][00][00]                     → Flow Control (Continue To Send)
ECU Response: [21][4C][44][42][38][57][5A][55] → Consecutive Frame 1
ECU Response: [22][33][48][5A][35][37][38][39] → Consecutive Frame 2
ECU Response: [23][30][00][00][00][00][00][00] → Consecutive Frame 3 (padded)
```

## Response Callback

The OBD-II stack delivers parsed responses via callback:

```c
typedef struct {
    uint8_t  service;       // Response service (0x41, 0x43, 0x49, 0x62, etc.)
    uint16_t pid;           // PID or DID that was requested
    uint8_t  data[256];     // Response payload (after ISO-TP reassembly)
    uint16_t data_len;      // Payload length
    uint32_t controller_id; // Responding ECU CAN ID
    int64_t  timestamp_us;  // Response timestamp
} obd2_response_t;

typedef void (*obd2_response_cb_t)(const obd2_response_t *response, void *user_data);
```

## Pending Request Tracking

The stack tracks outstanding requests to match responses:

```c
typedef struct {
    uint8_t  service;
    uint16_t pid;
    uint32_t target_id;     // 0x7DF for broadcast, 0x7E0-0x7E7 for specific
    int64_t  sent_time_us;
    bool     active;
} obd2_pending_request_t;
```

Timeout: 100ms default, configurable. Unmatched responses are discarded or forwarded to a sniffer callback.

## Files

```
obd2/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── obd2.h               # Public API
│   ├── obd2_types.h         # Request/response types, service constants
│   └── isotp.h              # ISO-TP transport interface
└── src/
    ├── obd2.c                # Request builder, response dispatcher
    ├── obd2_response.c       # Response parser (single-frame OBD-II)
    ├── isotp.c               # ISO-TP multi-frame state machine
    └── isotp_flow_control.c  # Flow control handling
```

## Dependencies

- `can_driver` (frame send/receive)
- `system` (logging, timing)
