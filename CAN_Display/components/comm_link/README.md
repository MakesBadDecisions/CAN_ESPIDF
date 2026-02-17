# comm_link - UART Communication (Display Node RX)

## Purpose

Receives real-time PID data and vehicle information from the CAN interface
node via UART over a USB-C cable. Parses incoming messages, maintains a local
PID value store, monitors connection health, and sends configuration commands
back to the CAN node.

## Responsibilities

- **Receive UART frames** from CAN interface node (2 Mbps, DMA-backed)
- **Parse message types** (PID batches, vehicle info, DTCs, heartbeat)
- **Maintain PID value store** - latest decoded value for each active PID
- **Connection monitoring** - heartbeat timeout detection, link status
- **Send commands** - poll list changes, log start/stop, scan requests back to CAN node
- **Proxy config** - relay user configuration from WiFi web UI to CAN node over UART

## PID Value Store

A flat array indexed by PID ID holding the latest value from the CAN node:

```c
typedef struct {
    uint16_t pid;
    float    value;
    unit_t   unit;
    int64_t  last_update_us;
    bool     valid;
} pid_value_entry_t;

// Accessed by gauge_engine (read) and comm_link (write)
// Protected by mutex for thread safety
```

The gauge engine and data logger read from this store. The comm_link task is
the only writer.

## UART Frame Format

Each message uses the framing defined in `shared/comm_protocol/`:

```
[0xAA] [Header 8B] [Payload 0..N bytes] [CRC16 2B]
```

The UART RX handler uses DMA to receive bytes into a ring buffer. A parser
task scans for the `0xAA` start byte, validates the CRC, and dispatches the
message by type.

## Files

```
comm_link/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── comm_link.h          # Public API
│   └── pid_store.h          # PID value store API
└── src/
    ├── comm_link.c           # UART init, receive handler, dispatch
    ├── comm_link_tx.c        # Outgoing commands to CAN node
    └── pid_store.c           # Thread-safe PID value storage
```

## Dependencies

- `driver` (ESP-IDF UART driver with DMA)
- `shared/comm_protocol` (message format)
- `shared/pid_types` (PID value types)
- `system` (logging, NVS)
