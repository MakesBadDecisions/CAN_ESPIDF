# comm_link - UART Communication (Display Node)

## Purpose

Receives real-time PID data and vehicle information from the CAN Interface
Node via UART over a USB-C cable. Parses incoming messages, maintains a local
PID value store, monitors connection health, and sends configuration commands
back to the CAN node.

## Responsibilities

- **Receive UART frames** from CAN Interface Node (2 Mbps, DMA-backed)
- **Parse message types** (PID batches, vehicle info, DTCs, heartbeat)
- **Maintain PID value store** - latest decoded value for each active PID
- **Connection monitoring** - heartbeat timeout detection, link status
- **Send commands** - poll list changes, log start/stop, scan requests back to CAN node

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
```

The gauge engine and data logger read from this store. The comm_link task is
the only writer. Protected by mutex for thread safety.

## UART Frame Format

Each message uses the framing defined in `shared/comm_protocol/`:

```
[0xAA] [Header 8B] [Payload 0..N bytes] [CRC16 2B]
```

The UART RX handler uses DMA to receive bytes into a ring buffer. A parser
task scans for the `0xAA` start byte, validates the CRC, and dispatches the
message by type.

## Connection Health

- TX heartbeat every 500ms
- RX heartbeat monitor with 2-second timeout
- `comm_link_get_state()` returns `COMM_LINK_CONNECTED` or `COMM_LINK_DISCONNECTED`
- `comm_link_get_stats()` returns frame counters (rx, tx, pid updates, errors)
- `comm_link_get_can_status()` returns remote CAN bus state from heartbeat (0=off, 1=running, 2=error)

## Files

```
comm_link/
├── README.md
├── CMakeLists.txt
├── comm_link.h     # Public API (init, start, state, stats, send)
└── comm_link.c     # UART init, RX task, frame parsing, PID store
```

## Dependencies

- `driver` (ESP-IDF UART driver with DMA)
- `shared/comm_protocol` (message format)
- `shared/pid_types` (PID value types)
- `system` (logging)
