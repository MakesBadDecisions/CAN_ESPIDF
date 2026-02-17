# comm_link - UART Communication (CAN Node TX)

## Purpose

Handles all communication from the CAN interface node to the display node via
UART over a USB-C cable. Batches PID data for efficient transmission, manages
connection state, and processes configuration commands received from the
display node.

## Why Wired UART

- **Deterministic delivery:** No packet loss, no retransmission needed
- **No payload size limit:** Unlike ESP-NOW's 250-byte cap
- **Full duplex:** Simultaneous TX and RX at 2 Mbps
- **DMA support:** Near-zero CPU overhead for data transfer
- **Power delivery:** Same USB-C cable carries 5V to display node
- **Simple:** No WiFi init, no peer management, no channel conflicts

## Message Flow

```
CAN Interface Node                    Display Node
       │            USB-C Cable            │
       │         (UART 2Mbps)              │
       │                                   │
       │──── PID_DATA_BATCH ──────────────>│  (every 20-50ms)
       │──── HEARTBEAT ───────────────────>│  (every 1000ms)
       │──── VEHICLE_INFO ────────────────>│  (on scan complete)
       │──── DTC_LIST ────────────────────>│  (on DTC read)
       │──── SCAN_STATUS ────────────────>│  (during scan)
       │                                   │
       │<─── CONFIG_CMD ──────────────────│  (poll list change)
       │<─── LOG_CONTROL ────────────────│  (start/stop log)
       │<─── HEARTBEAT ──────────────────│  (every 1000ms)
       │                                   │
       │──── CONFIG_RESP ─────────────────>│  (acknowledge)
       │                                   │
```

## Batching Strategy

PID values are batched into UART frames for efficiency:

```c
// Each PID value in a batch: 8 bytes
typedef struct __attribute__((packed)) {
    uint16_t pid;           // PID ID
    float    value;         // Decoded value
    uint16_t unit;          // Unit enum (for display formatting)
} comm_pid_value_t;         // 8 bytes
```

At 2 Mbps, throughput is ~200 KB/s. A batch of 50 PID values (~400 bytes
with framing) takes ~2ms to transmit, leaving ample headroom.

The comm_link task:
1. Checks the PID data store for recently updated values
2. Packs changed values into a batch frame (start byte + header + payload + CRC)
3. Sends the frame via UART TX with DMA
4. Large batches are sent in a single frame (no 250-byte limit)

## UART Frame Format

Each message uses the framing defined in `shared/comm_protocol/`:

```
[0xAA] [Header 8B] [Payload 0..N bytes] [CRC16 2B]
```

The UART driver handles byte-level transport. The comm_link layer handles
message framing, CRC validation, and dispatch.

## Connection Management

- **Link detection:** UART RX activity indicates the display node is connected
- **Heartbeat:** Sent every 1000ms in both directions
- **Timeout:** If no heartbeat received for 5000ms, mark link as down
- **Auto-resume:** Resume transmission when heartbeat resumes

## Files

```
comm_link/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   └── comm_link.h          # Public API
└── src/
    ├── comm_link.c           # UART init, frame send/receive, dispatch
    ├── comm_link_tx.c        # PID batching and transmission
    └── comm_link_rx.c        # Config command handling (from display node)
```

## Dependencies

- `driver` (ESP-IDF UART driver with DMA)
- `shared/comm_protocol` (message format definitions)
- `shared/pid_types` (PID value types)
- `system` (logging, NVS)
