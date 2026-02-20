# comm_protocol -- Inter-Node Message Format Definitions

## Overview

This component defines the wire format for all messages exchanged between the
CAN Interface node and the Display node over their UART link. Both nodes
include this component so they serialize and deserialize messages using
identical structures.

The transport layer is a UART connection at 2 Mbps carried over a USB-C cable
(D+/D- repurposed as TX/RX). The same cable also delivers 5V power from the
CAN Interface node to the Display node.

---

## UART Frame Format

Every message on the UART link is wrapped in a frame envelope for reliable
byte-stream transport:

```
+------------+------------+-------------------------------+-----------+
| Start Byte | Header     | Payload (0..N bytes)          | CRC16     |
| (1 byte)   | (8 bytes)  |                               | (2 bytes) |
+------------+------------+-------------------------------+-----------+
```

| Field       | Size    | Description                                          |
|-------------|---------|------------------------------------------------------|
| Start byte  | 1 byte  | `0xAA` -- marks the beginning of a frame             |
| Header      | 8 bytes | Message header (see below)                           |
| Payload     | 0..N    | Payload content depends on `msg_type`                |
| CRC16       | 2 bytes | CRC-16/CCITT over header + payload bytes             |

The receiver scans for `0xAA`, reads the header to determine payload length,
reads the payload, then validates the CRC. Invalid frames are discarded and
counted as errors.

### Why Wired UART Instead of ESP-NOW?

The original design used ESP-NOW (wireless). The wired UART approach provides:
- **Deterministic delivery** -- no packet loss, no retransmission needed
- **No payload size limit** -- ESP-NOW caps at 250 bytes; UART has no such limit
- **Power delivery** -- same cable carries 5V to the display node
- **Simpler stack** -- no WiFi radio init, no peer management, no channel config
- **Lower latency** -- direct byte stream, no radio arbitration

---

## Message Header

```c
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;      // MessageType enum
    uint8_t  sequence;      // Rolling sequence number (0-255)
    uint32_t timestamp;     // Sender uptime in milliseconds
    uint16_t payload_len;   // Length of the payload that follows
} MessageHeader;
```

**Size:** 8 bytes (packed).

| Field         | Type       | Description                                    |
|---------------|------------|------------------------------------------------|
| `msg_type`    | `uint8_t`  | Identifies the payload structure (see below)   |
| `sequence`    | `uint8_t`  | Incremented per message; used for loss detection |
| `timestamp`   | `uint32_t` | Sender's `esp_timer` value in milliseconds     |
| `payload_len` | `uint16_t` | Byte count of the payload section              |

---

## Endianness

All multi-byte fields use **little-endian** byte order, which is the native
byte order of the ESP32-S3 (Xtensa LX7). No byte-swapping is required when
both sender and receiver are ESP32-S3 devices.

---

## Message Types

```c
typedef enum {
    MSG_PID_DATA_SINGLE = 0x01,
    MSG_PID_DATA_BATCH  = 0x02,
    MSG_VEHICLE_INFO    = 0x03,
    MSG_DTC_LIST        = 0x04,
    MSG_PID_METADATA    = 0x05,
    MSG_HEARTBEAT       = 0x10,
    MSG_CONFIG_CMD      = 0x20,
    MSG_CONFIG_RESP     = 0x21,
    MSG_LOG_CONTROL     = 0x30,
    MSG_SCAN_STATUS     = 0x40,
    MSG_ERROR_ALERT     = 0x41,
    MSG_TIME_SYNC       = 0x42,
    MSG_OTA_BEGIN       = 0x50,
    MSG_OTA_DATA        = 0x51,
    MSG_OTA_STATUS      = 0x52,
    MSG_OTA_COMPLETE    = 0x53,
} MessageType;
```

---

## Payload Structures

### PID_DATA_SINGLE (0x01)

Carries a single decoded PID reading.

```c
typedef struct __attribute__((packed)) {
    PIDValue pid;           // Single PID value (11 bytes)
} PayloadPidSingle;
```

**Size:** 11 bytes.

Used for real-time streaming of high-priority PIDs (e.g., RPM, speed) where
low latency matters more than bandwidth efficiency.

---

### PID_DATA_BATCH (0x02)

Carries multiple PID readings in a single frame.

```c
typedef struct __attribute__((packed)) {
    uint8_t  count;         // Number of PIDValue entries
    PIDValue pids[];        // Flexible array of PIDValue (11 bytes each)
} PayloadPidBatch;
```

**Max entries:** Limited only by UART buffer size and processing time. A
practical batch of 30-50 PID values (331-551 bytes) is well within the UART
throughput budget at 2 Mbps.

Used for periodic bulk updates where throughput is preferred over latency.
The `count` field tells the receiver how many entries follow.

---

### VEHICLE_INFO (0x03)

Static vehicle information retrieved from the ECU (VIN, calibration IDs,
etc.).

```c
typedef struct __attribute__((packed)) {
    uint8_t  info_type;     // Sub-type: 0x00 = VIN, 0x01 = Cal ID, ...
    uint8_t  segment;       // Segment index for multi-frame data
    uint8_t  total_segments;// Total segments expected
    uint8_t  data_len;      // Bytes of data in this segment
    uint8_t  data[];        // Variable-length data
} PayloadVehicleInfo;
```

Long strings (e.g., a 17-character VIN) may span multiple segments. The
receiver reassembles them using `segment` and `total_segments`.

---

### DTC_LIST (0x04)

Diagnostic Trouble Codes read from the vehicle.

```c
typedef struct __attribute__((packed)) {
    uint8_t  dtc_count;     // Number of DTCs in this frame
    uint8_t  more_available;// 1 if additional frames follow, 0 if last
    struct __attribute__((packed)) {
        uint16_t code;      // Raw DTC code (ISO 15031-6 encoding)
        uint8_t  status;    // DTC status byte
    } dtcs[];               // Flexible array of DTC entries
} PayloadDtcList;
```

**Bytes per DTC entry:** 3.

With UART transport there is no 250-byte frame limit, so large DTC lists
can be sent in fewer frames than a wireless design would require. The
`more_available` field is retained for flexibility.

---

### PID_METADATA (0x05)

Batch of PID descriptors sent from the CAN Interface to the Display after a
vehicle scan. Each entry carries the PID number, human-readable name, and
base unit string. The Display stores these **in RAM only** (not flash) and
uses them to populate the PID dropdown and label gauge readings.

```c
typedef struct __attribute__((packed)) {
    uint16_t pid_id;                        // PID number
    char     name[PID_META_NAME_LEN];       // Human-readable name (32 chars)
    char     unit_str[PID_META_UNIT_LEN];   // Unit display string (8 chars)
} comm_pid_meta_t;
```

**Size per entry:** 42 bytes. Up to 24 entries per frame (at 1024-byte max
payload). Multiple `PID_METADATA` frames are sent in sequence to cover all
supported data PIDs. Only `FORMULA` and `ENUM` type PIDs are included;
bitmap, status, and string PIDs are filtered out.

**Scan flow:**
1. Interface sends `SCAN_STATUS` IN_PROGRESS (Display clears metadata store)
2. Interface sends `VEHICLE_INFO` (VIN, ECU count, supported PID bitmap)
3. Interface sends one or more `PID_METADATA` batches
4. Interface sends `SCAN_STATUS` COMPLETE (Display fires scan callback)

On restart, the Display discards all metadata and re-scans to get fresh data.

---

### HEARTBEAT (0x10)

Periodic keep-alive sent in both directions so each node can detect link
failure.

```c
typedef struct __attribute__((packed)) {
    uint8_t  node_state;    // 0 = idle, 1 = scanning, 2 = streaming, ...
    uint8_t  can_status;    // 0 = bus off, 1 = bus on, 2 = error passive
    uint16_t free_heap_kb;  // Free heap in kilobytes
    uint32_t uptime_ms;     // Sender uptime for time sync offset calculation
} PayloadHeartbeat;
```

**Size:** 8 bytes.

Recommended interval: 1 second. If either node misses several consecutive
heartbeats, it should indicate a link failure to the user.

The `uptime_ms` field enables **time synchronization** between nodes. The
Display node calculates a clock offset from the CAN node's uptime vs. its
own, so timestamps on PID data and log entries are consistent. This avoids
the drift issues that occur when two independent `esp_timer` clocks run
unsynchronized.

---

### CONFIG_CMD (0x20)

Command sent from the Display node to the CAN Interface node to change
runtime configuration.

```c
typedef struct __attribute__((packed)) {
    uint8_t  config_key;    // Which parameter to change
    uint8_t  value_len;     // Length of the value field
    uint8_t  value[];       // New value (format depends on config_key)
} PayloadConfigCmd;
```

Example `config_key` values:

| Key  | Meaning                        | Value Format          |
|------|--------------------------------|-----------------------|
| 0x01 | Set PID poll list              | Array of uint16_t     |
| 0x02 | Set poll interval (ms)         | uint16_t              |
| 0x03 | Set CAN bus speed              | uint32_t (baud rate)  |
| 0x04 | Enable/disable protocol auto-detect | uint8_t (0 or 1) |

---

### CONFIG_RESP (0x21)

Acknowledgment from the CAN Interface node in response to a CONFIG_CMD.

```c
typedef struct __attribute__((packed)) {
    uint8_t  config_key;    // Echoed from the command
    uint8_t  result;        // 0 = success, non-zero = error code
    uint8_t  data_len;      // Length of optional response data
    uint8_t  data[];        // Optional response data
} PayloadConfigResp;
```

---

### LOG_CONTROL (0x30)

Controls diagnostic logging on the CAN Interface node.

```c
typedef struct __attribute__((packed)) {
    uint8_t  action;        // 0 = stop, 1 = start, 2 = set level
    uint8_t  log_level;     // ESP-IDF log level (0=NONE .. 5=VERBOSE)
    uint8_t  module_id;     // 0 = all, or specific module ID
} PayloadLogControl;
```

**Size:** 3 bytes.

Allows the Display node to remotely adjust verbosity on the CAN Interface
node for troubleshooting without reflashing.

---

### SCAN_STATUS (0x40)

Reports progress of an OBD-II PID scan (supported-PID discovery).

```c
typedef struct __attribute__((packed)) {
    uint8_t  phase;         // 0 = not started, 1 = in progress, 2 = complete, 3 = error
    uint16_t pids_found;    // Number of supported PIDs discovered so far
    uint16_t pids_total;    // Estimated total PIDs to check
    uint8_t  current_range; // Current PID range being scanned (0x00, 0x20, 0x40, ...)
} PayloadScanStatus;
```

**Size:** 6 bytes.

The Display node uses this to show a progress indicator during the initial
PID enumeration phase.

---

### ERROR_ALERT (0x41)

Priority-based error and alert messages from CAN node to Display node. The
Display node decides how to present these to the user based on severity.

```c
typedef enum {
    ERROR_SEV_INFO     = 0,   // Informational -- show briefly, log, continue
    ERROR_SEV_WARNING  = 1,   // Warning -- show to user, log, continue
    ERROR_SEV_CRITICAL = 2,   // Critical -- show to user, halt affected system, await user decision
} ErrorSeverity;

typedef struct __attribute__((packed)) {
    uint8_t  severity;      // ErrorSeverity enum
    uint8_t  source_module; // Which component raised the error (0=CAN, 1=OBD2, 2=poll, ...)
    uint16_t error_code;    // Module-specific error code
    uint8_t  msg_len;       // Length of human-readable message
    char     message[];     // Null-terminated error description
} PayloadErrorAlert;
```

**Severity behavior:**

| Severity   | Display Behavior                                                    |
|------------|---------------------------------------------------------------------|
| INFO       | Brief toast notification. Logged to debug. System continues.        |
| WARNING    | Persistent notification on screen. Logged. System continues.        |
| CRITICAL   | Full-screen alert. System stops affected subsystem. User must acknowledge and decide (retry, ignore, shutdown). |

---

### TIME_SYNC (0x42)

Dedicated time synchronization message for precise clock offset calculation.
Supplements the coarse sync in heartbeat with a request/response round-trip.

```c
typedef struct __attribute__((packed)) {
    uint8_t  sync_type;     // 0 = request (Display->CAN), 1 = response (CAN->Display)
    uint32_t t1;            // Requester's send timestamp (ms)
    uint32_t t2;            // Responder's receive timestamp (ms)
    uint32_t t3;            // Responder's send timestamp (ms)
} PayloadTimeSync;
```

**Size:** 13 bytes.

The Display node sends a sync request (type=0, t1=now). The CAN node
responds (type=1, t1=echoed, t2=when received, t3=when sending response).
The Display node records t4=when response arrived. Clock offset and
round-trip time can then be calculated:

- Round-trip = (t4 - t1) - (t3 - t2)
- Offset = ((t2 - t1) + (t3 - t4)) / 2

---

### OTA_BEGIN (0x50)

Initiates an over-the-air firmware update of the CAN Interface Node. Sent
from Display Node after receiving the firmware image via WiFi.

```c
typedef struct __attribute__((packed)) {
    uint32_t firmware_size; // Total firmware image size in bytes
    uint32_t firmware_crc;  // CRC32 of the complete firmware image
    uint8_t  version_len;   // Length of version string
    char     version[];     // Firmware version string (e.g., "1.2.0")
} PayloadOtaBegin;
```

The CAN node validates it has enough OTA partition space, prepares for
writing, and responds with CONFIG_RESP (config_key=0x50, result=0 for
ready, non-zero for error).

---

### OTA_DATA (0x51)

Carries a chunk of firmware data during OTA update.

```c
typedef struct __attribute__((packed)) {
    uint32_t offset;        // Byte offset within the firmware image
    uint16_t chunk_len;     // Length of data in this frame
    uint8_t  data[];        // Firmware data chunk
} PayloadOtaData;
```

Chunk size is limited by practical UART frame size. Recommended: 1024 bytes
per chunk (fits comfortably at 2Mbps with ~5ms per frame).

---

### OTA_STATUS (0x52)

Progress and status updates during OTA from CAN node back to Display node.

```c
typedef struct __attribute__((packed)) {
    uint8_t  status;        // 0 = in progress, 1 = verify, 2 = error
    uint32_t bytes_written; // Total bytes written so far
    uint8_t  error_code;    // 0 = none, non-zero = specific error
} PayloadOtaStatus;
```

**Size:** 6 bytes.

---

### OTA_COMPLETE (0x53)

Signals OTA completion. Sent by Display node after all data chunks are
acknowledged. CAN node validates CRC, sets boot partition, and reboots.

```c
typedef struct __attribute__((packed)) {
    uint8_t  action;        // 0 = verify and reboot, 1 = abort and rollback
} PayloadOtaComplete;
```

**Size:** 1 byte.

---

## Sequence Numbering

The `sequence` field in the header is a uint8_t that increments by 1 for each
message sent, wrapping from 255 back to 0. The receiver tracks the last
sequence number per sender. A gap indicates one or more lost or corrupted
frames.

With UART transport, frame loss is rare (unlike wireless). Sequence numbers
primarily serve to:

1. Detect corrupted frames that passed CRC (extremely unlikely but possible).
2. Reassemble multi-frame payloads in the correct order (e.g., DTC_LIST with
   `more_available`, or segmented VEHICLE_INFO).
3. Provide a diagnostic metric for link health.

---

## CRC Calculation

The CRC-16/CCITT polynomial (`0x1021`, init `0xFFFF`) is computed over all
bytes from the header through the end of the payload. The 2-byte CRC is
appended after the payload in little-endian order.

The CRC implementation is provided by `shared/utils/`.
