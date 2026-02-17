# utils -- Common Utility Functions

## Overview

This component provides low-level utility functions used by both the CAN
Interface and Display nodes. These are kept in `shared/` so that both sides
compute checksums, pack bytes, and handle timestamps in exactly the same way.

---

## CRC Functions

### CRC8

```c
uint8_t crc8(const uint8_t *data, size_t len);
```

Computes an 8-bit CRC over the given buffer. Used for lightweight integrity
checks on short payloads or as a fast validation step before processing a
message.

**Polynomial:** 0x07 (CRC-8/SAE-J1850 or similar automotive-grade polynomial).

**Typical usage:** Appended to or verified against individual PID readings or
configuration commands where a full CRC16 would be unnecessary overhead.

### CRC16

```c
uint16_t crc16(const uint8_t *data, size_t len);
```

Computes a 16-bit CRC over the given buffer. Used for integrity checks on
larger payloads such as batch PID data or DTC lists.

**Polynomial:** 0x8005 (CRC-16/IBM) or 0x1021 (CRC-16/CCITT), depending on
project configuration.

**Typical usage:** Computed over the entire message (header + payload) by the
sender and verified by the receiver. A CRC mismatch causes the frame to be
silently dropped.

### Integration with comm_protocol

The CRC is calculated over the full serialized message (header + payload)
before transmission. The receiver recomputes the CRC on the received bytes
and compares it to the transmitted value. This catches bit errors introduced
by RF interference, which is common in automotive environments.

---

## Byte Packing / Unpacking Helpers

These functions provide portable serialization of multi-byte values into byte
buffers, ensuring consistent little-endian encoding regardless of compiler
behavior.

### Pack Functions

```c
void pack_uint16(uint8_t *buf, uint16_t value);
void pack_uint32(uint8_t *buf, uint32_t value);
void pack_float(uint8_t *buf, float value);
```

Write a value into `buf` in little-endian byte order. The caller must ensure
`buf` points to a region with enough space (2, 4, or 4 bytes respectively).

### Unpack Functions

```c
uint16_t unpack_uint16(const uint8_t *buf);
uint32_t unpack_uint32(const uint8_t *buf);
float    unpack_float(const uint8_t *buf);
```

Read a little-endian value from `buf` and return it as the native type.

### Why Not Just Use Packed Structs?

Packed structs (`__attribute__((packed))`) work correctly on the ESP32-S3 for
direct memory access. The pack/unpack helpers exist for cases where:

- Data is built incrementally into a byte buffer rather than written as a
  whole struct.
- A future port to a big-endian platform is possible.
- Code needs to operate on sub-fields of a buffer without casting to a struct
  pointer, avoiding strict-aliasing concerns.

---

## Timestamp Utilities

### get_timestamp_ms

```c
uint32_t get_timestamp_ms(void);
```

Returns the current time in milliseconds since boot, derived from
`esp_timer_get_time()` (which returns microseconds). This is the standard
timestamp source used in message headers and `PIDValue` structs.

The return value wraps after approximately 49.7 days of continuous uptime.
For the intended automotive diagnostic use case, this is acceptable.

### elapsed_ms

```c
uint32_t elapsed_ms(uint32_t start);
```

Returns the number of milliseconds elapsed since `start`, correctly handling
the uint32_t wrap-around case. Useful for timeout checks and interval timing
without worrying about overflow arithmetic.

```c
uint32_t t0 = get_timestamp_ms();
// ... do work ...
if (elapsed_ms(t0) > TIMEOUT_MS) {
    // handle timeout
}
```

---

## Usage Notes

All functions in this component are stateless and safe to call from any
FreeRTOS task without synchronization. None of them allocate heap memory.
