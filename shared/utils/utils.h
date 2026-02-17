/**
 * @file utils.h
 * @brief Common Utility Functions
 * 
 * CRC calculations, byte packing/unpacking, timestamps.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// CRC Functions
// ============================================================================

/**
 * @brief Compute CRC-8 over buffer
 * @param data Input buffer
 * @param len Buffer length
 * @return 8-bit CRC
 */
uint8_t crc8(const uint8_t *data, size_t len);

/**
 * @brief Compute CRC-16/CCITT over buffer
 * @param data Input buffer
 * @param len Buffer length
 * @return 16-bit CRC
 */
uint16_t crc16(const uint8_t *data, size_t len);

// ============================================================================
// Byte Packing (Little-Endian)
// ============================================================================

void pack_uint16(uint8_t *buf, uint16_t value);
void pack_uint32(uint8_t *buf, uint32_t value);
void pack_float(uint8_t *buf, float value);

uint16_t unpack_uint16(const uint8_t *buf);
uint32_t unpack_uint32(const uint8_t *buf);
float    unpack_float(const uint8_t *buf);

