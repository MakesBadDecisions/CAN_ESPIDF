/**
 * @file utils.c
 * @brief Common Utility Functions Implementation
 */

#include "utils.h"
#include <string.h>

// ============================================================================
// CRC-8 (Polynomial 0x07)
// ============================================================================

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// CRC-16/CCITT (Polynomial 0x1021, Init 0xFFFF)
// ============================================================================

uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= ((uint16_t)*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Byte Packing (Little-Endian)
// ============================================================================

void pack_uint16(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
}

void pack_uint32(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
}

void pack_float(uint8_t *buf, float value)
{
    uint32_t tmp;
    memcpy(&tmp, &value, sizeof(tmp));
    pack_uint32(buf, tmp);
}

uint16_t unpack_uint16(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

uint32_t unpack_uint32(const uint8_t *buf)
{
    return (uint32_t)buf[0] | 
           ((uint32_t)buf[1] << 8) | 
           ((uint32_t)buf[2] << 16) | 
           ((uint32_t)buf[3] << 24);
}

float unpack_float(const uint8_t *buf)
{
    uint32_t tmp = unpack_uint32(buf);
    float value;
    memcpy(&value, &tmp, sizeof(value));
    return value;
}

