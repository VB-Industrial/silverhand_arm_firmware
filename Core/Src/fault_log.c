#include "fault_log.h"

#include <stddef.h>
#include <string.h>

#include "at24_eeprom.h"

enum {
    FAULT_LOG_BASE_ADDRESS = 0x1800U,
    FAULT_LOG_SLOT_SIZE = 32U,
    FAULT_LOG_SLOT_COUNT = 64U,
    FAULT_LOG_VERSION = 1U,
    FAULT_LOG_VALID_MARKER = 0xA5U,
};

static const uint32_t FAULT_LOG_MAGIC = 0x4C464853UL; /* "SHFL" in EEPROM. */
static const uint32_t CRC32_POLYNOMIAL = 0xEDB88320UL;
static const uint32_t EEPROM_TIMEOUT_MS = 20U;

static bool log_available = false;
static bool has_last_record = false;
static uint8_t configured_joint_id = 0U;
static uint8_t next_slot = 0U;
static fault_log_record last_record;

_Static_assert(sizeof(fault_log_record) == FAULT_LOG_SLOT_SIZE, "fault log record must fit one EEPROM page");

static uint32_t crc32(const uint8_t* data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0U; i < size; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? CRC32_POLYNOMIAL : 0U);
        }
    }
    return ~crc;
}

static uint16_t slot_address(uint8_t slot)
{
    return (uint16_t)(FAULT_LOG_BASE_ADDRESS + ((uint16_t)slot * FAULT_LOG_SLOT_SIZE));
}

static bool sequence_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static bool record_is_valid(const fault_log_record* record)
{
    if ((record->magic != FAULT_LOG_MAGIC) ||
        (record->version != FAULT_LOG_VERSION) ||
        (record->size != sizeof(fault_log_record)) ||
        (record->valid != FAULT_LOG_VALID_MARKER)) {
        return false;
    }

    fault_log_record copy = *record;
    copy.valid = 0U;
    const uint32_t expected_crc = copy.crc32;
    return crc32((const uint8_t*)&copy, offsetof(fault_log_record, crc32)) == expected_crc;
}

void fault_log_init(uint8_t joint_id)
{
    configured_joint_id = joint_id;
    log_available = at24_isConnected() != 0;
    has_last_record = false;
    next_slot = 0U;
    memset(&last_record, 0, sizeof(last_record));

    if (!log_available) {
        return;
    }

    uint8_t newest_slot = 0U;
    for (uint8_t slot = 0U; slot < FAULT_LOG_SLOT_COUNT; ++slot) {
        fault_log_record record;
        if ((at24_read(slot_address(slot), (uint8_t*)&record, sizeof(record), EEPROM_TIMEOUT_MS) != 0) &&
            record_is_valid(&record) &&
            (!has_last_record || sequence_is_newer(record.sequence, last_record.sequence))) {
            last_record = record;
            newest_slot = slot;
            has_last_record = true;
        }
    }

    if (has_last_record) {
        next_slot = (uint8_t)((newest_slot + 1U) % FAULT_LOG_SLOT_COUNT);
    }
}

bool fault_log_append(
    uint32_t uptime_ms,
    uint32_t fault_mask,
    uint32_t tmc_gstat,
    uint32_t tmc_drv_status)
{
    if (!log_available || (fault_mask == 0U)) {
        return false;
    }

    fault_log_record record = {
        .magic = FAULT_LOG_MAGIC,
        .version = FAULT_LOG_VERSION,
        .size = sizeof(fault_log_record),
        .joint_id = configured_joint_id,
        .valid = 0U,
        .sequence = has_last_record ? (last_record.sequence + 1U) : 1U,
        .uptime_ms = uptime_ms,
        .fault_mask = fault_mask,
        .tmc_gstat = tmc_gstat,
        .tmc_drv_status = tmc_drv_status,
        .crc32 = 0U,
    };
    record.crc32 = crc32((const uint8_t*)&record, offsetof(fault_log_record, crc32));

    const uint16_t address = slot_address(next_slot);
    if (at24_write(address, (uint8_t*)&record, sizeof(record), EEPROM_TIMEOUT_MS) == 0) {
        return false;
    }

    const uint8_t valid = FAULT_LOG_VALID_MARKER;
    if (at24_write(
            (uint16_t)(address + offsetof(fault_log_record, valid)),
            (uint8_t*)&valid,
            sizeof(valid),
            EEPROM_TIMEOUT_MS) == 0) {
        return false;
    }

    fault_log_record verify;
    if ((at24_read(address, (uint8_t*)&verify, sizeof(verify), EEPROM_TIMEOUT_MS) == 0) ||
        !record_is_valid(&verify)) {
        return false;
    }

    last_record = verify;
    has_last_record = true;
    next_slot = (uint8_t)((next_slot + 1U) % FAULT_LOG_SLOT_COUNT);
    return true;
}

bool fault_log_available(void)
{
    return log_available;
}

bool fault_log_get_last(fault_log_record* record)
{
    if (!has_last_record || (record == NULL)) {
        return false;
    }
    *record = last_record;
    return true;
}
