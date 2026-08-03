#include "encoder_calibration_storage.h"

#include <stddef.h>
#include <string.h>

#include "at24_eeprom.h"

enum {
    CALIBRATION_VERSION = 2U,
    CALIBRATION_LEGACY_VERSION = 1U,
    CALIBRATION_VALID = 0xA5U,
    CALIBRATION_SLOT_SIZE = 768U,
    CALIBRATION_SLOT_A = 0x0000U,
    CALIBRATION_SLOT_B = 0x0300U,
    EEPROM_CHUNK_SIZE = 128U,
    EEPROM_TIMEOUT_MS = 100U,
};

static const uint32_t CALIBRATION_MAGIC = 0x43454853UL; /* "SHEC" */
static const uint32_t CRC32_POLYNOMIAL = 0xEDB88320UL;

typedef struct calibration_record {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t joint_id;
    uint8_t valid;
    uint16_t reserved;
    encoder_calibration_data data;
    uint32_t crc32;
} calibration_record;

typedef struct legacy_calibration_data {
    uint32_t sequence;
    uint16_t point_count;
    uint16_t limit_a_raw;
    uint16_t limit_b_raw;
    int32_t manual_span_ticks;
    uint16_t safe_margin_ticks;
    int32_t tmc_span_steps;
    int16_t correction_ticks[ENCODER_CALIBRATION_MAX_POINTS];
} legacy_calibration_data;

typedef struct legacy_calibration_record {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t joint_id;
    uint8_t valid;
    uint16_t reserved;
    legacy_calibration_data data;
    uint32_t crc32;
} legacy_calibration_record;

_Static_assert(sizeof(calibration_record) == 556U, "unexpected calibration v2 layout");
_Static_assert(sizeof(legacy_calibration_record) == 552U, "unexpected calibration v1 layout");
_Static_assert(sizeof(calibration_record) <= CALIBRATION_SLOT_SIZE, "calibration record exceeds EEPROM slot");

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

static bool sequence_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static bool read_bytes(uint16_t address, uint8_t* data, size_t size)
{
    while (size > 0U) {
        const uint8_t chunk = (uint8_t)((size > EEPROM_CHUNK_SIZE) ? EEPROM_CHUNK_SIZE : size);
        if (at24_read(address, data, chunk, EEPROM_TIMEOUT_MS) == 0) {
            return false;
        }
        address = (uint16_t)(address + chunk);
        data += chunk;
        size -= chunk;
    }
    return true;
}

static bool write_bytes(uint16_t address, uint8_t* data, size_t size)
{
    while (size > 0U) {
        const uint8_t chunk = (uint8_t)((size > EEPROM_CHUNK_SIZE) ? EEPROM_CHUNK_SIZE : size);
        if (at24_write(address, data, chunk, EEPROM_TIMEOUT_MS) == 0) {
            return false;
        }
        address = (uint16_t)(address + chunk);
        data += chunk;
        size -= chunk;
    }
    return true;
}

static bool record_valid(const calibration_record* record, uint8_t joint_id)
{
    if ((record->magic != CALIBRATION_MAGIC) ||
        (record->version != CALIBRATION_VERSION) ||
        (record->size != sizeof(calibration_record)) ||
        (record->joint_id != joint_id) ||
        (record->valid != CALIBRATION_VALID) ||
        (record->data.point_count < 2U) ||
        (record->data.point_count > ENCODER_CALIBRATION_MAX_POINTS)) {
        return false;
    }
    calibration_record copy = *record;
    copy.valid = 0U;
    const uint32_t expected = copy.crc32;
    return crc32((const uint8_t*)&copy, offsetof(calibration_record, crc32)) == expected;
}

static bool legacy_record_valid(const legacy_calibration_record* record, const uint8_t joint_id)
{
    if ((record->magic != CALIBRATION_MAGIC) ||
        (record->version != CALIBRATION_LEGACY_VERSION) ||
        (record->size != sizeof(legacy_calibration_record)) ||
        (record->joint_id != joint_id) ||
        (record->valid != CALIBRATION_VALID) ||
        (record->data.point_count < 2U) ||
        (record->data.point_count > ENCODER_CALIBRATION_MAX_POINTS)) {
        return false;
    }
    legacy_calibration_record copy = *record;
    copy.valid = 0U;
    const uint32_t expected = copy.crc32;
    return crc32((const uint8_t*)&copy, offsetof(legacy_calibration_record, crc32)) == expected;
}

static bool read_record(const uint16_t address, const uint8_t joint_id, calibration_record* record)
{
    struct {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
    } header;
    if (!read_bytes(address, (uint8_t*)&header, sizeof(header)) || (header.magic != CALIBRATION_MAGIC)) {
        return false;
    }
    if ((header.version == CALIBRATION_VERSION) && (header.size == sizeof(*record))) {
        return read_bytes(address, (uint8_t*)record, sizeof(*record)) && record_valid(record, joint_id);
    }
    if ((header.version == CALIBRATION_LEGACY_VERSION) &&
        (header.size == sizeof(legacy_calibration_record))) {
        legacy_calibration_record legacy;
        if (!read_bytes(address, (uint8_t*)&legacy, sizeof(legacy)) ||
            !legacy_record_valid(&legacy, joint_id)) {
            return false;
        }
        memset(record, 0, sizeof(*record));
        record->magic = CALIBRATION_MAGIC;
        record->version = CALIBRATION_VERSION;
        record->size = sizeof(*record);
        record->joint_id = joint_id;
        record->valid = CALIBRATION_VALID;
        record->data.sequence = legacy.data.sequence;
        record->data.point_count = legacy.data.point_count;
        record->data.limit_a_raw = legacy.data.limit_a_raw;
        record->data.limit_b_raw = legacy.data.limit_b_raw;
        record->data.manual_span_ticks = legacy.data.manual_span_ticks;
        record->data.safe_margin_ticks = legacy.data.safe_margin_ticks;
        record->data.tmc_span_steps = legacy.data.tmc_span_steps;
        memcpy(record->data.correction_ticks, legacy.data.correction_ticks, sizeof(legacy.data.correction_ticks));
        return true;
    }
    return false;
}

bool encoder_calibration_storage_load(uint8_t joint_id, encoder_calibration_data* data)
{
    if ((data == NULL) || (at24_isConnected() == 0)) {
        return false;
    }
    calibration_record a;
    calibration_record b;
    const bool valid_a = read_record(CALIBRATION_SLOT_A, joint_id, &a);
    const bool valid_b = read_record(CALIBRATION_SLOT_B, joint_id, &b);
    if (!valid_a && !valid_b) {
        return false;
    }
    *data = (valid_b && (!valid_a || sequence_is_newer(b.data.sequence, a.data.sequence))) ? b.data : a.data;
    return true;
}

bool encoder_calibration_storage_save(uint8_t joint_id, encoder_calibration_data* data)
{
    if ((data == NULL) || (at24_isConnected() == 0)) {
        return false;
    }

    encoder_calibration_data current;
    if (encoder_calibration_storage_load(joint_id, &current)) {
        data->sequence = current.sequence + 1U;
    } else {
        data->sequence = 1U;
    }
    const uint16_t address = ((data->sequence & 1U) != 0U) ? CALIBRATION_SLOT_A : CALIBRATION_SLOT_B;
    calibration_record record;
    memset(&record, 0, sizeof(record));
    record.magic = CALIBRATION_MAGIC;
    record.version = CALIBRATION_VERSION;
    record.size = sizeof(calibration_record);
    record.joint_id = joint_id;
    record.data = *data;
    record.crc32 = crc32((const uint8_t*)&record, offsetof(calibration_record, crc32));

    if (!write_bytes(address, (uint8_t*)&record, sizeof(record))) {
        return false;
    }
    const uint8_t valid = CALIBRATION_VALID;
    if (at24_write((uint16_t)(address + offsetof(calibration_record, valid)), (uint8_t*)&valid, 1U, EEPROM_TIMEOUT_MS) == 0) {
        return false;
    }
    calibration_record verify;
    if (!read_record(address, joint_id, &verify)) {
        return false;
    }
    *data = verify.data;
    return true;
}
