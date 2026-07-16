#include "ret_flash_storage.h"

#include "ret_board_config.h"

#include <stddef.h>
#include <string.h>

#define RET_STORAGE_MAGIC 0x32544552u /* "RET2" */
#define RET_STORAGE_VALID 0x56414C49u /* "VALI" */
#define RET_STORAGE_FORMAT 1u

typedef struct {
    ret_config_t config;
    uint16_t adc_min;
    uint16_t adc_max;
    uint8_t adc_calibrated;
    uint8_t reserved[3];
} ret_storage_payload_t;

typedef struct {
    uint32_t magic;
    uint32_t format;
    uint32_t sequence;
    uint32_t payload_size;
    uint32_t payload_crc;
    ret_storage_payload_t payload;
    uint32_t valid;
} ret_storage_record_t;

#define RET_STORAGE_RECORD_SIZE \
    ((sizeof(ret_storage_record_t) + 3u) & ~(size_t)3u)
#define RET_STORAGE_SLOT_COUNT (RET_FLASH_SIZE / RET_STORAGE_RECORD_SIZE)

_Static_assert((RET_STORAGE_RECORD_SIZE % 4u) == 0u,
               "flash records must be word aligned");
_Static_assert((offsetof(ret_storage_record_t, valid) % 4u) == 0u,
               "commit marker must be word aligned");
_Static_assert(RET_STORAGE_SLOT_COUNT >= 2u,
               "the selected flash sector is too small");

static uint32_t crc32(const void *data, const size_t length) {
    const uint8_t *bytes = data;
    uint32_t crc = 0xFFFFFFFFu;
    size_t index;
    for (index = 0u; index < length; ++index) {
        unsigned bit;
        crc ^= bytes[index];
        for (bit = 0u; bit < 8u; ++bit) {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static const ret_storage_record_t *record_at(const size_t slot) {
    return (const ret_storage_record_t *)(uintptr_t)
        (RET_FLASH_ADDRESS + (slot * RET_STORAGE_RECORD_SIZE));
}

static bool record_valid(const ret_storage_record_t *record) {
    return (record->magic == RET_STORAGE_MAGIC) &&
           (record->format == RET_STORAGE_FORMAT) &&
           (record->payload_size == sizeof(record->payload)) &&
           (record->valid == RET_STORAGE_VALID) &&
           (record->payload_crc ==
            crc32(&record->payload, sizeof(record->payload))) &&
           ret_config_is_valid(&record->payload.config);
}

static const ret_storage_record_t *newest_record(uint32_t *sequence,
                                                  size_t *free_slot) {
    const ret_storage_record_t *newest = NULL;
    size_t slot;
    *sequence = 0u;
    *free_slot = RET_STORAGE_SLOT_COUNT;
    for (slot = 0u; slot < RET_STORAGE_SLOT_COUNT; ++slot) {
        const ret_storage_record_t *record = record_at(slot);
        if ((record->magic == 0xFFFFFFFFu) &&
            (*free_slot == RET_STORAGE_SLOT_COUNT)) {
            *free_slot = slot;
        }
        if (record_valid(record) &&
            ((newest == NULL) ||
             ((int32_t)(record->sequence - *sequence) > 0))) {
            newest = record;
            *sequence = record->sequence;
        }
    }
    return newest;
}

bool ret_stm32_storage_load(void *opaque, ret_config_t *config) {
    ret_stm32_context_t *context = opaque;
    const ret_storage_record_t *record;
    uint32_t sequence;
    size_t free_slot;
    if ((context == NULL) || (config == NULL)) {
        return false;
    }
    record = newest_record(&sequence, &free_slot);
    (void)sequence;
    (void)free_slot;
    if (record == NULL) {
        return false;
    }
    *config = record->payload.config;
    context->adc_min = record->payload.adc_min;
    context->adc_max = record->payload.adc_max;
    context->adc_calibrated = record->payload.adc_calibrated != 0u;
    return true;
}

static bool erase_storage_sector(void) {
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0u;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = RET_FLASH_SECTOR;
    erase.NbSectors = 1u;
    return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
}

static bool program_words(const uint32_t address,
                          const void *data,
                          const size_t length) {
    const uint8_t *bytes = data;
    size_t offset;
    for (offset = 0u; offset < length; offset += sizeof(uint32_t)) {
        uint32_t word = 0xFFFFFFFFu;
        const size_t remaining = length - offset;
        const size_t copy_length =
            (remaining < sizeof(word)) ? remaining : sizeof(word);
        memcpy(&word, &bytes[offset], copy_length);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address + (uint32_t)offset,
                              word) != HAL_OK) {
            return false;
        }
    }
    return true;
}

bool ret_stm32_storage_save(void *opaque, const ret_config_t *config) {
    ret_stm32_context_t *context = opaque;
    ret_storage_record_t record;
    const ret_storage_record_t *ignored;
    uint32_t sequence;
    uint32_t address;
    size_t free_slot;
    size_t valid_offset;
    bool success = false;

    if ((context == NULL) || !ret_config_is_valid(config)) {
        return false;
    }
    ignored = newest_record(&sequence, &free_slot);
    (void)ignored;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (free_slot == RET_STORAGE_SLOT_COUNT) {
        if (!erase_storage_sector()) {
            goto finished;
        }
        free_slot = 0u;
    }

    memset(&record, 0, sizeof(record));
    record.magic = RET_STORAGE_MAGIC;
    record.format = RET_STORAGE_FORMAT;
    record.sequence = sequence + 1u;
    record.payload_size = sizeof(record.payload);
    record.payload.config = *config;
    record.payload.adc_min = context->adc_min;
    record.payload.adc_max = context->adc_max;
    record.payload.adc_calibrated = context->adc_calibrated ? 1u : 0u;
    record.payload_crc = crc32(&record.payload, sizeof(record.payload));
    record.valid = RET_STORAGE_VALID;

    address = RET_FLASH_ADDRESS +
              (uint32_t)(free_slot * RET_STORAGE_RECORD_SIZE);
    valid_offset = offsetof(ret_storage_record_t, valid);
    if (!program_words(address, &record, valid_offset)) {
        goto finished;
    }
    success = program_words(address + (uint32_t)valid_offset,
                            &record.valid, sizeof(record.valid));

finished:
    (void)HAL_FLASH_Lock();
    return success;
}
