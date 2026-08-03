#ifndef INC_ENCODER_CALIBRATION_STORAGE_H_
#define INC_ENCODER_CALIBRATION_STORAGE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { ENCODER_CALIBRATION_MAX_POINTS = 256U };

typedef struct encoder_calibration_data {
    uint32_t sequence;
    uint16_t point_count;
    uint16_t limit_a_raw;
    uint16_t limit_b_raw;
    int32_t manual_span_ticks;
    uint16_t safe_margin_ticks;
    int32_t tmc_span_steps;
    uint16_t zero_raw;
    uint8_t zero_valid;
    uint8_t reserved;
    int16_t correction_ticks[ENCODER_CALIBRATION_MAX_POINTS];
} encoder_calibration_data;

bool encoder_calibration_storage_load(uint8_t joint_id, encoder_calibration_data* data);
bool encoder_calibration_storage_save(uint8_t joint_id, encoder_calibration_data* data);

#ifdef __cplusplus
}
#endif

#endif
