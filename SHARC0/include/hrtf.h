/* Auto-generated from cipic_003.sofa -- do not edit by hand. */
#ifndef HRTF_DATA_H
#define HRTF_DATA_H

#include <stdint.h>

#define HRTF_IR_LEN  218
#define HRTF_NUM_AZ  50

typedef struct {
    int16_t  az_deg;
    int32_t  ir_L[HRTF_IR_LEN];
    int32_t  ir_R[HRTF_IR_LEN];
} HRTF_Entry;

extern const HRTF_Entry hrtf_table[HRTF_NUM_AZ];
extern const int8_t     hrtf_az_lut[360];

#endif /* HRTF_DATA_H */
