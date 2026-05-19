/**
 * hrtf_engine.h — Multi-stream HRTF engine for SHARC DSP
 *
 * Optimized for 4 concurrent streams within a single audio callback.
 * Uses: HRTF FIR convolution + ITD + ILD (head shadow)
 * No crossfade path — exponential smoothing on azimuth prevents clicks.
 *
 * Requires: hrtf.h (for HRTF_IR_LEN, hrtf_table, hrtf_az_lut)
 */

#ifndef HRTF_ENGINE_H
#define HRTF_ENGINE_H

#include <stdint.h>
#include <string.h>
#include "hrtf.h"

/* ─── Configuration ─── */
#define HRTF_NUM_STREAMS      4
#define HRTF_MAX_BLOCK_SIZE   64    /* match SYSTEM_BLOCK_SIZE */

/* Smoothing: alpha = NUM/DEN (fixed-point exponential filter on azimuth) */
#define HRTF_SMOOTH_NUM       15
#define HRTF_SMOOTH_DEN       100

/* ITD: max interaural time delay in samples at 48kHz (~0.65ms = 31 samples) */
#define HRTF_MAX_ITD_SAMPLES  31

/* ILD: max head shadow attenuation in dB (mapped to Q31 gain table) */
#define HRTF_ILD_MAX_DB       8

/* ─── Pre-computed ILD gain table (Q31) ─── */
/* ild_gain_q31[i] = round(2^31 * 10^(-ILD_MAX_DB * sin(i°) / 20))
 * for i = 0..90.  Use symmetry: ild_gain_q31[180-i] mirrors.
 * Index by |shadow_angle| where shadow_angle = sin mapping of azimuth */

/* ITD delay table: itd_delay_samples[i] = round(MAX_ITD_SAMPLES * |sin(i°)|)
 * for i = 0..360 (stored per-degree for direct lookup) */

/* ─── Per-stream state ─── */
typedef struct {
    /* FIR circular buffer */
    int32_t delay[HRTF_IR_LEN];
    int     widx;

    /* Azimuth smoothing (centidegrees 0..35999) */
    int32_t smooth_az_cdeg;

    /* Previous table indices (for detecting IR change — informational only) */
    int     prev_idx_lo;
    int     prev_idx_hi;

    /* ITD delay buffers: carry tail samples between blocks */
    int32_t itd_buf_L[HRTF_MAX_ITD_SAMPLES];
    int32_t itd_buf_R[HRTF_MAX_ITD_SAMPLES];
} HrtfStreamState;

/* ─── Pre-computed lookup tables (const, link to flash/L2) ─── */

/*
 * ITD delay per degree: number of samples to delay the far ear.
 * itd_lut[az] gives the signed delay: positive = delay left, negative = delay right.
 * Stored as signed int8 (fits in -31..+31).
 */
static const int8_t hrtf_itd_lut[360] = {
    /* Generated: round(31 * sin(az * pi / 180)) for az=0..359 */
     0,  1,  1,  2,  2,  3,  3,  4,  4,  5,  5,  6,  6,  7,  7,  8,
     9,  9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 16, 16,
    16, 17, 17, 18, 18, 19, 19, 20, 20, 20, 21, 21, 22, 22, 22, 23,
    23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 27,
    28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30,
    30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31,
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 30,
    30, 30, 30, 29, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 27, 27,
    27, 27, 26, 26, 26, 26, 25, 25, 25, 24, 24, 24, 23, 23, 23, 22,
    22, 22, 21, 21, 20, 20, 20, 19, 19, 18, 18, 17, 17, 16, 16, 16,
    15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10,  9,  9,  8,  7,
     7,  6,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  0,  0, -1,
    -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -9, -9,
   -10,-10,-11,-11,-12,-12,-13,-13,-14,-14,-14,-15,-16,-16,-16,-17,
   -17,-18,-18,-19,-19,-20,-20,-20,-21,-21,-22,-22,-22,-23,-23,-23,
   -24,-24,-24,-25,-25,-25,-26,-26,-26,-26,-27,-27,-27,-27,-28,-28,
   -28,-28,-28,-29,-29,-29,-29,-29,-29,-30,-30,-30,-30,-30,-30,-30,
   -30,-30,-31,-31,-31,-31,-31,-31,-31,-31,-31,-31,-31,-31,-31,-31,
   -31,-31,-31,-31,-31,-31,-31,-31,-30,-30,-30,-30,-30,-30,-30,-30,
   -30,-29,-29,-29,-29,-29,-29,-28,-28,-28,-28,-28,-27,-27,-27,-27,
   -26,-26,-26,-26,-25,-25,-25,-24,-24,-24,-23,-23,-23,-22,-22,-22,
   -21,-21,-20,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-16,-15,-14,
   -14,-14,-13,-13,-12,-12,-11,-11,-10,-10, -9, -9, -8, -7, -7, -6,
    -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1,  0
};

/*
 * ILD gain table (Q31): attenuation for the far ear.
 * Index = |itd_lut[az]| (0..31).  Maps delay magnitude to head-shadow gain.
 * gain = 10^(-8dB * (idx/31) / 20) scaled to Q31.
 */
static const int32_t hrtf_ild_gain_q31[32] = {
    /* idx=0: 0dB (no attenuation), idx=31: -8dB */
    2147483647, 2128793994, 2110270039, 2091909786, 2073710281, 2055669013,
    2037783094, 2020050113, 2002467338, 1985032225, 1967742131, 1950594511,
    1933586815, 1916716488, 1899980980, 1883377726, 1866904168, 1850557738,
    1834335870, 1818235993, 1802255535, 1786391919, 1770642571, 1755004903,
    1739476326, 1724054245, 1708736064, 1693519177, 1678400980, 1663378864,
    1648450216, 1633612418
};

/* ─── API ─── */

/**
 * Initialize all stream states to zero/defaults.
 */
static inline void hrtf_engine_init(HrtfStreamState states[HRTF_NUM_STREAMS])
{
    memset(states, 0, sizeof(HrtfStreamState) * HRTF_NUM_STREAMS);
    for (int i = 0; i < HRTF_NUM_STREAMS; i++) {
        states[i].prev_idx_lo = -1;
        states[i].prev_idx_hi = -1;
    }
}

/**
 * Process one stream: mono_in[num_frames] → left_out[], right_out[]
 * with azimuth smoothing, HRTF FIR, ITD, and ILD.
 *
 * target_az_deg: 0..359 target azimuth from IPC
 *
 * Designed to be called 4× per audio callback (once per stream).
 * Cycle budget: ~single FIR pass + ITD shift + ILD multiply.
 * NO crossfade (smoothing prevents clicks).
 */
#pragma optimize_for_speed
static inline void hrtf_engine_process(
    HrtfStreamState *st,
    const int32_t   *mono_in,
    int32_t         *left_out,
    int32_t         *right_out,
    int              num_frames,
    int              target_az_deg)
{
    /* ── 1. Azimuth smoothing (centidegrees, no float) ── */
    int32_t tgt_cdeg = (int32_t)((uint32_t)target_az_deg % 360u) * 100;
    /* SOFA convention: negate */
    tgt_cdeg = (int32_t)((36000 - (tgt_cdeg % 36000)) % 36000);

    int32_t diff = tgt_cdeg - st->smooth_az_cdeg;
    if (diff >  18000) diff -= 36000;
    if (diff < -18000) diff += 36000;
    st->smooth_az_cdeg = (st->smooth_az_cdeg +
                          (diff * HRTF_SMOOTH_NUM) / HRTF_SMOOTH_DEN
                          + 36000) % 36000;

    /* ── 2. Look up interpolated IR ── */
    int az_lo = (int)(st->smooth_az_cdeg / 100);
    int az_hi = (az_lo + 1) % 360;
    int32_t t = (int32_t)((st->smooth_az_cdeg % 100) * 32767 / 100); /* Q15 blend */

    int idx_lo = (int)(uint8_t)hrtf_az_lut[az_lo];
    int idx_hi = (int)(uint8_t)hrtf_az_lut[az_hi];

    const int32_t *ir_L_lo = hrtf_table[idx_lo].ir_L;
    const int32_t *ir_R_lo = hrtf_table[idx_lo].ir_R;
    const int32_t *ir_L_hi = hrtf_table[idx_hi].ir_L;
    const int32_t *ir_R_hi = hrtf_table[idx_hi].ir_R;

    st->prev_idx_lo = idx_lo;
    st->prev_idx_hi = idx_hi;

    /* ── 3. FIR convolution (single pass, interpolated IR) ── */
    for (int f = 0; f < num_frames; f++) {
        st->delay[st->widx] = mono_in[f];

        int64_t acc_L = 0, acc_R = 0;
        int d = st->widx;
        for (int k = 0; k < HRTF_IR_LEN; k++) {
            int32_t x = st->delay[d];
            int32_t h_L = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
            int32_t h_R = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
            acc_L += (int64_t)x * h_L;
            acc_R += (int64_t)x * h_R;
            if (--d < 0) d = HRTF_IR_LEN - 1;
        }

        if (++(st->widx) >= HRTF_IR_LEN) st->widx = 0;

        left_out[f]  = (int32_t)(acc_L >> 31);
        right_out[f] = (int32_t)(acc_R >> 31);
    }

    /* ── 4. ITD: delay the far ear using persistent carry buffer ── */
    int8_t itd_signed = hrtf_itd_lut[az_lo];
    if (itd_signed != 0) {
        int itd_abs = (itd_signed > 0) ? itd_signed : -itd_signed;
        int32_t *buf    = (itd_signed > 0) ? st->itd_buf_L : st->itd_buf_R;
        int32_t *target = (itd_signed > 0) ? left_out      : right_out;

        /* Shift: first itd_abs samples come from carry buffer */
        int d = (itd_abs < num_frames) ? itd_abs : num_frames;
        int32_t tmp[HRTF_MAX_BLOCK_SIZE];
        memcpy(tmp, target, num_frames * sizeof(int32_t));

        /* Output: [buf tail | signal shifted] */
        for (int i = 0; i < d; i++)
            target[i] = buf[i];
        if (itd_abs < num_frames) {
            /* memmove would work but manual copy avoids overlap issues */
            for (int i = itd_abs; i < num_frames; i++)
                target[i] = tmp[i - itd_abs];
        }

        /* Save tail into carry buffer for next block */
        int tail = (itd_abs < num_frames) ? itd_abs : num_frames;
        for (int i = 0; i < tail; i++)
            buf[i] = tmp[num_frames - tail + i];
    }

    /* ── 5. ILD: attenuate far ear (head shadow) ── */
    int ild_idx = (itd_signed > 0) ? itd_signed : -itd_signed;
    if (ild_idx > 0) {
        int32_t gain = hrtf_ild_gain_q31[ild_idx];
        int32_t *far_ear = (itd_signed > 0) ? left_out : right_out;
        for (int f = 0; f < num_frames; f++) {
            far_ear[f] = (int32_t)(((int64_t)far_ear[f] * gain) >> 31);
        }
    }
}

#endif /* HRTF_ENGINE_H */
