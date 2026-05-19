/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard includes. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* CCES includes */
#include <services/int/adi_sec.h>
#define DO_CYCLE_COUNTS
#include <cycle_count.h>

/* Simple service includes */
#include "sae.h"

/* IPC includes */
#include "ipc.h"
#include "hrtf.h"
#include "hrtf_engine.h"

SAE_CONTEXT *saeContext = NULL;
IPC_MSG_AUDIO *streamInfo[IPC_STREAM_ID_MAX];
SAE_MSG_BUFFER *cyclesMsg = NULL;

/***********************************************************************
 * Per-stem HRTF state
 *
 * 4 stems arrive on channels 0-3 of SHARC0_IN.  Each is processed
 * independently through its own HRTF filter, then all 4 L outputs are
 * summed and all 4 R outputs are summed to produce the stereo mix.
 *
 * IPC parameter IDs (id 0..3 = stem 0..3: vocals, drums, bass, other):
 *   ang  : azimuth  0..359 deg
 *   elev : elevation -90..+90 deg  (reserved for future use)
 *   dist : distance in cm           (reserved for future use)
 *   gain : linear gain Q15 (32768 = unity)
 **********************************************************************/
#define HRTF_NUM_STEMS    4   /* total stem slots (max) */
#define HRTF_ACTIVE_STEMS 4   /* stems currently processed (1 = single ch0, 4 = full 4ch WAV) */
#define HRTF_CONV_LEN     96  /* taps used (96 = 99.3% IR energy, ~half the MACs of full 218) */
#define SMOOTH_ALPHA_NUM  15
#define SMOOTH_ALPHA_DEN  100
#define SYSTEM_BLOCK_SIZE 64   /* max frames per IPC buffer */

/* Per-stem spatial parameters — written by IPC handler, read by audio thread */
static volatile uint32_t hrtf_target_az_deg[HRTF_NUM_STEMS];
static volatile int32_t  hrtf_target_elev[HRTF_NUM_STEMS];   /* -90..+90 deg */
static volatile uint32_t hrtf_target_dist[HRTF_NUM_STEMS];   /* cm, reference = 100 cm */
static volatile uint32_t stem_gain_q15[HRTF_NUM_STEMS];      /* Q15: 32768 = unity */

/* Elevation gain: Q15 cosine curve, indexed by |elev| in degrees (0..90).
   hrtf_elev_gain_q15[i] = round(32768 * cos(i * pi / 180))
   0° → 32768 (unity), 90° → 0 (silence). */
static const int32_t hrtf_elev_gain_q15[91] = {
    32768, 32763, 32748, 32723, 32688, 32643, 32588, 32524, 32449, 32365,
    32270, 32166, 32052, 31928, 31795, 31651, 31499, 31336, 31164, 30983,
    30792, 30592, 30382, 30163, 29935, 29698, 29452, 29197, 28932, 28660,
    28378, 28088, 27789, 27482, 27166, 26842, 26510, 26170, 25822, 25466,
    25102, 24730, 24351, 23965, 23571, 23170, 22763, 22348, 21926, 21498,
    21063, 20622, 20174, 19720, 19261, 18795, 18324, 17847, 17364, 16877,
    16384, 15886, 15384, 14876, 14365, 13848, 13328, 12803, 12275, 11743,
    11207, 10668, 10126,  9580,  9032,  8481,  7927,  7371,  6813,  6252,
     5690,  5126,  4560,  3993,  3425,  2856,  2286,  1715,  1144,   572,
        0
};

/* High-shelf biquad at 6 kHz, 48 kHz Fs, Q=0.707.
 * Direct Form I coefficients in Q30.
 * Order: { b0, b1, b2, -a1, -a2 }  (a1/a2 stored negated for add-only feedback)
 * Indices: 0=-6dB, 1=-3dB, 2=0dB(flat), 3=+3dB, 4=+6dB */
typedef struct { int32_t b0, b1, b2, na1, na2; } ShelfCoeffs;
static const ShelfCoeffs hrtf_shelf_table[5] = {
    /* -6dB */ {  646225191, -505127663,  182059008, 1168674952, -418089663 },
    /* -3dB */ {  832854079, -719687507,  255524829, 1092635147, -387584724 },
    /* +0dB */ { 1073741824,-1012333604,  357914089, 1012333604, -357914089 },
    /* +3dB */ { 1384301924,-1408659794,  499686486,  927843899, -329430692 },
    /* +6dB */ { 1784086292,-1941823364,  694680993,  839299839, -302501936 },
};

/* Maps elevation degree (offset by 45, so index 0 = -45°, index 45 = 0°, index 135 = +90°)
 * to shelf table index 0..4. */
static const uint8_t hrtf_shelf_lut[136] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
};

/* Per-stem biquad state: two delay taps (x history for b-side, y history for a-side).
 * One filter instance per channel (L and R share the same spectral shaping). */
typedef struct { int32_t x1, x2, y1, y2; } BiquadState;
static BiquadState shelf_state_L[HRTF_NUM_STEMS];
static BiquadState shelf_state_R[HRTF_NUM_STEMS];

/* Reflection delay tap: ELEV_REFL_DELAY samples at 48 kHz ≈ 2 ms (ceiling bounce).
 * Read from the stem's own FIR delay buffer (v2_dd) — no extra storage needed. */
#define ELEV_REFL_DELAY  96   /* samples */
#define ELEV_REFL_GAIN   8192 /* Q15: -12 dB */

static void hrtf_params_init(void)
{
    for (int s = 0; s < HRTF_NUM_STEMS; s++) {
        hrtf_target_az_deg[s] = 0;
        hrtf_target_elev[s]   = 0;
        hrtf_target_dist[s]   = 0;    /* 0=nearest (unity gain) default */
        stem_gain_q15[s]      = 32768;
    }
}

/* Per-stem filter state */
static int32_t hrtf_delay[HRTF_NUM_STEMS][HRTF_IR_LEN];
static int     hrtf_widx[HRTF_NUM_STEMS];

static int32_t hrtf_smooth_az_cdeg[HRTF_NUM_STEMS]; /* centidegrees 0..35999 */

static int     hrtf_prev_idx_lo[HRTF_NUM_STEMS];
static int     hrtf_prev_idx_hi[HRTF_NUM_STEMS];

static int32_t hrtf_prev_ir_L[HRTF_NUM_STEMS][HRTF_IR_LEN];
static int32_t hrtf_prev_ir_R[HRTF_NUM_STEMS][HRTF_IR_LEN];

static void hrtf_init(void)
{
    hrtf_params_init();
    for (int s = 0; s < HRTF_NUM_STEMS; s++) {
        hrtf_prev_idx_lo[s] = -1;
        hrtf_prev_idx_hi[s] = -1;
    }
}

static inline int hrtf_index(int az_deg)
{
    az_deg = ((az_deg % 360) + 360) % 360;
    return (int)(uint8_t)hrtf_az_lut[az_deg];
}

static inline int32_t clamp32(int64_t v)
{
    if (v >  0x7FFFFFFF) return  0x7FFFFFFF;
    if (v < -0x80000000) return (int32_t)(-0x80000000);
    return (int32_t)v;
}

/* Per-stem reflection ring buffer: holds ELEV_REFL_DELAY mono samples
 * (pre-HRTF signal).  Written by hrtf_apply_stem_v2's caller before
 * the shelf function runs.  96 samples × 4 stems × 4 bytes = 1.5 KB. */
static int32_t refl_buf[HRTF_NUM_STEMS][ELEV_REFL_DELAY];
static int     refl_widx[HRTF_NUM_STEMS];

/*
 * High-shelf biquad + early reflection for elevation emulation.
 *
 * Operates in-place on out_L/out_R after hrtf_apply_stem_v2.
 * mono_in[] is the pre-HRTF mono input for this stem (used for reflection).
 *
 * Shelf — Direct Form I, Q30, 5 muls/sample per channel:
 *   y[n] = (b0*x[n] + b1*x[n-1] + b2*x[n-2] + na1*y[n-1] + na2*y[n-2]) >> 30
 *   Coefficients selected from hrtf_shelf_table via hrtf_shelf_lut[elev+45].
 *   Below horizon: high-freq cut (darker).  Above: high-freq boost (brighter).
 *
 * Reflection — positive elevation only:
 *   Pre-HRTF mono is written into refl_buf every frame.
 *   ELEV_REFL_DELAY samples later (~2 ms) it is read back at -12 dB and
 *   added to both L and R — simulates a ceiling bounce.
 *   Gain scales linearly with elevation: 0 at horizon, full at +90°.
 */
#pragma optimize_for_speed
static void elev_shelf_and_reflection(int stem,
                                       const int32_t *mono_in, int in_stride,
                                       int32_t *out_L, int32_t *out_R,
                                       uint32_t n_frames,
                                       int32_t elev)
{
    if (elev < -45) elev = -45;
    if (elev >  90) elev =  90;

    /* Shelf coefficients */
    const ShelfCoeffs *c = &hrtf_shelf_table[hrtf_shelf_lut[elev + 45]];
    int32_t b0 = c->b0, b1 = c->b1, b2 = c->b2, na1 = c->na1, na2 = c->na2;

    BiquadState *sL = &shelf_state_L[stem];
    BiquadState *sR = &shelf_state_R[stem];
    int32_t xL1 = sL->x1, xL2 = sL->x2, yL1 = sL->y1, yL2 = sL->y2;
    int32_t xR1 = sR->x1, xR2 = sR->x2, yR1 = sR->y1, yR2 = sR->y2;

    /* Reflection gain: 0 at horizon, ELEV_REFL_GAIN at +90°, linear.
     * Q15: refl_g = ELEV_REFL_GAIN * elev / 90  (0 when elev <= 0) */
    int32_t refl_g = (elev > 0) ? (int32_t)((ELEV_REFL_GAIN * (uint32_t)elev) / 90u) : 0;

    int32_t *rb   = refl_buf[stem];
    int      rw   = refl_widx[stem];

    for (uint32_t f = 0; f < n_frames; f++) {
        int32_t xL = out_L[f];
        int32_t xR = out_R[f];

        /* --- Shelf biquad L --- */
        int64_t accL = (int64_t)b0*xL + (int64_t)b1*xL1 + (int64_t)b2*xL2
                     + (int64_t)na1*yL1 + (int64_t)na2*yL2;
        int32_t yL = (int32_t)(accL >> 30);
        xL2 = xL1; xL1 = xL; yL2 = yL1; yL1 = yL;

        /* --- Shelf biquad R --- */
        int64_t accR = (int64_t)b0*xR + (int64_t)b1*xR1 + (int64_t)b2*xR2
                     + (int64_t)na1*yR1 + (int64_t)na2*yR2;
        int32_t yR = (int32_t)(accR >> 30);
        xR2 = xR1; xR1 = xR; yR2 = yR1; yR1 = yR;

        /* --- Reflection tap --- */
        int32_t mono = mono_in[f * in_stride];
        int32_t tap  = rb[rw];   /* sample from ELEV_REFL_DELAY ago */
        rb[rw] = mono;
        if (++rw >= ELEV_REFL_DELAY) rw = 0;
        int32_t refl = (int32_t)(((int64_t)tap * refl_g) >> 15);

        out_L[f] = yL + refl;
        out_R[f] = yR + refl;
    }

    sL->x1 = xL1; sL->x2 = xL2; sL->y1 = yL1; sL->y2 = yL2;
    sR->x1 = xR1; sR->x2 = xR2; sR->y1 = yR1; sR->y2 = yR2;
    refl_widx[stem] = rw;
}

/*
 * Stripped-down HRTF: no crossfade, no smoothing — just FIR convolution at the
 * current azimuth. IR is pre-interpolated once per chunk. Doubled delay buffer
 * for branch-free inner loop. Use this when CPU budget is tight.
 */
#pragma optimize_for_speed
static void hrtf_apply_stem_v2(int stem,
                                const int32_t * restrict in, int in_stride,
                                int32_t * restrict out_L, int32_t * restrict out_R,
                                uint32_t n_frames)
{
    /* CIPIC: 0=front, counter-clockwise positive. GUI sends clockwise degrees.
       Mirror so GUI front=0 → HRTF front=0, GUI right → HRTF right. */
    int az = (360 - (int)(hrtf_target_az_deg[stem] % 360)) % 360;
    int idx = hrtf_index(az);

    /* Snap to nearest table entry, use only first HRTF_CONV_LEN taps (99.3% energy) */
    static int32_t v2_dd[HRTF_NUM_STEMS][HRTF_CONV_LEN * 2];
    const int32_t *h_L = hrtf_table[idx].ir_L;
    const int32_t *h_R = hrtf_table[idx].ir_R;

    int32_t *dd  = v2_dd[stem];
    int      widx = hrtf_widx[stem];

    for (uint32_t frame = 0; frame < n_frames; frame++) {
        int32_t mono = in[frame * in_stride];
        dd[widx] = dd[widx + HRTF_CONV_LEN] = mono;
        /* Read backwards from widx: rd[0]=newest, rd[1]=one-sample-old, ...
           The doubled buffer lets us walk forward through memory while
           logically reading the delay line from newest to oldest. */
        const int32_t *rd = dd + HRTF_CONV_LEN + widx;
        int64_t acc_L = 0, acc_R = 0;
        for (int k = 0; k < HRTF_CONV_LEN; k++) {
            acc_L += ((int64_t)rd[-k] * h_L[k]) >> 16;
            acc_R += ((int64_t)rd[-k] * h_R[k]) >> 16;
        }
        if (++widx >= HRTF_CONV_LEN) widx = 0;
        out_L[frame] = (int32_t)(acc_L >> 20);
        out_R[frame] = (int32_t)(acc_R >> 20);
    }

    hrtf_widx[stem] = widx;
}

/*
 * Process one stem for one audio chunk.
 *
 * stem      : stem index 0..HRTF_NUM_STEMS-1
 * in        : pointer to first sample of this stem (interleaved, stride=in_stride)
 * in_stride : src->numChannels
 * out_L/R   : output arrays of n_frames int32 samples
 */
#pragma optimize_for_speed
static void hrtf_apply_stem(int stem,
                             const int32_t *in, int in_stride,
                             int32_t *out_L, int32_t *out_R,
                             uint32_t n_frames)
{
    /* --- Azimuth smoothing (one step per chunk, block-rate) --- */
    int32_t tgt_cdeg = (int32_t)hrtf_target_az_deg[stem] * 100;
    tgt_cdeg = (int32_t)((36000 - (tgt_cdeg % 36000)) % 36000);

    int32_t diff = tgt_cdeg - hrtf_smooth_az_cdeg[stem];
    if (diff >  18000) diff -= 36000;
    if (diff < -18000) diff += 36000;
    hrtf_smooth_az_cdeg[stem] = (hrtf_smooth_az_cdeg[stem] +
                                  (diff * SMOOTH_ALPHA_NUM) / SMOOTH_ALPHA_DEN
                                 + 36000) % 36000;

    int az_lo = (int)(hrtf_smooth_az_cdeg[stem] / 100);
    int az_hi = (az_lo + 1) % 360;
    int32_t t = (int32_t)((hrtf_smooth_az_cdeg[stem] % 100) * 32767 / 100);

    int idx_lo = hrtf_index(az_lo);
    int idx_hi = hrtf_index(az_hi);

    bool ir_changed = (idx_lo != hrtf_prev_idx_lo[stem] ||
                       idx_hi != hrtf_prev_idx_hi[stem]);
    bool first_run  = (hrtf_prev_idx_lo[stem] == -1);

    const int32_t *ir_L_lo = hrtf_table[idx_lo].ir_L;
    const int32_t *ir_R_lo = hrtf_table[idx_lo].ir_R;
    const int32_t *ir_L_hi = hrtf_table[idx_hi].ir_L;
    const int32_t *ir_R_hi = hrtf_table[idx_hi].ir_R;

    int32_t *delay = hrtf_delay[stem];
    int      widx  = hrtf_widx[stem];

    if (ir_changed && !first_run) {
        /* Crossfade: old IR on live delay (continuing state),
           new IR on a zeroed temp delay (fresh zi, matching Python lfilter approach).
           One buffer per stem to avoid aliasing when multiple stems crossfade. */
        static int32_t new_delay[HRTF_NUM_STEMS][HRTF_IR_LEN];
        int new_widx = 0;
        memset(new_delay[stem], 0, sizeof(new_delay[stem]));

        for (uint32_t frame = 0; frame < n_frames; frame++) {
            int32_t mono = in[frame * in_stride];

            /* Old IR on live delay */
            delay[widx] = mono;
            int64_t acc_L_old = 0, acc_R_old = 0;
            int d = widx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x = delay[d];
                acc_L_old += ((int64_t)x * hrtf_prev_ir_L[stem][k]) >> 16;
                acc_R_old += ((int64_t)x * hrtf_prev_ir_R[stem][k]) >> 16;
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }
            if (++widx >= HRTF_IR_LEN) widx = 0;

            /* New IR on zeroed temp delay */
            new_delay[stem][new_widx] = mono;
            int64_t acc_L_new = 0, acc_R_new = 0;
            d = new_widx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x   = new_delay[stem][d];
                int32_t h_L = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
                int32_t h_R = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
                acc_L_new += ((int64_t)x * h_L) >> 16;
                acc_R_new += ((int64_t)x * h_R) >> 16;
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }
            if (++new_widx >= HRTF_IR_LEN) new_widx = 0;

            int32_t fade  = (int32_t)(((int64_t)(frame + 1) * 32767) / n_frames);
            int32_t l_old = (int32_t)(acc_L_old >> 15);
            int32_t r_old = (int32_t)(acc_R_old >> 15);
            int32_t l_new = (int32_t)(acc_L_new >> 15);
            int32_t r_new = (int32_t)(acc_R_new >> 15);

            out_L[frame] = l_old + (int32_t)(((int64_t)(l_new - l_old) * fade) >> 15);
            out_R[frame] = r_old + (int32_t)(((int64_t)(r_new - r_old) * fade) >> 15);
        }

        /* Adopt new delay as live state */
        memcpy(delay, new_delay[stem], HRTF_IR_LEN * sizeof(int32_t));
        widx = new_widx;
    } else {
        for (uint32_t frame = 0; frame < n_frames; frame++) {
            int32_t mono = in[frame * in_stride];

            delay[widx] = mono;
            int64_t acc_L = 0, acc_R = 0;
            int d = widx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x   = delay[d];
                int32_t h_L = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
                int32_t h_R = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
                acc_L += ((int64_t)x * h_L) >> 16;
                acc_R += ((int64_t)x * h_R) >> 16;
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }
            if (++widx >= HRTF_IR_LEN) widx = 0;

            out_L[frame] = (int32_t)(acc_L >> 15);
            out_R[frame] = (int32_t)(acc_R >> 15);
        }
    }

    hrtf_widx[stem] = widx;

    if (ir_changed || first_run) {
        for (int k = 0; k < HRTF_IR_LEN; k++) {
            hrtf_prev_ir_L[stem][k] = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
            hrtf_prev_ir_R[stem][k] = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
        }
        hrtf_prev_idx_lo[stem] = idx_lo;
        hrtf_prev_idx_hi[stem] = idx_hi;
    }
}

/*
 * Apply HRTF to all stems and mix into stereo output.
 * src: interleaved 4-channel input (ch0=vocals, ch1=drums, ch2=bass, ch3=other)
 * sink: 2-channel output (ch0=L, ch1=R)
 */
#pragma optimize_for_speed
static void hrtf_apply(IPC_MSG_AUDIO *src, IPC_MSG_AUDIO *sink)
{
    static int32_t stem_L[HRTF_NUM_STEMS][SYSTEM_BLOCK_SIZE];
    static int32_t stem_R[HRTF_NUM_STEMS][SYSTEM_BLOCK_SIZE];

    uint32_t n = src->numFrames;
    int      in_stride = (int)src->numChannels;

    for (int s = 0; s < HRTF_ACTIVE_STEMS; s++) {
        hrtf_apply_stem_v2(s,
            src->data + s,   /* channel s of interleaved stream */
            in_stride,
            stem_L[s], stem_R[s], n);
    }

    /* Elevation: high-shelf spectral shaping + ceiling reflection.
     * Distance + cosine level: combined in one multiply pass after. */
    for (int s = 0; s < HRTF_ACTIVE_STEMS; s++) {
        int32_t elev = hrtf_target_elev[s];

        elev_shelf_and_reflection(s,
            src->data + s, in_stride,
            stem_L[s], stem_R[s], n, elev);

        /* Level cues: cosine elevation gain × distance gain, one pass. */
        if (elev < -45) elev = -45;
        if (elev >  90) elev =  90;
        int32_t eg = hrtf_elev_gain_q15[elev < 0 ? -elev : elev];

        uint32_t dist = hrtf_target_dist[s];
        if (dist > 100) dist = 100;
        int32_t dg = (int32_t)((100u - dist) * 32768u / 100u);

        int32_t cg = (int32_t)(((int64_t)eg * dg) >> 15);

        for (uint32_t f = 0; f < n; f++) {
            stem_L[s][f] = (int32_t)(((int64_t)stem_L[s][f] * cg) >> 15);
            stem_R[s][f] = (int32_t)(((int64_t)stem_R[s][f] * cg) >> 15);
        }
    }

    /* Mix: apply per-stem gain then sum → stereo output. */
    int out_stride = (int)sink->numChannels;
    for (uint32_t f = 0; f < n; f++) {
        int64_t mix_L = 0, mix_R = 0;
        for (int s = 0; s < HRTF_ACTIVE_STEMS; s++) {
            int32_t g = (int32_t)stem_gain_q15[s];
            mix_L += ((int64_t)stem_L[s][f] * g) >> 15;
            mix_R += ((int64_t)stem_R[s][f] * g) >> 15;
        }
        sink->data[f * out_stride + 0] = (int32_t)mix_L;
        sink->data[f * out_stride + 1] = (int32_t)mix_R;
    }
}

/***********************************************************************
 * Audio functions
 **********************************************************************/

/*
 * In these functions, IN and OUT are relative to the SHARC.  This
 * code uses src and sink to help minimize confusion.  In all cases,
 * src buffers are copied to sink buffers.
 */
#pragma optimize_for_speed
static void processAudio(IPC_MSG_PROCESS_AUDIO *process)
{
    uint8_t clockDomain = process->clockDomain;
    IPC_MSG_AUDIO *src, *sink, *stream;
    unsigned i;
    cycle_t startCycles;
    cycle_t finalCycles;

    START_CYCLE_COUNT(startCycles);

    src = streamInfo[IPC_STREAMID_SHARC0_IN];
    sink = streamInfo[IPC_STREAMID_SHARC0_OUT];

    if ((src == NULL) || (sink == NULL)) {
        return;
    }
    if (src->clockDomain != clockDomain) {
        return;
    }
    if (sink->clockDomain != clockDomain) {
        return;
    }

    if (src->numFrames != sink->numFrames) {
        return;
    }
    if (src->wordSize != sink->wordSize) {
        return;
    }
    if (src->wordSize != sizeof(int32_t)) {
        return;
    }
    // if (src->numChannels < HRTF_ACTIVE_STEMS) {
    //     return;
    // }

    hrtf_apply(src, sink);

    /* Clear the contents of src/in buffer */
    memset(src->data, 0,
                src->numChannels * src->numFrames * src->wordSize);

    /* Invalidate all streams associated with this clock domain */
    for (i = 0; i < IPC_STREAM_ID_MAX; i++) {
        stream = streamInfo[i];
        if (stream && (stream->clockDomain == clockDomain)) {
            streamInfo[i] = NULL;
        }
    }

    STOP_CYCLE_COUNT(finalCycles, startCycles);

    if (cyclesMsg && (clockDomain < IPC_CYCLE_DOMAIN_MAX)) {
        IPC_MSG *msg = sae_getMsgBufferPayload(cyclesMsg);
        msg->cycles.cycles[clockDomain] = finalCycles;
    }
}

static void newAudio(IPC_MSG_AUDIO *audio)
{
    bool clear = false;
    bool unknown = false;

    switch (audio->streamID) {
        case IPC_STREAMID_SHARC0_IN:
            break;
        case IPC_STREAMID_SHARC0_OUT:
            clear = true;
            break;
        default:
            unknown = true;
            break;
    }

    if (!unknown) {
        streamInfo[audio->streamID] = audio;
        if (clear) {
            memset(audio->data, 0,
                audio->numChannels * audio->numFrames * audio->wordSize);
        }
    }
}

/***********************************************************************
 * Application IPC functions
 **********************************************************************/
SAE_RESULT ipcToCore(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *ipcBuffer,
    SAE_CORE_IDX core)
{
    SAE_RESULT result;

    result = sae_sendMsgBuffer(saeContext, ipcBuffer, core, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, ipcBuffer);
    }

    return(result);
}

SAE_RESULT quickIpcToCore(SAE_CONTEXT *saeContext, enum IPC_TYPE type,
    SAE_CORE_IDX core)
{
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg;

    ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    if (ipcBuffer) {
        msg->type = type;
        result = ipcToCore(saeContext, ipcBuffer, core);
    } else {
        result = SAE_RESULT_ERROR;
    }

    return(result);
}

static void ipcMsgRx(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *buffer,
    void *payload, void *usrPtr)
{
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg = (IPC_MSG *)payload;
    IPC_MSG *replyMsg;

    /* Process the message */
    switch (msg->type) {
        case IPC_TYPE_PING:
            ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*replyMsg), (void **)&replyMsg);
            replyMsg->type = IPC_TYPE_PING;
            result = sae_sendMsgBuffer(saeContext, ipcBuffer, IPC_CORE_ARM, true);
            if (result != SAE_RESULT_OK) {
                sae_unRefMsgBuffer(saeContext, ipcBuffer);
            }
            break;
        case IPC_TYPE_PROCESS_AUDIO:
            processAudio((IPC_MSG_PROCESS_AUDIO *)&msg->process);
            break;
        case IPC_TYPE_AUDIO:
            newAudio((IPC_MSG_AUDIO *)&msg->audio);
            break;
        case IPC_TYPE_CYCLES:
            if (cyclesMsg) {
                sae_refMsgBuffer(saeContext, cyclesMsg);
                result = sae_sendMsgBuffer(saeContext, cyclesMsg, IPC_CORE_ARM, true);
                if (result != SAE_RESULT_OK) {
                    sae_unRefMsgBuffer(saeContext, cyclesMsg);
                }
            }
            break;
        case IPC_TYPE_PARAMETER:
            /* id 0..3 = stem 0..3 (vocals, drums, bass, other) */
            if (msg->parameter.id < HRTF_NUM_STEMS) {
                uint8_t s = msg->parameter.id;
                hrtf_target_az_deg[s] = msg->parameter.ang;
                hrtf_target_elev[s]   = msg->parameter.elev;
                hrtf_target_dist[s]   = msg->parameter.dist;
                stem_gain_q15[s]      = (uint32_t)(msg->parameter.gain * 32768u / 100u);
            }
            break;
        default:
            break;
    }

    /* Done with the message so decrement the ref count */
    result = sae_unRefMsgBuffer(saeContext, buffer);
}

int main(int argc, char **argv)
{
    SAE_RESULT ok = SAE_RESULT_OK;
    IPC_MSG *msg;

    /* Initialize the SEC */
    adi_sec_Init();

    /* Initialize per-stem HRTF state */
    hrtf_init();

    /* Initialize the SHARC Audio Engine */
    sae_initialize(&saeContext, IPC_CORE_SHARC0, false);

    /* Create a persistent message for cycle counts */
    cyclesMsg = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    if (cyclesMsg) {
        msg->type = IPC_TYPE_CYCLES;
        msg->cycles.core = IPC_CORE_SHARC0;
        msg->cycles.max = IPC_CYCLE_DOMAIN_MAX;
    }

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(saeContext, ipcMsgRx, NULL);

    /* Tell the ARM we're ready */
    quickIpcToCore(saeContext, IPC_TYPE_SHARC0_READY, IPC_CORE_ARM);

    while(1) {
        asm("nop;");
    };
}
