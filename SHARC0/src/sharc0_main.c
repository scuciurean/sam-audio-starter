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
 * IPC parameter IDs:
 *   id 0 = stem 0 (vocals) azimuth  (0..359 deg)
 *   id 1 = stem 1 (drums)  azimuth
 *   id 2 = stem 2 (bass)   azimuth
 *   id 3 = stem 3 (other)  azimuth
 **********************************************************************/
#define HRTF_NUM_STEMS    4   /* total stem slots (max) */
#define HRTF_ACTIVE_STEMS 4   /* stems currently processed (1 = single ch0, 4 = full 4ch WAV) */
#define HRTF_CONV_LEN     96  /* taps used (96 = 99.3% IR energy, ~half the MACs of full 218) */
#define SMOOTH_ALPHA_NUM  15
#define SMOOTH_ALPHA_DEN  100
#define SYSTEM_BLOCK_SIZE 64   /* max frames per IPC buffer */

/* Per-stem gain in Q15 (32768 = unity). Already includes /4 headroom for 4-stem mix.
   Tune individual values to balance loudness. Max safe sum = 4 × 8192 = 32768 (unity). */
static const int32_t stem_gain_q15[HRTF_NUM_STEMS] = {
    32768,   /* stem 0: vocals  — 0.50 */
    32768,   /* stem 1: drums   — 0.50 */
    32768,   /* stem 2: bass    — 0.50 */
    32768,   /* stem 3: other   — 0.50 */
};

/* Target azimuths written by IPC handler, read by audio thread */
static volatile uint32_t hrtf_target_az_deg[HRTF_NUM_STEMS];

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
        const int32_t *rd = dd + widx;
        int64_t acc_L = 0, acc_R = 0;
        for (int k = 0; k < HRTF_CONV_LEN; k++) {
            acc_L += ((int64_t)rd[k] * h_L[k]) >> 16;
            acc_R += ((int64_t)rd[k] * h_R[k]) >> 16;
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

    /* Mix: apply per-stem gain then sum → stereo output. */
    int out_stride = (int)sink->numChannels;
    for (uint32_t f = 0; f < n; f++) {
        int64_t mix_L = 0, mix_R = 0;
        for (int s = 0; s < HRTF_ACTIVE_STEMS; s++) {
            mix_L += ((int64_t)stem_L[s][f] * stem_gain_q15[s]) >> 15;
            mix_R += ((int64_t)stem_R[s][f] * stem_gain_q15[s]) >> 15;
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
            /* id 0..3 = azimuth for stem 0..3 (vocals, drums, bass, other) */
            if (msg->parameter.id < HRTF_NUM_STEMS) {
                hrtf_target_az_deg[msg->parameter.id] = msg->parameter.value;
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
