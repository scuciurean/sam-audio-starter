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

//#define SMOOTH_ALPHA_NUM  15
//#define SMOOTH_ALPHA_DEN  100

///* Current smoothed azimuth in 1/100ths of a degree (avoids float) */
//static int32_t hrtf_smooth_az_cdeg = 0;

/* Target azimuth set via IPC (0..359 degrees) */

static int32_t hrtf_delay[HRTF_IR_LEN];      /* active delay line */
static int32_t hrtf_delay_prev[HRTF_IR_LEN]; /* previous IR's delay line for crossfade */
static int      hrtf_widx = 0;

#define SMOOTH_ALPHA_NUM  15
#define SMOOTH_ALPHA_DEN  100

/* Current smoothed azimuth in 1/100ths of a degree (avoids float) */
static int32_t hrtf_smooth_az_cdeg = 0; /* centidegrees, 0..35999 */

/* Target azimuth set via IPC (0..359 degrees) */
static volatile uint32_t hrtf_target_az_deg = 0;

/* Write index into hrtf_delay_prev at the moment of the snapshot */
static int hrtf_prev_widx = 0;

/* Previous table indices — detect IR table boundary crossings */
static int hrtf_prev_idx_lo = -1;
static int hrtf_prev_idx_hi = -1;

/* Previous interpolated IR coefficients (for crossfade) */
static int32_t hrtf_prev_ir_L[HRTF_IR_LEN];
static int32_t hrtf_prev_ir_R[HRTF_IR_LEN];

static inline int hrtf_index(int az_deg)
{
    az_deg = ((az_deg % 360) + 360) % 360;
    return (int)(uint8_t)hrtf_az_lut[az_deg];
}

#pragma optimize_for_speed
static void hrtf_apply(IPC_MSG_AUDIO *src, IPC_MSG_AUDIO *sink)
{
    unsigned frame;
    int32_t *in, *out;

    int32_t tgt_cdeg = (int32_t)hrtf_target_az_deg * 100;
    tgt_cdeg = (int32_t)((36000 - (tgt_cdeg % 36000)) % 36000);

    int32_t diff = tgt_cdeg - hrtf_smooth_az_cdeg;
    if (diff >  18000) diff -= 36000;
    if (diff < -18000) diff += 36000;
    hrtf_smooth_az_cdeg = (hrtf_smooth_az_cdeg +
                            (diff * SMOOTH_ALPHA_NUM) / SMOOTH_ALPHA_DEN
                           + 36000) % 36000;

    int az_lo = (int)(hrtf_smooth_az_cdeg / 100);
    int az_hi = (az_lo + 1) % 360;
    int32_t t = (int32_t)((hrtf_smooth_az_cdeg % 100) * 32767 / 100);

    int idx_lo = hrtf_index(az_lo);
    int idx_hi = hrtf_index(az_hi);

    bool ir_changed = (idx_lo != hrtf_prev_idx_lo || idx_hi != hrtf_prev_idx_hi);
    bool first_run  = (hrtf_prev_idx_lo == -1);

    const int32_t *ir_L_lo = hrtf_table[idx_lo].ir_L;
    const int32_t *ir_R_lo = hrtf_table[idx_lo].ir_R;
    const int32_t *ir_L_hi = hrtf_table[idx_hi].ir_L;
    const int32_t *ir_R_hi = hrtf_table[idx_hi].ir_R;

    in  = src->data;
    out = sink->data;
    uint32_t n_frames = src->numFrames;

    if (ir_changed && !first_run) {
        /* prev_ridx tracks the read position into the frozen hrtf_delay_prev snapshot */
        int prev_ridx = hrtf_prev_widx;

        for (frame = 0; frame < n_frames; frame++) {
            int32_t mono = in[frame * src->numChannels];

            hrtf_delay[hrtf_widx] = mono;

            int64_t acc_L_new = 0, acc_R_new = 0;
            int d = hrtf_widx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x   = hrtf_delay[d];
                int32_t h_L = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
                int32_t h_R = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
                acc_L_new += (int64_t)x * h_L;
                acc_R_new += (int64_t)x * h_R;
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }

            int64_t acc_L_old = 0, acc_R_old = 0;
            d = prev_ridx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x   = hrtf_delay_prev[d];
                acc_L_old += (int64_t)x * hrtf_prev_ir_L[k];
                acc_R_old += (int64_t)x * hrtf_prev_ir_R[k];
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }

            if (++hrtf_widx >= HRTF_IR_LEN) hrtf_widx = 0;
            if (++prev_ridx >= HRTF_IR_LEN) prev_ridx = 0;

            int32_t fade  = (int32_t)(((int64_t)(frame + 1) * 32767) / n_frames);
            int32_t l_old = (int32_t)(acc_L_old >> 31);
            int32_t r_old = (int32_t)(acc_R_old >> 31);
            int32_t l_new = (int32_t)(acc_L_new >> 31);
            int32_t r_new = (int32_t)(acc_R_new >> 31);

            out[frame * sink->numChannels + 0] = l_old + (int32_t)(((int64_t)(l_new - l_old) * fade) >> 15);
            out[frame * sink->numChannels + 1] = r_old + (int32_t)(((int64_t)(r_new - r_old) * fade) >> 15);
        }
    } else {
        for (frame = 0; frame < n_frames; frame++) {
            int32_t mono = in[frame * src->numChannels];

            hrtf_delay[hrtf_widx] = mono;

            int64_t acc_L = 0, acc_R = 0;
            int d = hrtf_widx;
            for (int k = 0; k < HRTF_IR_LEN; k++) {
                int32_t x   = hrtf_delay[d];
                int32_t h_L = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
                int32_t h_R = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
                acc_L += (int64_t)x * h_L;
                acc_R += (int64_t)x * h_R;
                if (--d < 0) d = HRTF_IR_LEN - 1;
            }

            if (++hrtf_widx >= HRTF_IR_LEN) hrtf_widx = 0;

            out[frame * sink->numChannels + 0] = (int32_t)(acc_L >> 31);
            out[frame * sink->numChannels + 1] = (int32_t)(acc_R >> 31);
        }
    }

    /* Always refresh snapshot so the next crossfade has a clean starting state */
    memcpy(hrtf_delay_prev, hrtf_delay, sizeof(hrtf_delay));
    hrtf_prev_widx = hrtf_widx;

    if (ir_changed || first_run) {
        for (int k = 0; k < HRTF_IR_LEN; k++) {
            hrtf_prev_ir_L[k] = ir_L_lo[k] + (int32_t)(((int64_t)(ir_L_hi[k] - ir_L_lo[k]) * t) >> 15);
            hrtf_prev_ir_R[k] = ir_R_lo[k] + (int32_t)(((int64_t)(ir_R_hi[k] - ir_R_lo[k]) * t) >> 15);
        }
        hrtf_prev_idx_lo = idx_lo;
        hrtf_prev_idx_hi = idx_hi;
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
    unsigned i, channel, frame, channels;
    int32_t *in, *out;
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

#if 1
    if (src->numFrames != sink->numFrames) {
        return;
    }
    if (src->wordSize != sink->wordSize) {
        return;
    }
    if (src->wordSize != sizeof(int32_t)) {
        return;
    }
#endif
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

    if (cyclesMsg &&(clockDomain < IPC_CYCLE_DOMAIN_MAX)) {
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
            if (msg->parameter.id == 0) {
                 hrtf_target_az_deg = msg->parameter.value;
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
