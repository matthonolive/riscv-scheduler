#ifndef SERDES_SIM_H
#define SERDES_SIM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  Compile-time parameters
 * ═══════════════════════════════════════════════════════════════════════ */
#define OSF             16          /* oversampling factor                */
#define N_BIT           2048        /* PRBS length (symbols)              */
#define ADC_BITS        5           /* ADC quantisation bits              */
#define NUM_LEVELS      4           /* PAM-4                              */

/* TX FFE (pre-programmed, not trained) */
#define TX_FFE_PRE      4
#define TX_FFE_POST     0
#define TX_FFE_LEN      (TX_FFE_PRE + 1 + TX_FFE_POST)

/* RX FFE */
#define RX_FFE_PRE      3
#define RX_FFE_POST     10
#define RX_FFE_LEN      (RX_FFE_PRE + 1 + RX_FFE_POST)

/* DFE */
#define N_DFE           1

/* CDR */
#define LEN_CDR         1000

/* CTLE sweep */
#define CTLE_NA         7           /* # gain steps                       */
#define CTLE_NZ         5           /* # zero-frequency steps             */
#define CTLE_WINDOW     500         /* symbols per sweep point            */

/* Channel */
#define MAX_CHANNEL_TAPS 4096

/* Number of oversampled points processed per scheduler step call.      */
#define STEP_SIZE       OSF


/* ══════════════════════════════════════════════════════════════════════
 *  Lane state machine
 * ═══════════════════════════════════════════════════════════════════════ */
typedef enum {
    INIT,           /* generate PRBS, build channel, run CDR              */
    CTLE,           /* CTLE sweep / training                              */
    RX,             /* RX FFE + DFE adaptation                            */
    DONE            /* link training complete                              */
} LaneState;

/* ═══════════════════════════════════════════════════════════════════════
 *  CTLE filter structure
 * ═══════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* 1st-order Butterworth highpass */
    double bhp[2];
    double ahp[2];
    double zi_hp[1];

    /* 2nd-order Butterworth lowpass */
    double blp[3];
    double alp[3];
    double zi_lp[2];

    double A;               /* CTLE boost gain */
} CTLEFilter;

/* ═══════════════════════════════════════════════════════════════════════
 *  Per-lane context  — holds ALL mutable state for one SerDes lane
 * ═══════════════════════════════════════════════════════════════════════ */
typedef struct {
    LaneState state;
    int       dataRateGbps;         /* symbol rate in Gbps               */
    double    Fs;                   /* sample rate = OSF * dataRate       */

    /* ── Channel ────────────────────────────────────────────────────── */
    double h_fir[MAX_CHANNEL_TAPS]; /* FIR taps loaded from file          */
    int    L;                       /* number of channel taps             */
    double channel_buffer[MAX_CHANNEL_TAPS]; /* FIR delay line            */
    const char *channel_file;       /* path to channel taps file          */

    /* ── Bitstream (heap-allocated in lane_init) ────────────────────── */
    double *bits;                   /* [N_BIT]                            */
    double *bits_osf;               /* [N_BIT * OSF]                      */

    /* ── CDR results ────────────────────────────────────────────────── */
    int sample_instant;
    int lag;

    /* ── TX FFE (pre-programmed, not trained) ───────────────────────── */
    double TX_FFE[TX_FFE_LEN];

    /* ── CTLE ───────────────────────────────────────────────────────── */
    CTLEFilter ctle;
    double ctle_z;
    double ctle_p;
    double ctle_A;
    double A_vec[CTLE_NA];
    double z_vec[CTLE_NZ];
    double J[CTLE_NA][CTLE_NZ];
    int    ia, iz;
    int    ctle_cnt;
    double err_acc;
    int    ctle_train_done;

    /* ── RX FFE + DFE ──────────────────────────────────────────────── */
    double RX_FFE[RX_FFE_LEN];
    double DFE[N_DFE];
    double d_hist[N_DFE];
    double rx_buffer[RX_FFE_LEN];
    int    en_DFE;
    double mu_ffe;
    double mu_dfe;

    /* ── Iteration bookkeeping ──────────────────────────────────────── */
    int pt;                         /* current sample index in phase      */
    int N_samp;                     /* total samples for current phase    */
} LaneContext;

/* ═══════════════════════════════════════════════════════════════════════
 *  Utility functions
 * ═══════════════════════════════════════════════════════════════════════ */
void   butter1_highpass(double Wn, double b[2], double a[2]);
void   butter2_lowpass (double Wn, double b[3], double a[3]);

void   ctle_design(CTLEFilter *ctle, double Fs,
                   double zHz, double pHz, double A);
double ctle_step  (CTLEFilter *ctle, double x);

int    load_channel_taps(const char *filename, double *h_fir, int max_taps);
double adc_quantize(double x, int B);
int    int_mode(const int *arr, int n);

/* ═══════════════════════════════════════════════════════════════════════
 *  Lane API  — called from the scheduler
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  lane_init()          Allocate buffers and enter INIT state.
 *                       Lightweight — no DSP work is performed.
 *
 *  lane_step_init()     Load channel taps from file, generate PRBS,
 *                       run CDR.  Transitions → CTLE.
 *
 *  lane_step_ctle()     Advance CTLE sweep by STEP_SIZE samples.
 *                       Transitions → RX when sweep is complete.
 *
 *  lane_step_rx()       Advance RX FFE + DFE training by STEP_SIZE
 *                       samples.  Transitions → DONE when complete.
 *
 *  lane_soft_reset()    Re-enter INIT (reloads channel on next step).
 *
 *  lane_destroy()       Free heap memory owned by the context.
 */
void lane_init         (LaneContext *ctx, int dataRateGbps,
                        const char *channel_file);
void lane_step_init    (LaneContext *ctx);
void lane_step_ctle    (LaneContext *ctx);
void lane_step_rx      (LaneContext *ctx);
void lane_soft_reset   (LaneContext *ctx);
void lane_destroy      (LaneContext *ctx);
void print_lane_status(int lane_id, const LaneContext *ctx);
const char *state_name(LaneState s);

/* ═══════════════════════════════════════════════════════════════════════
 *  Generic: provides generic interface to the scheudler using void pointers
 * ══════════════════════════════════════════════════════════════════════= 
 */

 void generic_lane_init(void *ctx, void* args);
 void generic_lane_step(void *ctx, void* args);

 /* ══════════════════════════════════════════════════════════════════════
 *  Generic arg structs
 * ═══════════════════════════════════════════════════════════════════════ */
typedef struct {
    int dataRateGbps;
    const char *channel_file;
} LaneInitArgs;

typedef struct {
    int dataRateGbps; // Only used if flags=2 (data rate change interrupt)
    char flags; // If flags = 0, then run as usual. Otherwise, flags represent the interrupt type (1 = soft reset, 2 = data rate change, etc.)
} LaneStepArgs;

// Debugging
void updateLaneTick();
void setLogFile(FILE *fp);


#endif /* SERDES_SIM_H */