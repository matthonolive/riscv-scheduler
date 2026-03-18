/*
 * serdes_sim.c
 *
 * Scheduler-friendly SerDes link simulation.
 *
 * State machine driven by the scheduler in sched.c:
 *
 *   INIT  →  CTLE  →  RX  →  DONE
 *
 *   lane_init()        →  allocate buffers, enter INIT (no DSP work)
 *   lane_step_init()   →  generate PRBS, build channel, run CDR → CTLE
 *   lane_step_ctle()   →  advance CTLE sweep by STEP_SIZE samples
 *   lane_step_rx()     →  advance RX FFE+DFE training by STEP_SIZE samples
 *   lane_soft_reset()  →  restart from INIT (keeps channel if loaded)
 *   lane_destroy()     →  free heap memory
 *
 * TX FFE taps are pre-programmed (unit tap at pre-cursor position).
 */

#include "serdes_sim.h"

int lane_tick = 0;
FILE *lane_logfp = NULL;

/* ═══════════════════════════════════════════════════════════════════════
 *  Utility helpers
 * ═══════════════════════════════════════════════════════════════════════ */

void butter1_highpass(double Wn, double b[2], double a[2])
{
    double C = 1.0 / tan(M_PI * Wn / 2.0);
    double D = 1.0 + C;
    b[0] =  C / D;
    b[1] = -C / D;
    a[0] =  1.0;
    a[1] =  (1.0 - C) / D;
}

void butter2_lowpass(double Wn, double b[3], double a[3])
{
    double C  = 1.0 / tan(M_PI * Wn / 2.0);
    double C2 = C * C;
    double S2 = sqrt(2.0);
    double D  = C2 + S2 * C + 1.0;

    b[0] = 1.0 / D;
    b[1] = 2.0 / D;
    b[2] = 1.0 / D;
    a[0] = 1.0;
    a[1] = 2.0 * (1.0 - C2) / D;
    a[2] = (C2 - S2 * C + 1.0) / D;
}

void ctle_design(CTLEFilter *ctle, double Fs,
                 double zHz, double pHz, double A)
{
    double Wn_hp = zHz / (Fs / 2.0);
    double Wn_lp = pHz / (Fs / 2.0);

    if (Wn_hp <= 0.0) Wn_hp = 1e-12;
    if (Wn_hp >= 1.0) Wn_hp = 1.0 - 1e-12;
    if (Wn_lp <= 0.0) Wn_lp = 1e-12;
    if (Wn_lp >= 1.0) Wn_lp = 1.0 - 1e-12;

    butter1_highpass(Wn_hp, ctle->bhp, ctle->ahp);
    butter2_lowpass (Wn_lp, ctle->blp, ctle->alp);
    ctle->A = A;

    ctle->zi_hp[0] = 0.0;
    ctle->zi_lp[0] = 0.0;
    ctle->zi_lp[1] = 0.0;
}

double ctle_step(CTLEFilter *ctle, double x)
{
    double hp, v, y;

    hp = ctle->bhp[0] * x + ctle->zi_hp[0];
    ctle->zi_hp[0] = ctle->bhp[1] * x - ctle->ahp[1] * hp;

    v = x + ctle->A * hp;

    y = ctle->blp[0] * v + ctle->zi_lp[0];
    ctle->zi_lp[0] = ctle->blp[1] * v - ctle->alp[1] * y + ctle->zi_lp[1];
    ctle->zi_lp[1] = ctle->blp[2] * v - ctle->alp[2] * y;

    return y;
}

int load_channel_taps(const char *filename, double *h_fir, int max_taps)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "ERROR: cannot open '%s'\n", filename);
        return -1;
    }
    int L = 0;
    while (L < max_taps && fscanf(fp, "%lf", &h_fir[L]) == 1)
        L++;
    fclose(fp);
    return L;
}

double adc_quantize(double x, int B)
{
    double clamped = x;
    if (clamped >  1.0) clamped =  1.0;
    if (clamped < -1.0) clamped = -1.0;

    int levels = 1 << B;
    int xq = (int)round((clamped + 1.0) * (levels - 1) / 2.0);
    return xq * 2.0 / (levels - 1) - 1.0;
}

int int_mode(const int *arr, int n)
{
    if (n == 0) return 0;

    int mn = arr[0], mx = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] < mn) mn = arr[i];
        if (arr[i] > mx) mx = arr[i];
    }
    int range = mx - mn + 1;
    int *counts = (int *)calloc(range, sizeof(int));
    for (int i = 0; i < n; i++)
        counts[arr[i] - mn]++;

    int best_val = mn, best_cnt = 0;
    for (int i = 0; i < range; i++) {
        if (counts[i] > best_cnt) {
            best_cnt = counts[i];
            best_val = mn + i;
        }
    }
    free(counts);
    return best_val;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: TX FFE convolution (pre-programmed taps, not trained)
 * ═══════════════════════════════════════════════════════════════════════ */
static double apply_tx_ffe(const LaneContext *ctx, int pt)
{
    if (pt > TX_FFE_PRE * OSF) {
        double tx_out = 0.0;
        for (int k = 0; k < TX_FFE_LEN; k++) {
            int idx = pt + (k - TX_FFE_PRE) * OSF;
            if (idx >= 0 && idx < N_BIT * OSF)
                tx_out += ctx->TX_FFE[k] * ctx->bits_osf[idx];
        }
        return tx_out;
    }
    return ctx->bits_osf[pt];
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: push sample through channel FIR
 * ═══════════════════════════════════════════════════════════════════════ */
static double apply_channel(LaneContext *ctx, double sample_in)
{
    memmove(ctx->channel_buffer + 1, ctx->channel_buffer,
            (ctx->L - 1) * sizeof(double));
    ctx->channel_buffer[0] = sample_in;

    double y = 0.0;
    for (int k = 0; k < ctx->L; k++)
        y += ctx->h_fir[k] * ctx->channel_buffer[k];
    return y;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: generate PAM-4 PRBS
 * ═══════════════════════════════════════════════════════════════════════ */
static void generate_prbs(LaneContext *ctx)
{
    for (int i = 0; i < N_BIT; i++) {
        int sym = rand() % NUM_LEVELS;
        ctx->bits[i] = (2.0 * sym - (NUM_LEVELS - 1)) / (NUM_LEVELS - 1);
        for (int j = 0; j < OSF; j++)
            ctx->bits_osf[i * OSF + j] = ctx->bits[i];
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: run CDR
 * ═══════════════════════════════════════════════════════════════════════ */
static void run_cdr(LaneContext *ctx)
{
    int total_cdr = LEN_CDR * OSF;

    double *cb = (double *)calloc(ctx->L, sizeof(double));
    double *d_edge = (double *)malloc(total_cdr * sizeof(double));

    double post_channel = 0.0;

    for (int pt = 0; pt < total_cdr; pt++) {
        int sym_pair = pt / OSF;
        int clk_val  = (sym_pair % 2);

        memmove(cb + 1, cb, (ctx->L - 1) * sizeof(double));
        cb[0] = clk_val;

        double post_prev = post_channel;
        post_channel = 0.0;
        for (int k = 0; k < ctx->L; k++)
            post_channel += ctx->h_fir[k] * cb[k];

        d_edge[pt] = post_channel - post_prev;
    }

    /* find zero crossings */
    int *cross_raw = (int *)malloc(total_cdr * sizeof(int));
    int  n_cross = 0;
    for (int i = 0; i < total_cdr - 1; i++) {
        if (d_edge[i] * d_edge[i + 1] <= 0.0) {
            int zc = (fabs(d_edge[i]) <= fabs(d_edge[i + 1])) ? i : i + 1;
            cross_raw[n_cross++] = zc;
        }
    }

    int *cross_mod = (int *)malloc(n_cross * sizeof(int));
    for (int i = 0; i < n_cross; i++)
        cross_mod[i] = cross_raw[i] % OSF;
    ctx->sample_instant = int_mode(cross_mod, n_cross);

    double d_max = -DBL_MAX;
    for (int i = 0; i < total_cdr; i++)
        if (!isnan(d_edge[i]) && d_edge[i] > d_max)
            d_max = d_edge[i];

    int first_high = 0;
    for (int i = 0; i < total_cdr; i++) {
        if (!isnan(d_edge[i]) && d_edge[i] > d_max * 0.8) {
            first_high = i;
            break;
        }
    }
    ctx->lag = first_high -
               (int)round((double)ctx->sample_instant / 2.0);

    free(cross_raw);
    free(cross_mod);
    free(d_edge);
    free(cb);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: reset signal-path state for the start of a new phase
 * ═══════════════════════════════════════════════════════════════════════ */
static void reset_signal_path(LaneContext *ctx)
{
    memset(ctx->channel_buffer, 0, ctx->L * sizeof(double));
    memset(ctx->rx_buffer, 0, sizeof(ctx->rx_buffer));
    memset(ctx->d_hist,    0, sizeof(ctx->d_hist));
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: set up CTLE sweep parameters and enter CTLE state
 * ═══════════════════════════════════════════════════════════════════════ */
static void enter_ctle_phase(LaneContext *ctx)
{
    ctx->state = CTLE;
    ctx->pt    = 0;
    ctx->N_samp = (N_BIT - TX_FFE_POST) * OSF;

    ctx->ctle_p = 100e9;
    double ctle_A_max = 2.0;

    for (int i = 0; i < CTLE_NA; i++)
        ctx->A_vec[i] = 1.0 + i * (ctle_A_max - 1.0) / (CTLE_NA - 1);
    for (int i = 0; i < CTLE_NZ; i++)
        ctx->z_vec[i] = ctx->ctle_p / 2.0 +
                         i * (ctx->ctle_p / 2.0) / (CTLE_NZ - 1);

    ctx->ia = 0;
    ctx->iz = 0;
    ctx->ctle_cnt = 0;
    ctx->err_acc  = 0.0;
    ctx->ctle_train_done = 0;

    for (int a = 0; a < CTLE_NA; a++)
        for (int z = 0; z < CTLE_NZ; z++)
            ctx->J[a][z] = 1e30;

    ctx->ctle_A = ctx->A_vec[0];
    ctx->ctle_z = ctx->z_vec[0];
    ctle_design(&ctx->ctle, ctx->Fs, ctx->ctle_z, ctx->ctle_p, ctx->ctle_A);

    reset_signal_path(ctx);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Internal: set up RX FFE + DFE and enter RX state
 * ═══════════════════════════════════════════════════════════════════════ */
static void enter_rx_phase(LaneContext *ctx)
{
    ctx->state  = RX;
    ctx->pt     = 0;
    ctx->N_samp = (N_BIT - TX_FFE_POST) * OSF;

    memset(ctx->RX_FFE, 0, sizeof(ctx->RX_FFE));
    ctx->RX_FFE[RX_FFE_PRE] = 1.0;

    memset(ctx->DFE, 0, sizeof(ctx->DFE));
    ctx->en_DFE  = 1;
    ctx->mu_ffe  = 0.01;
    ctx->mu_dfe  = 0.005;

    ctle_design(&ctx->ctle, ctx->Fs,
                ctx->ctle_z, ctx->ctle_p, ctx->ctle_A);

    reset_signal_path(ctx);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════ */

/* ── lane_init ────────────────────────────────────────────────────────
 *  Lightweight: allocate buffers, set pre-programmed TX FFE taps,
 *  store channel file path, and enter INIT state.
 *  No DSP work is performed here.
 */
void lane_init(LaneContext *ctx, int dataRateGbps, const char *channel_file)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->state        = INIT;
    ctx->dataRateGbps = dataRateGbps;
    ctx->Fs           = (double)OSF * (double)dataRateGbps * 1e9;
    ctx->channel_file = channel_file;

    ctx->bits     = (double *)malloc(N_BIT * sizeof(double));
    ctx->bits_osf = (double *)malloc(N_BIT * OSF * sizeof(double));

    /* pre-programmed TX FFE: unit tap at pre-cursor position */
    memset(ctx->TX_FFE, 0, sizeof(ctx->TX_FFE));
    ctx->TX_FFE[TX_FFE_PRE] = 1.0;
}

/* ── lane_step_init ───────────────────────────────────────────────────
 *  Generate PRBS, build default channel (if none loaded from file),
 *  run CDR, then transition → CTLE.
 *
 *  Completes in a single call (CDR is inherently a batch operation).
 */
void lane_step_init(LaneContext *ctx)
{
    if (ctx->state != INIT) return;

    /* recompute Fs in case dataRateGbps changed */
    ctx->Fs = (double)OSF * (double)ctx->dataRateGbps * 1e9;

    /* load channel FIR taps from file */
    int L = load_channel_taps(ctx->channel_file, ctx->h_fir, MAX_CHANNEL_TAPS);
    if (L <= 0) {
        fprintf(stderr, "lane_step_init: failed to load '%s'\n",
                ctx->channel_file);
        return;   /* stay in INIT — scheduler will retry */
    }
    ctx->L = L;

    generate_prbs(ctx);
    run_cdr(ctx);

    enter_ctle_phase(ctx);
}

/* ── lane_soft_reset ──────────────────────────────────────────────────
 *  Restart link training from INIT.  Keeps loaded channel taps (if any)
 *  so CDR and training re-run against the same channel.
 */
void lane_soft_reset(LaneContext *ctx)
{
    /* reset TX FFE to pre-programmed values */
    memset(ctx->TX_FFE, 0, sizeof(ctx->TX_FFE));
    ctx->TX_FFE[TX_FFE_PRE] = 1.0;

    ctx->state = INIT;
}

/* ── lane_step_ctle ───────────────────────────────────────────────────
 *  Advance the CTLE sweep by up to STEP_SIZE oversampled points.
 *  Transitions → RX when the sweep grid has been fully evaluated
 *  (or all samples are exhausted).
 */
void lane_step_ctle(LaneContext *ctx)
{
    if (ctx->state != CTLE) return;

    int end = ctx->pt + STEP_SIZE;
    if (end > ctx->N_samp) end = ctx->N_samp;

    for (; ctx->pt < end; ctx->pt++) {
        int pt = ctx->pt;

        double tx_out = apply_tx_ffe(ctx, pt);
        double post_ch = apply_channel(ctx, tx_out);
        post_ch = ctle_step(&ctx->ctle, post_ch);

        if (pt % OSF == ctx->sample_instant &&
            (pt - ctx->lag - TX_FFE_PRE * OSF > 0))
        {
            int lag_idx = pt - ctx->lag;
            if (lag_idx > 0 && lag_idx < N_BIT * OSF) {
                double desired   = ctx->bits_osf[lag_idx];
                double bit_error = desired - post_ch;

                if (!ctx->ctle_train_done) {
                    ctx->ctle_cnt++;
                    ctx->err_acc += bit_error * bit_error;

                    if (ctx->ctle_cnt == CTLE_WINDOW) {
                        ctx->J[ctx->ia][ctx->iz] =
                            ctx->err_acc / CTLE_WINDOW;
                        ctx->ctle_cnt = 0;
                        ctx->err_acc  = 0.0;

                        ctx->ia++;
                        if (ctx->ia >= CTLE_NA) {
                            ctx->ia = 0;
                            ctx->iz++;
                        }
                        if (ctx->iz >= CTLE_NZ) {
                            /* sweep complete — pick best */
                            double best_J = 1e30;
                            int ia_best = 0, iz_best = 0;
                            for (int a = 0; a < CTLE_NA; a++)
                                for (int z = 0; z < CTLE_NZ; z++)
                                    if (ctx->J[a][z] < best_J) {
                                        best_J  = ctx->J[a][z];
                                        ia_best = a;
                                        iz_best = z;
                                    }
                            ctx->ctle_A = ctx->A_vec[ia_best];
                            ctx->ctle_z = ctx->z_vec[iz_best];
                            ctle_design(&ctx->ctle, ctx->Fs,
                                        ctx->ctle_z, ctx->ctle_p,
                                        ctx->ctle_A);
                            ctx->ctle_train_done = 1;
                        } else {
                            ctx->ctle_A = ctx->A_vec[ctx->ia];
                            ctx->ctle_z = ctx->z_vec[ctx->iz];
                            ctle_design(&ctx->ctle, ctx->Fs,
                                        ctx->ctle_z, ctx->ctle_p,
                                        ctx->ctle_A);
                        }
                    }
                }
            }
        }
    }

    /* Transition when sweep finished or samples exhausted */
    if (ctx->ctle_train_done || ctx->pt >= ctx->N_samp) {
        if (!ctx->ctle_train_done) {
            double best_J = 1e30;
            int ia_best = 0, iz_best = 0;
            for (int a = 0; a < CTLE_NA; a++)
                for (int z = 0; z < CTLE_NZ; z++)
                    if (ctx->J[a][z] < best_J) {
                        best_J  = ctx->J[a][z];
                        ia_best = a;
                        iz_best = z;
                    }
            ctx->ctle_A = ctx->A_vec[ia_best];
            ctx->ctle_z = ctx->z_vec[iz_best];
        }
        enter_rx_phase(ctx);
    }
}

/* ── lane_step_rx ─────────────────────────────────────────────────────
 *  Advance RX FFE + DFE training by up to STEP_SIZE oversampled points.
 *  Transitions → DONE when all samples are consumed.
 */
void lane_step_rx(LaneContext *ctx)
{
    if (ctx->state != RX) return;

    int end = ctx->pt + STEP_SIZE;
    if (end > ctx->N_samp) end = ctx->N_samp;

    for (; ctx->pt < end; ctx->pt++) {
        int pt = ctx->pt;

        double tx_out = apply_tx_ffe(ctx, pt);
        double post_ch = apply_channel(ctx, tx_out);
        post_ch = ctle_step(&ctx->ctle, post_ch);
        post_ch = adc_quantize(post_ch, ADC_BITS);

        memmove(ctx->rx_buffer + 1, ctx->rx_buffer,
                (RX_FFE_LEN - 1) * sizeof(double));
        ctx->rx_buffer[0] = post_ch;

        if (pt % OSF == ctx->sample_instant && (pt - ctx->lag > 0)) {

            double y_ffe = 0.0;
            for (int k = 0; k < RX_FFE_LEN; k++)
                y_ffe += ctx->RX_FFE[k] * ctx->rx_buffer[k];

            double y = y_ffe;
            if (ctx->en_DFE) {
                for (int k = 0; k < N_DFE; k++)
                    y -= ctx->DFE[k] * ctx->d_hist[k];
            }

            double d_hat = (y < 0.0) ? -1.0 : 1.0;

            int lag_idx = pt - ctx->lag;
            if (lag_idx >= 0 && lag_idx < N_BIT * OSF) {
                double desired   = ctx->bits_osf[lag_idx];
                double bit_error = desired - y;

                for (int k = 0; k < RX_FFE_LEN; k++)
                    ctx->RX_FFE[k] += ctx->mu_ffe * bit_error *
                                      ctx->rx_buffer[k];

                if (ctx->en_DFE) {
                    for (int k = 0; k < N_DFE; k++)
                        ctx->DFE[k] -= ctx->mu_dfe * bit_error *
                                        ctx->d_hist[k];
                }
            }

            if (N_DFE > 0)
                memmove(ctx->d_hist + 1, ctx->d_hist,
                        (N_DFE - 1) * sizeof(double));
            ctx->d_hist[0] = d_hat;
        }
    }

    if (ctx->pt >= ctx->N_samp)
        ctx->state = DONE;
}

/* ── lane_destroy ─────────────────────────────────────────────────────
 *  Free heap memory owned by the lane context.
 */
void lane_destroy(LaneContext *ctx)
{
    free(ctx->bits);
    free(ctx->bits_osf);
    ctx->bits     = NULL;
    ctx->bits_osf = NULL;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Debug: print lane status to console and log file
 * ═══════════════════════════════════════════════════════════════════════ */

void print_lane_status(int id, const LaneContext *l)
{
    printf("  Lane %2d | %s | %d Gbps", id, state_name(l->state), l->dataRateGbps);

    if (l->state == CTLE || l->state == RX || l->state == DONE)
        printf(" | CDR instant=%d lag=%d", l->sample_instant, l->lag);

    if (l->state == CTLE)
        printf(" | sweep [%d,%d]/%d", l->ia + l->iz * CTLE_NA,
               CTLE_NA * CTLE_NZ, CTLE_NA * CTLE_NZ);

    if (l->state == RX || l->state == DONE) {
        printf(" | CTLE A=%.4f z=%.3e", l->ctle_A, l->ctle_z);
        printf("\n         RX_FFE[");
        for (int k = 0; k < RX_FFE_LEN; k++)
            printf("%s%+.4f", k ? " " : "", l->RX_FFE[k]);
        printf("]");
        printf("\n         DFE[");
        for (int k = 0; k < N_DFE; k++)
            printf("%s%+.4f", k ? " " : "", l->DFE[k]);
        printf("]");
    }
    printf("\n");
}

const char *state_name(LaneState s)
{
    switch (s) {
        case INIT: return "INIT";
        case CTLE: return "CTLE";
        case RX:   return "RX";
        case DONE: return "DONE";
    }
    return "?";
}

void generic_lane_init(void *ctx, void *args)
{
    LaneContext *lane_ctx = (LaneContext *)ctx;
    LaneInitArgs *init_args = (LaneInitArgs *)args;

    lane_init(lane_ctx, init_args->dataRateGbps, init_args->channel_file);
}

void updateLaneTick()
{
    lane_tick++;
}

void taskStepForward(void *ctx, int lane_id)
{

    LaneContext *lane_ctx_p = (LaneContext *)ctx;

    LaneState prev = lane_ctx_p->state;
    int prev_pt = lane_ctx_p->pt;
    int prev_ia = lane_ctx_p->ia;
    int prev_iz = lane_ctx_p->iz;

    if (lane_ctx_p->state == INIT) {
        lane_step_init(lane_ctx_p);
    }
    else if (lane_ctx_p->state == CTLE) {
        lane_step_ctle(lane_ctx_p);
    }
    else if (lane_ctx_p->state == RX) {
        lane_step_rx(lane_ctx_p);
    }

    /* ── Verbose file log: every step ── */
    if (lane_logfp) {
        fprintf(lane_logfp, "[tick %8d] Lane %2d  state=%-4s  pt=%d/%d\n",
                lane_tick, lane_id, state_name(lane_ctx_p->state),
                lane_ctx_p->pt, lane_ctx_p->N_samp);

        /* CTLE sweep: log when a grid point completes (ia or iz advanced) */
        if (lane_ctx_p->state == CTLE && prev == CTLE) {
            int old_grid = prev_ia + prev_iz * CTLE_NA;
            int new_grid = lane_ctx_p->ia + lane_ctx_p->iz * CTLE_NA;
            if (new_grid != old_grid) {
                fprintf(lane_logfp, "             Lane %2d  CTLE sweep: completed grid [%d/%d]"
                        "  A=%.4f z=%.3e  MSE=%.6f\n",
                        lane_id, old_grid + 1, CTLE_NA * CTLE_NZ,
                        lane_ctx_p->A_vec[prev_ia], lane_ctx_p->z_vec[prev_iz],
                        lane_ctx_p->J[prev_ia][prev_iz]);
                fprintf(lane_logfp, "             Lane %2d  CTLE sweep: now testing [%d/%d]"
                        "  A=%.4f z=%.3e\n",
                        lane_id, new_grid + 1, CTLE_NA * CTLE_NZ,
                        lane_ctx_p->ctle_A, lane_ctx_p->ctle_z);
            }
        }

        /* RX progress: log taps every 25% */
        if (lane_ctx_p->state == RX && prev == RX) {
            int quarter = lane_ctx_p->N_samp / 4;
            if (quarter > 0 && prev_pt / quarter != lane_ctx_p->pt / quarter) {
                int pct = (lane_ctx_p->pt * 100) / lane_ctx_p->N_samp;
                fprintf(lane_logfp, "             Lane %2d  RX training %d%%  RX_FFE[main]=%.6f"
                        "  DFE[0]=%.6f\n",
                        lane_id, pct,
                        lane_ctx_p->RX_FFE[RX_FFE_PRE], lane_ctx_p->DFE[0]);
                fprintf(lane_logfp, "               RX_FFE = [");
                for (int k = 0; k < RX_FFE_LEN; k++)
                    fprintf(lane_logfp, "%s%+.6f", k ? ", " : "", lane_ctx_p->RX_FFE[k]);
                fprintf(lane_logfp, "]\n");
            }
        }
    }

    /* ── State transition: console + file ── */
    if (lane_ctx_p->state != prev) {

        /* --- Console (concise) --- */
        printf("[Lane %2d] %s → %s", lane_id,
               state_name(prev), state_name(lane_ctx_p->state));

        if (prev == INIT)
            printf("  (loaded %d taps, CDR instant=%d lag=%d)",
                   lane_ctx_p->L, lane_ctx_p->sample_instant, lane_ctx_p->lag);

        if (prev == CTLE)
            printf("  (CTLE A=%.4f z=%.3e)",
                   lane_ctx_p->ctle_A, lane_ctx_p->ctle_z);

        if (prev == RX) {
            printf("\n  RX_FFE = [");
            for (int k = 0; k < RX_FFE_LEN; k++)
                printf("%s%+.6f", k ? ", " : "", lane_ctx_p->RX_FFE[k]);
            printf("]\n  DFE    = [");
            for (int k = 0; k < N_DFE; k++)
                printf("%s%+.6f", k ? ", " : "", lane_ctx_p->DFE[k]);
            printf("]");
        }
        printf("\n");
        fflush(stdout);

        /* --- Log file (verbose) --- */
        if (lane_logfp) {
            fprintf(lane_logfp, "========== Lane %2d TRANSITION: %s → %s (tick %d) ==========\n",
                    lane_id, state_name(prev), state_name(lane_ctx_p->state), lane_tick);

            if (prev == INIT) {
                fprintf(lane_logfp, "  Channel:  %s (%d taps)\n",
                        lane_ctx_p->channel_file, lane_ctx_p->L);
                fprintf(lane_logfp, "  Data rate: %d Gbps  Fs=%.3e Hz\n",
                        lane_ctx_p->dataRateGbps, lane_ctx_p->Fs);
                fprintf(lane_logfp, "  CDR:       sample_instant=%d  lag=%d\n",
                        lane_ctx_p->sample_instant, lane_ctx_p->lag);
                fprintf(lane_logfp, "  TX FFE:    [");
                for (int k = 0; k < TX_FFE_LEN; k++)
                    fprintf(lane_logfp, "%s%+.6f", k ? ", " : "", lane_ctx_p->TX_FFE[k]);
                fprintf(lane_logfp, "]\n");
            }

            if (prev == CTLE) {
                fprintf(lane_logfp, "  Best CTLE: A=%.6f  z=%.6e  p=%.6e\n",
                        lane_ctx_p->ctle_A, lane_ctx_p->ctle_z, lane_ctx_p->ctle_p);
                fprintf(lane_logfp, "  Sweep MSE grid (A rows x z cols):\n");
                for (int a = 0; a < CTLE_NA; a++) {
                    fprintf(lane_logfp, "    A=%.4f |", lane_ctx_p->A_vec[a]);
                    for (int z = 0; z < CTLE_NZ; z++) {
                        if (lane_ctx_p->J[a][z] < 1e20)
                            fprintf(lane_logfp, " %10.6f", lane_ctx_p->J[a][z]);
                        else
                            fprintf(lane_logfp, "        N/A");
                    }
                    fprintf(lane_logfp, "\n");
                }
            }

            if (prev == RX) {
                fprintf(lane_logfp, "  RX FFE taps (%d total):\n", RX_FFE_LEN);
                for (int k = 0; k < RX_FFE_LEN; k++)
                    fprintf(lane_logfp, "    RX_FFE[%2d] = %+.8f%s\n", k, lane_ctx_p->RX_FFE[k],
                            k == RX_FFE_PRE ? "  <-- main cursor" : "");
                fprintf(lane_logfp, "  DFE taps (%d total):\n", N_DFE);
                for (int k = 0; k < N_DFE; k++)
                    fprintf(lane_logfp, "    DFE[%d]    = %+.8f\n", k, lane_ctx_p->DFE[k]);
            }

            fprintf(lane_logfp, "==========================================================\n");
            fflush(lane_logfp);
        }
    }
}

