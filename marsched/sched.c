#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdlib.h>

#include "serdes_sim.h"

#define NUM_LANES 16
#define DEFAULT_DATA_RATE 60
#define LOG_FILE "sched.log"

typedef struct {
    LaneContext lane;
    int priority;
} Task;

int rr_index = 0;
int pll_enabled = 1;
FILE *logfp = NULL;
int tick = 0;

static const char *state_name(LaneState s)
{
    switch (s) {
        case INIT: return "INIT";
        case CTLE: return "CTLE";
        case RX:   return "RX";
        case DONE: return "DONE";
    }
    return "?";
}

static void print_lane_status(int id, const LaneContext *l)
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

void taskStepForward(Task *task, int lane_id)
{
    LaneState prev = task->lane.state;
    int prev_pt = task->lane.pt;
    int prev_ia = task->lane.ia;
    int prev_iz = task->lane.iz;
    task->priority++;

    if (task->lane.state == INIT) {
        lane_step_init(&task->lane);
    }
    else if (task->lane.state == CTLE) {
        lane_step_ctle(&task->lane);
    }
    else if (task->lane.state == RX) {
        lane_step_rx(&task->lane);
    }

    /* ── Verbose file log: every step ── */
    if (logfp) {
        fprintf(logfp, "[tick %8d] Lane %2d  state=%-4s  pt=%d/%d  prio=%d\n",
                tick, lane_id, state_name(task->lane.state),
                task->lane.pt, task->lane.N_samp, task->priority);

        /* CTLE sweep: log when a grid point completes (ia or iz advanced) */
        if (task->lane.state == CTLE && prev == CTLE) {
            int old_grid = prev_ia + prev_iz * CTLE_NA;
            int new_grid = task->lane.ia + task->lane.iz * CTLE_NA;
            if (new_grid != old_grid) {
                /* the grid point that just finished is (prev_ia, prev_iz) */
                fprintf(logfp, "             Lane %2d  CTLE sweep: completed grid [%d/%d]"
                        "  A=%.4f z=%.3e  MSE=%.6f\n",
                        lane_id, old_grid + 1, CTLE_NA * CTLE_NZ,
                        task->lane.A_vec[prev_ia], task->lane.z_vec[prev_iz],
                        task->lane.J[prev_ia][prev_iz]);
                fprintf(logfp, "             Lane %2d  CTLE sweep: now testing [%d/%d]"
                        "  A=%.4f z=%.3e\n",
                        lane_id, new_grid + 1, CTLE_NA * CTLE_NZ,
                        task->lane.ctle_A, task->lane.ctle_z);
            }
        }

        /* RX progress: log taps every 25% */
        if (task->lane.state == RX && prev == RX) {
            int quarter = task->lane.N_samp / 4;
            if (quarter > 0 && prev_pt / quarter != task->lane.pt / quarter) {
                int pct = (task->lane.pt * 100) / task->lane.N_samp;
                fprintf(logfp, "             Lane %2d  RX training %d%%  RX_FFE[main]=%.6f"
                        "  DFE[0]=%.6f\n",
                        lane_id, pct,
                        task->lane.RX_FFE[RX_FFE_PRE], task->lane.DFE[0]);
                fprintf(logfp, "               RX_FFE = [");
                for (int k = 0; k < RX_FFE_LEN; k++)
                    fprintf(logfp, "%s%+.6f", k ? ", " : "", task->lane.RX_FFE[k]);
                fprintf(logfp, "]\n");
            }
        }
    }

    /* ── State transition: console + file ── */
    if (task->lane.state != prev) {

        /* --- Console (concise) --- */
        printf("[Lane %2d] %s → %s", lane_id,
               state_name(prev), state_name(task->lane.state));

        if (prev == INIT)
            printf("  (loaded %d taps, CDR instant=%d lag=%d)",
                   task->lane.L, task->lane.sample_instant, task->lane.lag);

        if (prev == CTLE)
            printf("  (CTLE A=%.4f z=%.3e)",
                   task->lane.ctle_A, task->lane.ctle_z);

        if (prev == RX) {
            printf("\n  RX_FFE = [");
            for (int k = 0; k < RX_FFE_LEN; k++)
                printf("%s%+.6f", k ? ", " : "", task->lane.RX_FFE[k]);
            printf("]\n  DFE    = [");
            for (int k = 0; k < N_DFE; k++)
                printf("%s%+.6f", k ? ", " : "", task->lane.DFE[k]);
            printf("]");
        }
        printf("\n");
        fflush(stdout);

        /* --- Log file (verbose) --- */
        if (logfp) {
            fprintf(logfp, "========== Lane %2d TRANSITION: %s → %s (tick %d) ==========\n",
                    lane_id, state_name(prev), state_name(task->lane.state), tick);

            if (prev == INIT) {
                fprintf(logfp, "  Channel:  %s (%d taps)\n",
                        task->lane.channel_file, task->lane.L);
                fprintf(logfp, "  Data rate: %d Gbps  Fs=%.3e Hz\n",
                        task->lane.dataRateGbps, task->lane.Fs);
                fprintf(logfp, "  CDR:       sample_instant=%d  lag=%d\n",
                        task->lane.sample_instant, task->lane.lag);
                fprintf(logfp, "  TX FFE:    [");
                for (int k = 0; k < TX_FFE_LEN; k++)
                    fprintf(logfp, "%s%+.6f", k ? ", " : "", task->lane.TX_FFE[k]);
                fprintf(logfp, "]\n");
            }

            if (prev == CTLE) {
                fprintf(logfp, "  Best CTLE: A=%.6f  z=%.6e  p=%.6e\n",
                        task->lane.ctle_A, task->lane.ctle_z, task->lane.ctle_p);
                fprintf(logfp, "  Sweep MSE grid (A rows x z cols):\n");
                for (int a = 0; a < CTLE_NA; a++) {
                    fprintf(logfp, "    A=%.4f |", task->lane.A_vec[a]);
                    for (int z = 0; z < CTLE_NZ; z++) {
                        if (task->lane.J[a][z] < 1e20)
                            fprintf(logfp, " %10.6f", task->lane.J[a][z]);
                        else
                            fprintf(logfp, "        N/A");
                    }
                    fprintf(logfp, "\n");
                }
            }

            if (prev == RX) {
                fprintf(logfp, "  RX FFE taps (%d total):\n", RX_FFE_LEN);
                for (int k = 0; k < RX_FFE_LEN; k++)
                    fprintf(logfp, "    RX_FFE[%2d] = %+.8f%s\n", k, task->lane.RX_FFE[k],
                            k == RX_FFE_PRE ? "  <-- main cursor" : "");
                fprintf(logfp, "  DFE taps (%d total):\n", N_DFE);
                for (int k = 0; k < N_DFE; k++)
                    fprintf(logfp, "    DFE[%d]    = %+.8f\n", k, task->lane.DFE[k]);
            }

            fprintf(logfp, "==========================================================\n");
            fflush(logfp);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <channel_taps.txt>\n", argv[0]);
        return 1;
    }

    setvbuf(stdout, NULL, _IOLBF, 0);

    logfp = fopen(LOG_FILE, "w");
    if (!logfp)
        fprintf(stderr, "Warning: could not open %s for writing\n", LOG_FILE);

    const char *channel_file = argv[1];
    int clock = 0;
    fd_set readfds;

    Task taskList[NUM_LANES];

    // Initialize all lanes with the same channel file
    for (int i = 0; i < NUM_LANES; i++) {
        lane_init(&taskList[i].lane, DEFAULT_DATA_RATE, channel_file);
        taskList[i].priority = 1;
    }

    printf("Scheduler started with channel '%s'.\n", channel_file);
    printf("Logs → %s\n", LOG_FILE);
    printf("Commands:\n");
    printf("  s [lane]  - show status (all lanes, or one lane)\n");
    printf("  d <lane> <rate>\n");
    printf("  r <lane>\n");
    printf("  p\n");

    if (logfp) {
        fprintf(logfp, "=== Scheduler started ===\n");
        fprintf(logfp, "Channel file: %s\n", channel_file);
        fprintf(logfp, "Lanes: %d   Data rate: %d Gbps\n", NUM_LANES, DEFAULT_DATA_RATE);
        fprintf(logfp, "OSF=%d  N_BIT=%d  ADC_BITS=%d  NUM_LEVELS=%d\n",
                OSF, N_BIT, ADC_BITS, NUM_LEVELS);
        fprintf(logfp, "TX_FFE: pre=%d post=%d len=%d\n", TX_FFE_PRE, TX_FFE_POST, TX_FFE_LEN);
        fprintf(logfp, "RX_FFE: pre=%d post=%d len=%d\n", RX_FFE_PRE, RX_FFE_POST, RX_FFE_LEN);
        fprintf(logfp, "DFE taps: %d\n", N_DFE);
        fprintf(logfp, "CTLE sweep: %d A steps x %d z steps, window=%d symbols\n",
                CTLE_NA, CTLE_NZ, CTLE_WINDOW);
        fprintf(logfp, "Step size: %d samples/step\n\n", STEP_SIZE);
        fflush(logfp);
    }

    int stdin_open = 1;

    while (1) {
        clock++;
        tick++;

        /* -------- INTERRUPT HANDLING -------- */
        if (stdin_open) {
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            struct timeval tv = {0, 0};

            int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

            if (ret > 0) {
                char buf[64];
                if (!fgets(buf, sizeof(buf), stdin)) {
                    stdin_open = 0;   /* EOF — stop polling */
                } else if (buf[0] == 's') {
                    int lane = -1;
                    sscanf(buf, "s %d", &lane);
                    if (lane >= 0 && lane < NUM_LANES) {
                        print_lane_status(lane, &taskList[lane].lane);
                    } else {
                        printf("─── Lane Status ───\n");
                        for (int i = 0; i < NUM_LANES; i++)
                            print_lane_status(i, &taskList[i].lane);
                    }
                    if (logfp) fprintf(logfp, "[tick %8d] CMD: status query\n", tick);
                }
                else if (buf[0] == 'd') {
                    int lane, rate;
                    sscanf(buf, "d %d %d", &lane, &rate);
                    if (lane >= 0 && lane < NUM_LANES) {
                        taskList[lane].lane.dataRateGbps = rate;
                        lane_soft_reset(&taskList[lane].lane);
                        taskList[lane].priority = 0;
                        printf("Lane %d rate changed to %d Gbps\n", lane, rate);
                        if (logfp) fprintf(logfp, "[tick %8d] CMD: lane %d rate → %d Gbps (soft reset)\n",
                                           tick, lane, rate);
                    }
                }
                else if (buf[0] == 'r') {
                    int lane;
                    sscanf(buf, "r %d", &lane);
                    if (lane >= 0 && lane < NUM_LANES) {
                        lane_soft_reset(&taskList[lane].lane);
                        taskList[lane].priority = 0;
                        printf("Lane %d soft reset\n", lane);
                        if (logfp) fprintf(logfp, "[tick %8d] CMD: lane %d soft reset\n", tick, lane);
                    }
                }
                else if (buf[0] == 'p') {
                    pll_enabled = !pll_enabled;
                    printf("PLL %s\n", pll_enabled ? "ON" : "OFF");
                    if (logfp) fprintf(logfp, "[tick %8d] CMD: PLL %s\n", tick,
                                       pll_enabled ? "ON" : "OFF");
                }
            }
        }

        /* -------- SCHEDULING -------- */

        int best = 999999;
        int chosen = -1;

        /* Find best priority */
        for (int i = 0; i < NUM_LANES; i++) {
            if (taskList[i].lane.state != DONE &&
                taskList[i].priority < best)
            {
                best = taskList[i].priority;
            }
        }

        /* Round-robin among best */
        for (int k = 0; k < NUM_LANES; k++) {
            int i = (rr_index + k) % NUM_LANES;

            if (taskList[i].lane.state != DONE &&
                taskList[i].priority == best)
            {
                chosen = i;
                break;
            }
        }

        if (chosen >= 0 && pll_enabled) {
            rr_index = (chosen + 1) % NUM_LANES;
            taskStepForward(&taskList[chosen], chosen);
        }

        usleep(10);    /* simulate firmware time slice */
    }

    return 0;
}