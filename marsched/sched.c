#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdlib.h>

#include "serdes_sim.h"

#define NUM_LANES 16
#define DEFAULT_DATA_RATE 60

typedef struct {
    LaneContext lane;
    int priority;
} Task;

int rr_index = 0;
int pll_enabled = 1;

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

    /* print on state transitions */
    if (task->lane.state != prev) {
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
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <channel_taps.txt>\n", argv[0]);
        return 1;
    }

    setvbuf(stdout, NULL, _IOLBF, 0);

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
    printf("Commands:\n");
    printf("  s [lane]  - show status (all lanes, or one lane)\n");
    printf("  d <lane> <rate>\n");
    printf("  r <lane>\n");
    printf("  p\n");

    int stdin_open = 1;

    while (1) {
        clock++;

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
                }
                else if (buf[0] == 'd') {
                    int lane, rate;
                    sscanf(buf, "d %d %d", &lane, &rate);
                    if (lane >= 0 && lane < NUM_LANES) {
                        taskList[lane].lane.dataRateGbps = rate;
                        lane_soft_reset(&taskList[lane].lane);
                        taskList[lane].priority = 0;
                        printf("Lane %d rate changed to %d Gbps\n", lane, rate);
                    }
                }
                else if (buf[0] == 'r') {
                    int lane;
                    sscanf(buf, "r %d", &lane);
                    if (lane >= 0 && lane < NUM_LANES) {
                        lane_soft_reset(&taskList[lane].lane);
                        taskList[lane].priority = 0;
                        printf("Lane %d soft reset\n", lane);
                    }
                }
                else if (buf[0] == 'p') {
                    pll_enabled = !pll_enabled;
                    printf("PLL %s\n", pll_enabled ? "ON" : "OFF");
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