// TODO: change sched.c to use the generic tasks and strip references to the lanestate directly

#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <stdlib.h>
#include <string.h>

#include "serdes_sim.h"

#define NUM_LANES 16
#define DEFAULT_DATA_RATE 60
#define LOG_FILE "sched.log"

typedef struct {
    void *task_data; // Pointer to this task struct
    void (*task_run)(void *task_data, void *task_args); // Function pointer to the task's run function
    int priority;
    char is_active; // 1 if the task should be sccheduled, 0 if it is done or should not be scheduled
} Task;

int rr_index = 0;
int pll_enabled = 1;
FILE *logfp = NULL;
int tick = 0;

typedef struct {
    void *task_buffer; // Initialized in main as an arrayList of Task structs
    int task_buffer_size;
    int task_buffer_capacity; 
} Task_List;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <channel_taps.txt> [-r]\n", argv[0]);
        fprintf(stderr, "  -r   assign random initial priorities to each lane\n");
        return 1;
    }

    /* parse args */
    const char *channel_file = NULL;
    int random_prio = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-r") == 0)
            random_prio = 1;
        else
            channel_file = argv[i];
    }

    if (!channel_file) {
        fprintf(stderr, "Error: no channel file specified.\n");
        return 1;
    }


    srand((unsigned)time(NULL));
    setvbuf(stdout, NULL, _IOLBF, 0);

    logfp = fopen(LOG_FILE, "w");
    if (!logfp)
        fprintf(stderr, "Warning: could not open %s for writing\n", LOG_FILE);

    setLogFile(logfp);
    fd_set readfds;

    Task_List taskList;
    taskList.task_buffer_capacity = NUM_LANES;
    taskList.task_buffer = malloc(taskList.task_buffer_capacity * sizeof(Task));
    taskList.task_buffer_size = 0;

    /* Initialize all lanes */
    for (int i = 0; i < NUM_LANES; i++) {
        lane_init(&taskList.task_buffer[i].lane, DEFAULT_DATA_RATE, channel_file);
        taskList.task_buffer[i].priority = random_prio ? (rand() % NUM_LANES) : 1;
    }

    printf("Scheduler started with channel '%s'.\n", channel_file);
    printf("Priority mode: %s\n", random_prio ? "RANDOM" : "EQUAL");
    printf("Logs → %s\n", LOG_FILE);

    /* print initial priorities */
    printf("Initial priorities:");
    for (int i = 0; i < NUM_LANES; i++)
        printf(" [%d]=%d", i, taskList[i].priority);
    printf("\n");

    printf("Commands:\n");
    printf("  s [lane]          - show status (all lanes, or one lane)\n");
    printf("  d <lane> <rate>   - change data rate for a lane\n");
    printf("  r <lane>          - soft reset a lane\n");
    printf("  p                 - turn PLL on/off\n");

    if (logfp) {
        fprintf(logfp, "=== Scheduler started ===\n");
        fprintf(logfp, "Channel file: %s\n", channel_file);
        fprintf(logfp, "Priority mode: %s\n", random_prio ? "RANDOM" : "EQUAL");
        fprintf(logfp, "Lanes: %d   Data rate: %d Gbps\n", NUM_LANES, DEFAULT_DATA_RATE);
        fprintf(logfp, "Initial priorities:");
        for (int i = 0; i < NUM_LANES; i++)
            fprintf(logfp, " [%d]=%d", i, taskList[i].priority);
        fprintf(logfp, "\n");
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
        tick++;
        updateLaneTick();

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
            Task *t = &taskList[chosen];
            t->task_run(&t->lane, NULL);
        }

        if (chosen < 0 && pll_enabled) {
            goto exit;
        }

        usleep(10);    /* simulate firmware time slice */
    }

exit:
    return 0;
}