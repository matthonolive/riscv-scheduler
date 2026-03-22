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
    int (*task_run)(void *task_data, void *task_args); // Function pointer to the task's run function
    int priority;
    char is_active; // 1 if the task should be sccheduled, 0 if it is done or should not be scheduled
} Task;

int rr_index = 0;
int pll_enabled = 1;
FILE *logfp = NULL;
int tick = 0;

typedef struct {
    Task *task_buffer; // Initialized in main as an arrayList of Task structs
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
    taskList.task_buffer = calloc(taskList.task_buffer_capacity, sizeof(Task));
    if (!taskList.task_buffer) {
        perror("calloc(task_buffer)");
        return 1;
    }
    taskList.task_buffer_size = 0;

    /* Initialize all lanes */
    for (int i = 0; i < NUM_LANES; i++) {
        Task *cur_task = &taskList.task_buffer[i];      // <- FIXED pointer arithmetic
        /* pass the LaneContext pointer, not the address of the pointer */
        generic_lane_init(&cur_task->task_data, &(LaneInitArgs){.dataRateGbps = DEFAULT_DATA_RATE, .channel_file = channel_file});
        cur_task->task_run = generic_lane_step;
        cur_task->priority = random_prio ? (rand() % NUM_LANES) : 1;
        cur_task->is_active = 1;
    }
    taskList.task_buffer_size = NUM_LANES; // set size explicitly

    printf("Scheduler started with channel '%s'.\n", channel_file);
    printf("Priority mode: %s\n", random_prio ? "RANDOM" : "EQUAL");
    printf("Logs → %s\n", LOG_FILE);

    /* print initial priorities */
    printf("Initial priorities:");
    for (int i = 0; i < NUM_LANES; i++)
        printf(" [%d]=%d", i, taskList.task_buffer[i].priority);
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
            fprintf(logfp, " [%d]=%d", i, taskList.task_buffer[i].priority);
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
                char buf[128];
                if (!fgets(buf, sizeof(buf), stdin)) {
                    stdin_open = 0;   /* EOF — stop polling */
                } else {
                    LaneStepArgs step_args;
                    memset(&step_args, 0, sizeof(step_args));

                    /* simple parsing */
                    if (buf[0] == 's') {
                        int lane = -1;
                        if (sscanf(buf, "s %d", &lane) == 1) {
                            if (lane >= 0 && lane < NUM_LANES) {
                                step_args.flags = PRINT_STATUS;
                                Task *t = &taskList.task_buffer[lane];
                                t->task_run(t->task_data, &step_args);
                                if (logfp) fprintf(logfp, "[tick %8d] CMD: status query lane %d\n", tick, lane);
                            } else {
                                printf("Invalid lane %d\n", lane);
                            }
                        } else {
                            /* print all lanes */
                            for (int i = 0; i < NUM_LANES; i++) {
                                step_args.flags = PRINT_STATUS;
                                Task *t = &taskList.task_buffer[i];
                                t->task_run(t->task_data, &step_args);
                            }
                            if (logfp) fprintf(logfp, "[tick %8d] CMD: status query ALL\n", tick);
                        }
                    }
                    else if (buf[0] == 'd') {
                        int lane, rate;
                        if (sscanf(buf, "d %d %d", &lane, &rate) == 2) {
                            if (lane >= 0 && lane < NUM_LANES) {
                                step_args.flags = DATA_RATE_CHANGE;
                                step_args.dataRateGbps = rate;
                                Task *t = &taskList.task_buffer[lane];
                                t->task_run(t->task_data, &step_args);
                                if (logfp) fprintf(logfp, "[tick %8d] CMD: lane %d rate → %d Gbps\n", tick, lane, rate);
                            } else {
                                printf("Invalid lane %d\n", lane);
                            }
                        } else {
                            printf("Usage: d <lane> <rate>\n");
                        }
                    }
                    else if (buf[0] == 'r') {
                        int lane;
                        if (sscanf(buf, "r %d", &lane) == 1) {
                            if (lane >= 0 && lane < NUM_LANES) {
                                step_args.flags = SOFT_RESET;
                                Task *t = &taskList.task_buffer[lane];
                                t->task_run(t->task_data, &step_args);
                                if (logfp) fprintf(logfp, "[tick %8d] CMD: lane %d soft reset\n", tick, lane);
                            } else {
                                printf("Invalid lane %d\n", lane);
                            }
                        } else {
                            printf("Usage: r <lane>\n");
                        }
                    }
                    else if (buf[0] == 'p') {
                        pll_enabled = !pll_enabled;
                        printf("PLL %s\n", pll_enabled ? "ON" : "OFF");
                        if (logfp) fprintf(logfp, "[tick %8d] CMD: PLL %s\n", tick, pll_enabled ? "ON" : "OFF");
                    }
                }
            }
        }

        /* -------- SCHEDULING -------- */

        int best = 999999;
        int chosen = -1;

        /* Find best priority */
        for (int i = 0; i < NUM_LANES; i++) {
            if (taskList.task_buffer[i].is_active == 1 &&
                taskList.task_buffer[i].priority < best)
            {
                best = taskList.task_buffer[i].priority;
            }
        }

        /* Round-robin among best */
        for (int k = 0; k < NUM_LANES; k++) {
            int i = (rr_index + k) % NUM_LANES;

            if (taskList.task_buffer[i].is_active == 1 &&
                taskList.task_buffer[i].priority == best)
            {
                chosen = i;
                break;
            }
        }

        if (chosen >= 0 && pll_enabled) {
            rr_index = (chosen + 1) % NUM_LANES;
            Task *t = &taskList.task_buffer[chosen];

            LaneStepArgs step_args;
            memset(&step_args, 0, sizeof(step_args));
            step_args.flags = NO_INTERRUPT; /* normal scheduled step; use other flags for interrupts */

            int ret = t->task_run(t->task_data, &step_args);
            if (ret != 0) {
                t->is_active = 0; // Mark task as inactive if it returns DONE
            }
        }

        if (chosen < 0 && pll_enabled) {
            goto exit;
        }

        usleep(10);    /* simulate firmware time slice */
    }

exit:
    return 0;
}