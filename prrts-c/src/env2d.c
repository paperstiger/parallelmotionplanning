#include <math.h>
#include <assert.h>
#include <err.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "linear.h"
#include "linear_inline.h"
#include "collide.h"
#include "alloc.h"
#include "prrts.h"
#include "hrtimer.h"
#include "naocup.h"

#define DIMENSIONS 2

static const double nao_init_config[] = {
    0, 0
};

static const double nao_min_config[] = {
    0, 0
};

static const double nao_max_config[] = {
    1, 1
};

static const double nao_target_config[] = {
    1, 1
};

static double
nao_dist(const double *a, const double *b)
{
        double sum = 0;
        double d;
        int i;

        for (i=0 ; i<DIMENSIONS ; ++i) {
                d = b[i] - a[i];
                sum += d*d;
        }

        return sqrt(sum);
}

static bool
nao_in_goal(void *system_arg, const double *config)
{
        printf("call in goal");
        return false;
}

static bool
nao_clear(void *system_arg, const double *config)
{
    return true;
}

typedef void nao_world_t;

static bool
nao_link_impl(nao_world_t *world, const double *a, const double *b)
{
    // use my analytic result
    double x1 = a[0], x2 = b[0], y1 = a[1], y2 = b[1];
    if((x1 - 0.5) * (x2 - 0.5) >= 0)
        return true;
    else{
        double factor = (0.5 - x2) / (x1 - x2);
        double interpy = y2 + factor * (y1 - y2);
        return (interpy > 0.8) | (interpy < 0.2);
    }
}

static bool
nao_link(void *system_arg, const double *a, const double *b)
{
        nao_world_t *world = (nao_world_t*)system_arg;

        return nao_link_impl(world, a, b);
}

static void *
nao_system_data_alloc(int thread_no, const double *sample_min, const double *sample_max)
{
    return NULL;
}

static void
nao_system_data_free(void *system)
{
    return;  // nothing to do
}

prrts_system_t *
env2d_create_system()
{
        prrts_system_t *system;

        if ((system = struct_alloc(prrts_system_t)) == NULL)
                return NULL;

        memset(system, 0, sizeof(system));

        system->dimensions = DIMENSIONS;

        system->init = nao_init_config;
        system->min = nao_min_config;
        system->max = nao_max_config;
        system->target = nao_target_config;

        system->system_data_alloc_func = nao_system_data_alloc;
        system->system_data_free_func = nao_system_data_free;
        system->dist_func = nao_dist;
        system->in_goal_func = nao_in_goal;
        system->clear_func = nao_clear;
        system->link_func = nao_link;

        return system;
}

void
env2d_free_system(prrts_system_t *system)
{
        free(system);
}

static void
usage(char *prog_name)
{
        printf("usage: %s -t thread_count -n sample_count -r\n", prog_name);
        exit(1);
}

static void
print_solution(prrts_solution_t *solution)
{
        int i, j;
        double sum;

        printf("angleList = [\\\n");
        for (i=0 ; i<DIMENSIONS ; ++i) {
                printf("  [ ");
                for (j=0 ; j<solution->path_length ; ++j) {
                        printf("%f, ", solution->configs[j][i]);
                }
                printf("], \\\n");
        }
        printf("]\n");
        printf("times = [\\\n");
        sum = 0.0;
        printf("  [ ");
        for (j=1 ; j<solution->path_length ; ++j) {
                sum += nao_dist(solution->configs[j-1], solution->configs[j]);
                printf("%f, ", sum);
        }
        printf("], \\\n");
        printf("]\n");
}


int
vec2d_main(int argc, char *argv[])
{   
    long thread_count = 1;
    long sample_count = 10000;
    //for (int i = 0; i<totalIterations;i++) {
        prrts_system_t *system;
        prrts_options_t options;
        prrts_solution_t *solution;
        //long thread_count = 4;
        int ch;
        //long sample_count = 40000;
        hrtimer_t start_time;
        hrtimer_t duration;
        char *ep;

        memset(&options, 0, sizeof(options));
        options.gamma = 5.0;
        options.regional_sampling = true;
        options.samples_per_step = 1;

        while ((ch = getopt(argc, argv, "t:n:r")) != -1) {
                switch (ch) {
                case 't':
                        thread_count = strtol(optarg, &ep, 10);
                        if (thread_count < 1 || *ep != '\0') {
                                warnx("invalid thread count -t argument -- %s", optarg);
                                usage(argv[0]);
                        }
                        break;                                
                case 'n':
                        sample_count = strtol(optarg, &ep, 10);
                        if (sample_count < 1 || *ep != '\0') {
                                warnx("invalid sample count -n argument -- %s", optarg);
                                usage(argv[0]);
                        }
                        break;
                case 'r':
                        options.regional_sampling = true;
                        break;
                case '?':
                default:
                        usage(argv[0]);
                }
        }

        system = env2d_create_system();

        start_time = hrtimer_get();
        solution = prrts_run_for_samples(system, &options, thread_count, sample_count);
        duration = hrtimer_get() - start_time;

        env2d_free_system(system);

        printf("%d samples computed in %f seconds (%f cpu-seconds) on %d threads (%f samples/second)\n",
               (int)sample_count,
               duration / (double)HRTICK,
               duration * thread_count / (double)HRTICK,
               (int)thread_count,
               sample_count * (double)HRTICK / duration);
        //listOfTimes[i]=duration / (double)HRTICK;
        if (solution == NULL) {
                printf("No solution found\n");
        } else {
                printf("Solution found, length=%u, cost=%f\n", (unsigned)solution->path_length, solution->path_cost);
                print_solution(solution);
                //listOfCosts[i] = (solution->path_cost);
        }

        printf("===================================\n");
    //}
    
    // printf("Times: \n");
    // for(int i=0;i < totalIterations;i++){
    // printf("%f \n",listOfTimes[i]);}
    // printf("Costs: \n");
    // for(int i=0;i < totalIterations;i++){
    // printf("%f \n",listOfCosts[i]);}
        return 0;
}
