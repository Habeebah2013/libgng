/***
 * Svoboda
 * --------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of Svoboda.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef __SVO_GPC_H__
#define __SVO_GPC_H__

#include <boruvka/core.h>
#include <boruvka/rand-mt.h>
#include <boruvka/tasks.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _svo_gpc_t;
struct _svo_gpc_tree_t;
struct _svo_gpc_pred_t;
struct _svo_gpc_class_t;


/**
 * Genetic Programming for Data Classification
 * ============================================
 *
 */


/**
 * Operators
 * ----------
 *
 * See svo_gpc_ops_t.
 */

/** vvvv */

/**
 * Returns fitness of a decision tree.
 * The argument {class} is array of classification results per data row.
 */
typedef bor_real_t (*svo_gpc_fitness)(struct _svo_gpc_t *gpc, int *class, void *);

/**
 * Ought to return {i}'th row from dataset
 */
typedef void *(*svo_gpc_data_row)(struct _svo_gpc_t *gpc, int i, void *);

/**
 * Callback that is called periodically every .callback_period'th cycle.
 */
typedef void (*svo_gpc_callback)(struct _svo_gpc_t *gpc, void *);
/** ^^^^ */

struct _svo_gpc_ops_t {
    svo_gpc_fitness fitness;       /*!< Default: NULL, must be defined */
    svo_gpc_data_row data_row;     /*!< Default: NULL, must be defined */

    svo_gpc_callback callback;     /*!< Default: NULL */
    unsigned long callback_period; /*!< Default: 0, i.e., never */

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */
    void *fitness_data;
    void *data_row_data;
    void *callback_data;
};
typedef struct _svo_gpc_ops_t svo_gpc_ops_t;

/**
 * Initializes operators to default values
 */
void svoGPCOpsInit(svo_gpc_ops_t *ops);



/**
 * Parameters
 * -----------
 */
struct _svo_gpc_params_t {
    int pop_size;            /*!< Size of population. Default: 1 */
    int max_depth;           /*!< Maximal depth of a tree. Default: 5 */
    int keep_best;           /*!< Number of best individuals that ought to
                                  be reproduced to next population
                                  preferentialy. Default: 1 */
    int throw_worst;         /*!< Number of best individuals that are
                                  thrown away preferentialy. Default: 1 */
    unsigned long max_steps; /*!< Maximal number of steps of algorithm.
                                  Default: 10 */
    int data_rows;           /*!< Number of data rows. Default: 0 */

    int tournament_size;     /*!< Number of individuals that enter
                                  tournament selection. Default: 5 */

    /* Probabilities of undergone actions. Any numbers can be used because
     * all will be normized to (pr + pc + pm) = 1 */
    bor_real_t pr;  /*!< Probability of reproduction. Default: 14 */
    bor_real_t pc;  /*!< Probability of crossover. Default: 85 */
    bor_real_t pm;  /*!< Probability of mutation. Default: 1 */

    unsigned long simplify;      /*!< A simplification of the trees will be
                                      executed every {simplify} step.
                                      Default: 0 (i.e., never) */
    unsigned long prune_deep;    /*!< Prunes all trees that exceeds max_depth
                                      every specified step. Default: 0 */
    unsigned long rm_duplicates; /*!< Freq of removing duplicates. Default: 0 */
    unsigned long inc_max_depth; /*!< Freq of increasing a max. depth by
                                      .inc_max_depth_step. Default: 0 */
    int inc_max_depth_step;      /*!< Default: 1 */

    int parallel; /*!< Number of parallel threads that will be used.
                       Default: 0 */
};
typedef struct _svo_gpc_params_t svo_gpc_params_t;

/**
 * Initialize parameters to default values
 */
void svoGPCParamsInit(svo_gpc_params_t *params);



/**
 * Functions
 * ----------
 */

#define SVO_GPC_PRED_INIT_SIZE 10
#define SVO_GPC_CLASS_INIT_SIZE 10

struct _svo_gpc_t {
    svo_gpc_params_t params;
    svo_gpc_ops_t ops;

    bor_rand_mt_t *rand;

    struct _svo_gpc_tree_t **pop[3]; /*!< Population (actual and two tmps) */
    int pop_size[2];                 /*!< Current size of populations */
    int pop_cur;                     /*!< Idx of current population array */

    struct _svo_gpc_pred_t *pred;    /*!< List of predicates */
    int pred_size, pred_len;
    struct _svo_gpc_class_t *class;  /*!< List of classes */
    int class_size, class_len;

    int threads;
    bor_tasks_t *tasks;
    int **eval_results; /*!< Array of results from evaluation */


    struct {
        unsigned long elapsed; /*!< Number of elapsed steps */
    } stats;
};
typedef struct _svo_gpc_t svo_gpc_t;

/**
 * Creates a new instance of GPC
 */
svo_gpc_t *svoGPCNew(const svo_gpc_ops_t *ops, const svo_gpc_params_t *params);

/**
 * Deletes GPC
 */
void svoGPCDel(svo_gpc_t *gpc);

/**
 * Returns current max. depth of a tree individual.
 */
int svoGPCMaxDepth(const svo_gpc_t *gpc);

/**
 * Predicate callback.
 * Should return the number of next descendant node in evaluating a
 * decision tree.
 */
typedef int (*svo_gpc_pred)(svo_gpc_t *gpc, void *mem, void *data, void *userdata);

/**
 * Initialize callback a predicate data
 */
typedef void (*svo_gpc_pred_init)(svo_gpc_t *gpc, void *mem, void *userdata);

/**
 * Format a callback into given string.
 */
typedef void (*svo_gpc_pred_format)(svo_gpc_t *gpc, void *mem, void *userdata,
                                    char *str, size_t str_maxlen);

/**
 * Adds a new predicate.
 * TODO
 */
int svoGPCAddPred(svo_gpc_t *gpc,
                  svo_gpc_pred pred,
                  svo_gpc_pred_init init,
                  svo_gpc_pred_format format,
                  int num_descendants, size_t memsize,
                  void *userdata);

/**
 * Adds a new class (terminal).
 */
int svoGPCAddClass(svo_gpc_t *gpc, int class_id);



/**
 * Run GPC algorithm
 */
int svoGPCRun(svo_gpc_t *gpc);

/**
 * Returns fitness of the best individual
 */
bor_real_t svoGPCBestFitness(const svo_gpc_t *gpc);

/**
 * Returns i'th best tree from the current population
 */
void *svoGPCTree(const svo_gpc_t *gpc, int i);

/**
 * Evaluates a tree using the specified data and returns a resulting class.
 */
int svoGPCTreeEval(svo_gpc_t *gpc, void *tree, void *data);

/**
 * Returns a depth of the tree
 */
int svoGPCTreeDepth(svo_gpc_t *gpc, void *tree);

/**
 * Prints formated tree in form of C function to specified file.
 */
void svoGPCTreePrintC(svo_gpc_t *gpc, void *tree, const char *func_name, FILE *out);

/**
 * Returns random number from range
 */
_bor_inline bor_real_t svoGPCRand(svo_gpc_t *gpc, bor_real_t f, bor_real_t t);

/**
 * Returns random number [0, 1)
 */
_bor_inline bor_real_t svoGPCRand01(svo_gpc_t *gpc);

/**
 * Returns random integer number [f, t)
 */
_bor_inline int svoGPCRandInt(svo_gpc_t *gpc, int f, int t);


/**
 * Statistics
 * -----------
 */
struct _svo_gpc_stats_t {
    float min_fitness;
    float max_fitness;
    float avg_fitness;
    float med_fitness; /*!< median */

    int min_nodes;
    int max_nodes;
    float avg_nodes;

    int min_depth;
    int max_depth;
    float avg_depth;

    unsigned long elapsed; /*!< Number of elapsed steps */
};
typedef struct _svo_gpc_stats_t svo_gpc_stats_t;

/**
 * Fills given structure with statistics about from current population
 */
void svoGPCStats(const svo_gpc_t *gpc, svo_gpc_stats_t *stats);



int __svoGPCPredMemsize(const svo_gpc_t *gpc, int idx);

/**** INLINES ****/
_bor_inline bor_real_t svoGPCRand(svo_gpc_t *gpc, bor_real_t f, bor_real_t t)
{
    return borRandMT(gpc->rand, f, t);
}

_bor_inline bor_real_t svoGPCRand01(svo_gpc_t *gpc)
{
    return borRandMT01(gpc->rand);
}

_bor_inline int svoGPCRandInt(svo_gpc_t *gpc, int f, int t)
{
    return BOR_MIN(borRandMT(gpc->rand, f, t), t - 1);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GPC_H__ */

