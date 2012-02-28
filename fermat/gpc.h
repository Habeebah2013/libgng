/***
 * fermat
 * -------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of fermat.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef __FER_GPC_H__
#define __FER_GPC_H__

#include <fermat/core.h>
#include <fermat/rand-mt.h>
#include <fermat/gpc-data.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _fer_gpc_t;
struct _fer_gpc_tree_t;
struct _fer_gpc_pred_t;
struct _fer_gpc_class_t;


/**
 * Genetic Programming for Data Classification
 * ============================================
 *
 */


/**
 * Operators
 * ----------
 *
 * See fer_gpc_ops_t.
 */

/** vvvv */

/**
 * Returns fitness of a decision tree.
 * The argument {class} is array of classification results per data row.
 */
typedef fer_real_t (*fer_gpc_fitness)(struct _fer_gpc_t *gpc, int *class, void *);

/**
 * Ought to return {i}'th row from dataset
 */
typedef void *(*fer_gpc_data_row)(struct _fer_gpc_t *gpc, int i, void *);

/**
 * Callback that is called periodically every .callback_period'th cycle.
 */
typedef void (*fer_gpc_callback)(struct _fer_gpc_t *gpc, void *);
/** ^^^^ */

struct _fer_gpc_ops_t {
    fer_gpc_fitness fitness;       /*!< Default: NULL, must be defined */
    fer_gpc_data_row data_row;     /*!< Default: NULL, must be defined */

    fer_gpc_callback callback;     /*!< Default: NULL */
    unsigned long callback_period; /*!< Default: 0, i.e., never */

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */
    void *fitness_data;
    void *data_row_data;
    void *callback_data;
};
typedef struct _fer_gpc_ops_t fer_gpc_ops_t;

/**
 * Initializes operators to default values
 */
void ferGPCOpsInit(fer_gpc_ops_t *ops);



/**
 * Parameters
 * -----------
 */
struct _fer_gpc_params_t {
    fer_real_t pc;         /*!< Probability of crossover. Default: 0.7 */
    fer_real_t pm;         /*!< Probability of mutation. Default: 0.001 */
    size_t pop_size;       /*!< Size of population. Default: 1 */
    size_t max_depth;      /*!< Maximal depth of a tree. Default: 5 */
    size_t data_rows;      /*!< Number of data rows. Default: 0 */
};
typedef struct _fer_gpc_params_t fer_gpc_params_t;

/**
 * Initialize parameters to default values
 */
void ferGPCParamsInit(fer_gpc_params_t *params);



/**
 * Functions
 * ----------
 */

#define FER_GPC_PRED_INIT_SIZE 10
#define FER_GPC_CLASS_INIT_SIZE 10

struct _fer_gpc_t {
    fer_gpc_params_t params;
    fer_gpc_ops_t ops;

    fer_rand_mt_t *rand;

    struct _fer_gpc_tree_t **pop[2]; /*!< Population (actual and tmp) */
    size_t pop_cur;                  /*!< 0/1 - actual population */

    struct _fer_gpc_pred_t *pred;    /*!< List of predicates */
    size_t pred_size, pred_len;
    struct _fer_gpc_class_t *class;  /*!< List of classes */
    size_t class_size, class_len;

    int *eval_results; /*!< Array of results from evaluation */
};
typedef struct _fer_gpc_t fer_gpc_t;

/**
 * Creates a new instance of GPC
 */
fer_gpc_t *ferGPCNew(const fer_gpc_ops_t *ops, const fer_gpc_params_t *params);

/**
 * Deletes GPC
 */
void ferGPCDel(fer_gpc_t *gpc);


typedef unsigned int (*fer_gpc_pred)(fer_gpc_t *gpc, void *mem, void *data, void *userdata);
typedef void (*fer_gpc_pred_init)(fer_gpc_t *gpc, void *mem, void *userdata);

/**
 * Adds a new predicate.
 * TODO
 */
int ferGPCAddPred(fer_gpc_t *gpc,
                  fer_gpc_pred pred, fer_gpc_pred_init init,
                  unsigned int num_descendants, size_t memsize,
                  void *userdata);

/**
 * Adds a new class (terminal).
 */
int ferGPCAddClass(fer_gpc_t *gpc, int class_id);



/**
 * Run GPC algorithm
 */
int ferGPCRun(fer_gpc_t *gpc);

/**
 * Returns random number from range
 */
_fer_inline fer_real_t ferGPCRand(fer_gpc_t *gpc, fer_real_t f, fer_real_t t);

/**
 * Returns random number [0, 1)
 */
_fer_inline fer_real_t ferGPCRand01(fer_gpc_t *gpc);

/**
 * Returns random integer number [f, t)
 */
_fer_inline int ferGPCRandInt(fer_gpc_t *gpc, int f, int t);


/**** INLINES ****/
_fer_inline fer_real_t ferGPCRand(fer_gpc_t *gpc, fer_real_t f, fer_real_t t)
{
    return ferRandMT(gpc->rand, f, t);
}

_fer_inline fer_real_t ferGPCRand01(fer_gpc_t *gpc)
{
    return ferRandMT01(gpc->rand);
}

_fer_inline int ferGPCRandInt(fer_gpc_t *gpc, int f, int t)
{
    return ferRandMT(gpc->rand, f, t);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __FER_GPC_H__ */
