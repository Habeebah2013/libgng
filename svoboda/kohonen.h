/***
 * Svoboda
 * --------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
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

#ifndef __SVO_KOHONEN_H__
#define __SVO_KOHONEN_H__

#include <boruvka/core.h>
#include <boruvka/net.h>
#include <boruvka/nn.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _svo_kohonen_t;

/**
 * Kohonen Map
 * ============
 *
 * See svo_kohonen_t.
 */

struct _svo_kohonen_node_t {
    bor_vec_t *w;

    bor_net_node_t net;
    bor_nn_el_t nn;

    bor_list_t fifo;
    unsigned int update;
    unsigned int depth;
    int8_t fixed;

    int _id;
};
typedef struct _svo_kohonen_node_t svo_kohonen_node_t;

/**
 * Operators
 * ----------
 *
 * See svo_kohonen_ops_t.
 */

/**
 * Returns input signal
 */
typedef const bor_vec_t *(*svo_kohonen_input_signal)(struct _svo_kohonen_t *k, void *);

/**
 * Returns 0 if {cur} (which is in {depth} in BFS manner) is still part of
 * {center}'s neighborhood and fills with the value of strength.
 */
typedef int (*svo_kohonen_neighborhood)(struct _svo_kohonen_t *k,
                                        const svo_kohonen_node_t *center,
                                        const svo_kohonen_node_t *cur,
                                        int depth,
                                        bor_real_t *val, void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_kohonen_terminate)(struct _svo_kohonen_t *k, void *);

/**
 * Callback that is called peridically every .callback_period'th cycle.
 */
typedef void (*svo_kohonen_callback)(struct _svo_kohonen_t *k, void *);


struct _svo_kohonen_ops_t {
    svo_kohonen_input_signal input_signal;
    svo_kohonen_neighborhood neighborhood;
    svo_kohonen_terminate    terminate;
    svo_kohonen_callback     callback;

    void *data;
    void *input_signal_data;
    void *neighborhood_data;
    void *terminate_data;
    void *callback_data;

    unsigned long callback_period; /*!< Default: 100 */
};
typedef struct _svo_kohonen_ops_t svo_kohonen_ops_t;

/**
 * Initializes operators struct
 */
void svoKohonenOpsInit(svo_kohonen_ops_t *ops);


/**
 * Parameters
 * -----------
 */
struct _svo_kohonen_params_t {
    int dim;               /*!< Dimensionality. Default: 2 */
    bor_real_t learn_rate; /*!< Learning rate */
    bor_nn_params_t nn;    /*!< Nearest neighbor search params */
};
typedef struct _svo_kohonen_params_t svo_kohonen_params_t;

/**
 * Initializes parameters
 */
void svoKohonenParamsInit(svo_kohonen_params_t *p);

struct _svo_kohonen_t {
    svo_kohonen_ops_t ops;
    svo_kohonen_params_t params;

    bor_net_t *net;
    bor_nn_t *nn;

    unsigned int update;

    bor_vec_t *tmpv;
};
typedef struct _svo_kohonen_t svo_kohonen_t;


/**
 * Functions
 * ----------
 */

/**
 * Creates Kohonen Map
 */
svo_kohonen_t *svoKohonenNew(const svo_kohonen_ops_t *ops,
                             const svo_kohonen_params_t *params);

/**
 * Deletes kohonen map
 */
void svoKohonenDel(svo_kohonen_t *k);


/**
 * Runs Kohonen Map algorithm
 */
void svoKohonenRun(svo_kohonen_t *k);

/**
 * Dumps 2D and 3D kohonen map in SVT format
 */
void svoKohonenDumpSVT(const svo_kohonen_t *k, FILE *out, const char *name);

/**
 * Node functions
 * ---------------
 */

/**
 * Creates new node
 */
svo_kohonen_node_t *svoKohonenNodeNew(svo_kohonen_t *k, const bor_vec_t *init);

/**
 * Deletes node
 */
void svoKohonenNodeDel(svo_kohonen_t *k, svo_kohonen_node_t *n);

/**
 * Connects {n1} and {n2} nodes
 */
void svoKohonenNodeConnect(svo_kohonen_t *k,
                           svo_kohonen_node_t *n1,
                           svo_kohonen_node_t *n2);

/**
 * Returns true if node is fixed.
 */
_bor_inline int8_t svoKohonenNodeFixed(const svo_kohonen_node_t *n);

/**
 * Set a node as fixed
 */
_bor_inline void svoKohonenNodeSetFixed(svo_kohonen_node_t *n, int8_t fixed);

/**** INLINES ****/
_bor_inline int8_t svoKohonenNodeFixed(const svo_kohonen_node_t *n)
{
    return n->fixed;
}

_bor_inline void svoKohonenNodeSetFixed(svo_kohonen_node_t *n, int8_t fixed)
{
    n->fixed = fixed;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SVO_KOHONEN_H__ */
