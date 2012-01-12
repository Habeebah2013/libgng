/***
 * fermat
 * -------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
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

#ifndef __FER_KOHONEN_H__
#define __FER_KOHONEN_H__

#include <fermat/core.h>
#include <fermat/net.h>
#include <fermat/nn.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _fer_kohonen_t;

/**
 * Kohonen Map
 * ============
 *
 * See fer_kohonen_t.
 */

struct _fer_kohonen_node_t {
    fer_vec_t *w;

    fer_net_node_t net;
    fer_nn_el_t nn;

    fer_list_t fifo;
    unsigned int update;
    unsigned int depth;
    int8_t fixed;

    int _id;
};
typedef struct _fer_kohonen_node_t fer_kohonen_node_t;

/**
 * Operators
 * ----------
 *
 * See fer_kohonen_ops_t.
 */

/**
 * Returns input signal
 */
typedef const fer_vec_t *(*fer_kohonen_input_signal)(struct _fer_kohonen_t *k, void *);

/**
 * Returns 0 if {cur} (which is in {depth} in BFS manner) is still part of
 * {center}'s neighborhood and fills with the value of strength.
 */
typedef int (*fer_kohonen_neighborhood)(struct _fer_kohonen_t *k,
                                        const fer_kohonen_node_t *center,
                                        const fer_kohonen_node_t *cur,
                                        int depth,
                                        fer_real_t *val, void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*fer_kohonen_terminate)(struct _fer_kohonen_t *k, void *);

/**
 * Callback that is called peridically every .callback_period'th cycle.
 */
typedef void (*fer_kohonen_callback)(struct _fer_kohonen_t *k, void *);


struct _fer_kohonen_ops_t {
    fer_kohonen_input_signal input_signal;
    fer_kohonen_neighborhood neighborhood;
    fer_kohonen_terminate    terminate;
    fer_kohonen_callback     callback;

    void *data;
    void *input_signal_data;
    void *neighborhood_data;
    void *terminate_data;
    void *callback_data;

    unsigned long callback_period; /*!< Default: 100 */
};
typedef struct _fer_kohonen_ops_t fer_kohonen_ops_t;

/**
 * Initializes operators struct
 */
void ferKohonenOpsInit(fer_kohonen_ops_t *ops);


/**
 * Parameters
 * -----------
 */
struct _fer_kohonen_params_t {
    int dim;               /*!< Dimensionality. Default: 2 */
    fer_real_t learn_rate; /*!< Learning rate */
    fer_nn_params_t nn;    /*!< Nearest neighbor search params */
};
typedef struct _fer_kohonen_params_t fer_kohonen_params_t;

/**
 * Initializes parameters
 */
void ferKohonenParamsInit(fer_kohonen_params_t *p);

struct _fer_kohonen_t {
    fer_kohonen_ops_t ops;
    fer_kohonen_params_t params;

    fer_net_t *net;
    fer_nn_t *nn;

    unsigned int update;

    fer_vec_t *tmpv;
};
typedef struct _fer_kohonen_t fer_kohonen_t;


/**
 * Functions
 * ----------
 */

/**
 * Creates Kohonen Map
 */
fer_kohonen_t *ferKohonenNew(const fer_kohonen_ops_t *ops,
                             const fer_kohonen_params_t *params);

/**
 * Deletes kohonen map
 */
void ferKohonenDel(fer_kohonen_t *k);


/**
 * Runs Kohonen Map algorithm
 */
void ferKohonenRun(fer_kohonen_t *k);

/**
 * Dumps 2D and 3D kohonen map in SVT format
 */
void ferKohonenDumpSVT(const fer_kohonen_t *k, FILE *out, const char *name);

/**
 * Node functions
 * ---------------
 */

/**
 * Creates new node
 */
fer_kohonen_node_t *ferKohonenNodeNew(fer_kohonen_t *k, const fer_vec_t *init);

/**
 * Deletes node
 */
void ferKohonenNodeDel(fer_kohonen_t *k, fer_kohonen_node_t *n);

/**
 * Connects {n1} and {n2} nodes
 */
void ferKohonenNodeConnect(fer_kohonen_t *k,
                           fer_kohonen_node_t *n1,
                           fer_kohonen_node_t *n2);

/**
 * Returns true if node is fixed.
 */
_fer_inline int8_t ferKohonenNodeFixed(const fer_kohonen_node_t *n);

/**
 * Set a node as fixed
 */
_fer_inline void ferKohonenNodeSetFixed(fer_kohonen_node_t *n, int8_t fixed);

/**** INLINES ****/
_fer_inline int8_t ferKohonenNodeFixed(const fer_kohonen_node_t *n)
{
    return n->fixed;
}

_fer_inline void ferKohonenNodeSetFixed(fer_kohonen_node_t *n, int8_t fixed)
{
    n->fixed = fixed;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __FER_KOHONEN_H__ */