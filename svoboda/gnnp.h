/***
 * Svoboda
 * --------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
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

#ifndef __SVO_GNNP_H__
#define __SVO_GNNP_H__

#include <boruvka/core.h>
#include <boruvka/net.h>
#include <boruvka/nn.h>
#include <boruvka/dij.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _svo_gnnp_t;

/**
 * Growing Neural Network for Planning
 * ====================================
 * See svo_gnnp_t.
 */

struct _svo_gnnp_node_t {
    bor_vec_t *w;  /*!< Weight vector */
    bor_net_node_t net;
    bor_nn_el_t nn;
    bor_nn_el_t nn_path;

    uint8_t fixed; /*!< True if node is fixed (1 - FREE, 2 - OBST) */

    bor_list_t path;
    struct _svo_gnnp_node_t *prev; /*!< Learned path from node towards
                                        init or goal node */
    uint8_t prev_type;             /*!< prev is: 1 -> init, 2 -> goal */

    int _id;
};
typedef struct _svo_gnnp_node_t svo_gnnp_node_t;


/**
 * Operators
 * ----------
 * See svo_gnnp_ops_t.
 */

/** vvvv */

/**
 * Returns random input signal.
 */
typedef const bor_vec_t *(*svo_gnnp_input_signal)(struct _svo_gnnp_t *nn, void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_gnnp_terminate)(struct _svo_gnnp_t *nn, void *);

/**
 * Returns true if it {conf} is in free space
 */
typedef int (*svo_gnnp_eval)(struct _svo_gnnp_t *nn, const bor_vec_t *conf, void *);

/**
 * Callback that is peridically called from GNG.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_gnnp_callback)(struct _svo_gnnp_t *nn, void *);

/** ^^^^ */

struct _svo_gnnp_ops_t {
    svo_gnnp_input_signal input_signal; /*!< Default: NULL */
    svo_gnnp_terminate    terminate;    /*!< Default: NULL */
    svo_gnnp_eval         eval;         /*!< Default: NULL */

    svo_gnnp_callback callback;
    unsigned int callback_period;

    void *data;
    void *input_signal_data;
    void *terminate_data;
    void *eval_data;
    void *callback_data;
};
typedef struct _svo_gnnp_ops_t svo_gnnp_ops_t;

/**
 * Initializes operators struct
 */
void svoGNNPOpsInit(svo_gnnp_ops_t *ops);


/**
 * Parameters
 * -----------
 */
struct _svo_gnnp_params_t {
    int dim;           /*!< Dimension. Default: 2 */
    bor_real_t ew;     /*!< Winner node's learning rate. Default: 0.05 */
    bor_real_t en;     /*!< Winner neighbor's learning rate. Default: 0.0005 */
    unsigned int rmax; /*!< Max rank of node. Default: 4 */
    bor_real_t h;      /*!< Resolution. Default: 0.1 */

    bor_nn_params_t nn; /*!< Params of nearest neighbor search */
};
typedef struct _svo_gnnp_params_t svo_gnnp_params_t;

/**
 * Initialize parameters
 */
void svoGNNPParamsInit(svo_gnnp_params_t *p);


/**
 * Functions
 * ----------
 */

struct _svo_gnnp_t {
    svo_gnnp_ops_t ops;
    svo_gnnp_params_t params;
    bor_net_t *net;
    bor_nn_t *nn;
    bor_nn_t *nn_path;

    svo_gnnp_node_t *init, *goal; /*!< Init and goal nodes */
    bor_vec_t *tmpv;
};
typedef struct _svo_gnnp_t svo_gnnp_t;

/**
 * Creates network
 */
svo_gnnp_t *svoGNNPNew(const svo_gnnp_ops_t *ops, const svo_gnnp_params_t *p);

/**
 * Deletes network
 */
void svoGNNPDel(svo_gnnp_t *nn);


/**
 * Finds path between start and goal.
 * If path was found, {path} argument is filled and 0 is returned.
 */
int svoGNNPFindPath(svo_gnnp_t *nn,
                    const bor_vec_t *start, const bor_vec_t *goal,
                    bor_list_t *path);

/**
 * Dumps net (if it is 2-D or 3-D) as SVT object
 */
void svoGNNPDumpSVT(const svo_gnnp_t *nn, FILE *out, const char *name);

/**
 * Returns number of nodes in network
 */
_bor_inline size_t svoGNNPNodesLen(const svo_gnnp_t *nn);

/**
 * Returns network
 */
_bor_inline bor_net_t *svoGNNPNet(svo_gnnp_t *nn);

/**** INLINES ****/
_bor_inline size_t svoGNNPNodesLen(const svo_gnnp_t *nn)
{
    return borNetNodesLen(nn->net);
}

_bor_inline bor_net_t *svoGNNPNet(svo_gnnp_t *nn)
{
    return nn->net;
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GNNP_H__ */
