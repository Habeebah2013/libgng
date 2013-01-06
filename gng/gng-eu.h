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

#ifndef __SVO_GNG_EU_H__
#define __SVO_GNG_EU_H__

#include <boruvka/net.h>
#include <boruvka/pairheap.h>
#include <boruvka/vec.h>
#include <boruvka/vec2.h>
#include <boruvka/vec3.h>
#include <boruvka/pc.h>
#include <boruvka/nn.h>
#include <boruvka/alloc.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/**
 * Growing Neural Gas In Euclidean Space
 * ======================================
 */


struct _svo_gng_eu_node_t {
    bor_net_node_t node;

    bor_real_t err;               /*!< Overall error */
    unsigned long err_cycle;      /*!< Last cycle in which were .err changed */
    bor_pairheap_node_t err_heap; /*!< Connection to error heap */

    bor_vec_t *w;   /*!< Weight vector */
    bor_nn_el_t nn; /*!< Struct for NN search */

    int _id; /*!< Currently useful only for svoGNGEuDumpSVT(). */
};
typedef struct _svo_gng_eu_node_t svo_gng_eu_node_t;


struct _svo_gng_eu_edge_t {
    bor_net_edge_t edge;

    int age;
};
typedef struct _svo_gng_eu_edge_t svo_gng_eu_edge_t;



/**
 * GNGEu Operations
 * -----------------
 *
 * See svo_gng_eu_ops_t.
 */

/** vvvv */

/**
 * Create new node initialized weight vector to input_signal.
 */
typedef svo_gng_eu_node_t *(*svo_gng_eu_new_node)(const bor_vec_t *input_signal, void *);

/**
 * Deletes given node.
 */
typedef void (*svo_gng_eu_del_node)(svo_gng_eu_node_t *n, void *);

/**
 * Returns random input signal.
 */
typedef const bor_vec_t *(*svo_gng_eu_input_signal)(void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_gng_eu_terminate)(void *);

/**
 * Callback that is peridically called from GNGEu.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_gng_eu_callback)(void *);

/** ^^^^ */

struct _svo_gng_eu_ops_t {
    svo_gng_eu_new_node     new_node;
    svo_gng_eu_del_node     del_node;
    svo_gng_eu_input_signal input_signal;
    svo_gng_eu_terminate    terminate;

    svo_gng_eu_callback callback;
    unsigned long callback_period;

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */
    void *new_node_data;
    void *del_node_data;
    void *input_signal_data;
    void *terminate_data;
    void *callback_data;
};
typedef struct _svo_gng_eu_ops_t svo_gng_eu_ops_t;


/**
 * Initializes ops struct to NULL values.
 */
void svoGNGEuOpsInit(svo_gng_eu_ops_t *ops);



/**
 * GNGEu Parameters
 * -----------------
 */
struct _svo_gng_eu_params_t {
    int dim;          /*!< Dimension */

    size_t lambda;    /*!< Number of steps between adding nodes */
    bor_real_t eb;    /*!< Winner node learning rate */
    bor_real_t en;    /*!< Winners' neighbors learning rate */
    bor_real_t alpha; /*!< Decrease error counter rate */
    bor_real_t beta;  /*!< Decrease error counter rate for all nodes */
    int age_max;      /*!< Maximal age of edge */

    bor_nn_params_t nn; /*!< Defines which algorithm will be used for
                             nearest neighbor search.
                             Default is Growing Uniform Grid with default
                             values */
};
typedef struct _svo_gng_eu_params_t svo_gng_eu_params_t;

/**
 * Initializes params struct to default values.
 */
void svoGNGEuParamsInit(svo_gng_eu_params_t *params);



/**
 * GNGEu Algorithm
 * ----------------
 *
 * See svo_gng_eu_t.
 */

struct _svo_gng_eu_t {
    bor_net_t *net;
    bor_pairheap_t *err_heap;

    svo_gng_eu_ops_t ops;
    svo_gng_eu_params_t params;

    bor_real_t *beta_n; /*!< Precomputed beta^n for n = 1, ..., lambda */
    bor_real_t *beta_lambda_n; /*!< Precomputed beta^(n*lambda) */
    size_t beta_lambda_n_len;

    size_t step;
    unsigned long cycle;

    bor_nn_t *nn;

    bor_vec_t *tmpv;
};
typedef struct _svo_gng_eu_t svo_gng_eu_t;


/**
 * Creates new instance of GNGEu algorithm.
 */
svo_gng_eu_t *svoGNGEuNew(const svo_gng_eu_ops_t *ops,
                     const svo_gng_eu_params_t *params);

/**
 * Deletes GNGEu.
 */
void svoGNGEuDel(svo_gng_eu_t *gng_eu);

/**
 * Runs GNGEu algorithm.
 *
 * This runs whole algorithm in loop until operation terminate() returns
 * true:
 * ~~~~~~
 * svoGNGEuinit()
 * do:
 *     for (step = 1 .. params.lambda):
 *         svoGNGEuLearn()
 *     svoGNGEuNewNode()
 * while not ops.terminate()
 */
void svoGNGEuRun(svo_gng_eu_t *gng_eu);


/**
 * Initialize gng_eu net.
 *
 * ~~~~~
 * if ops.init != NULL:
 *     ops.init()
 * else:
 *     is = ops.input_signal()
 *     n1 = ops.new_node(is)
 *
 *     is = ops.input_signal()
 *     n2 = ops.new_node(is)
 * create edge between n1 and n2
 */
void svoGNGEuInit(svo_gng_eu_t *gng_eu);

/**
 * One competitive hebbian learning step.
 */
void svoGNGEuLearn(svo_gng_eu_t *gng_eu);

/**
 * Creates new node in place with highest error counter.
 */
void svoGNGEuNewNode(svo_gng_eu_t *gng_eu);

/**
 * Performs hebbian learning between two given nodes - connection
 * between these two nodes is strengthened, i.e., edge is eigher created or
 * age of the existing edge is set to zero.
 */
void svoGNGEuHebbianLearning(svo_gng_eu_t *gng_eu,
                           svo_gng_eu_node_t *n1, svo_gng_eu_node_t *n2);

/**
 * Returns node with highest error counter.
 */
svo_gng_eu_node_t *svoGNGEuNodeWithHighestError(svo_gng_eu_t *gng_eu);

/**
 * Finds out node with highest error counter ({n1}) and its neighbor with
 * highest error counter ({n2}). Into {edge} is stored edge connecting
 * those nodes. {n1}, {n2} and {edge} are ignored if NULL is passed.
 */
void svoGNGEuNodeWithHighestError2(svo_gng_eu_t *gng_eu,
                                 svo_gng_eu_node_t **n1, svo_gng_eu_node_t **n2,
                                 svo_gng_eu_edge_t **edge);

void svoGNGEuDumpSVT(svo_gng_eu_t *gng_eu, FILE *out, const char *name);


/**
 * Net Related API
 * ----------------
 *
 * See svo_gng_eu_node_t.
 * See svo_gng_eu_edge_t.
 */

/**
 * Returns net of nodes.
 */
_bor_inline bor_net_t *svoGNGEuNet(svo_gng_eu_t *gng_eu);

/**
 * Returns list of nodes.
 */
_bor_inline bor_list_t *svoGNGEuNodes(svo_gng_eu_t *gng_eu);

/**
 * Returns number of nodes in net.
 */
_bor_inline size_t svoGNGEuNodesLen(const svo_gng_eu_t *gng_eu);

/**
 * Returns list of edges.
 */
_bor_inline bor_list_t *svoGNGEuNodes(svo_gng_eu_t *gng_eu);

/**
 * Returns number of edges in net.
 */
_bor_inline size_t svoGNGEuEdgesLen(const svo_gng_eu_t *gng_eu);

/**
 * Returns list of nodes
 */
_bor_inline bor_list_t *svoGNGEuEdges(svo_gng_eu_t *gng_eu);

/**
 * Returns GNGEu node from list pointer.
 *
 * Usage:
 * ~~~~~
 * bor_list_t *list, *item;
 * svo_gng_eu_node_t *n;
 *
 * list = svoGNGEuNodes(gng_eu);
 * BOR_LIST_FOR_EACH(list, item){
 *     n = svoGNGEuNodeFromList(item);
 *     ....
 * }
 */
_bor_inline svo_gng_eu_node_t *svoGNGEuNodeFromList(bor_list_t *item);

/**
 * Similar to *svoGNGEuNodeFromList()* but works with nodes.
 */
_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeFromList(bor_list_t *item);

/**
 * Cast Net node to GNGEu node.
 */
_bor_inline svo_gng_eu_node_t *svoGNGEuNodeFromNet(bor_net_node_t *n);

/**
 * Cast Net edge to GNGEu edge.
 */
_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeFromNet(bor_net_edge_t *e);

/**
 * Cast GNGEu node to Net node.
 */
_bor_inline bor_net_node_t *svoGNGEuNodeToNet(svo_gng_eu_node_t *n);

/**
 * Cast GNGEu edge to Net edge.
 */
_bor_inline bor_net_edge_t *svoGNGEuEdgeToNet(svo_gng_eu_edge_t *e);



/**
 * Node API
 * ^^^^^^^^^
 *
 * See svo_gng_eu_node_t.
 */

/**
 * Adds node into network
 */
_bor_inline void svoGNGEuNodeAdd(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                 const bor_vec_t *w);

/**
 * Removes node from network
 */
_bor_inline void svoGNGEuNodeRemove(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n);

/**
 * Removes node from network and deletes it (ops.del_node is used).
 */
_bor_inline void svoGNGEuNodeDel(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n);

/**
 * Fixes node's error counter, i.e. applies correct beta^(n * lambda)
 */
_bor_inline void svoGNGEuNodeFixError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n);

/**
 * Increment error counter
 */
_bor_inline void svoGNGEuNodeIncError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                    bor_real_t inc);
/**
 * Scales error counter
 */
_bor_inline void svoGNGEuNodeScaleError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                      bor_real_t scale);

/**
 * Disconnects node from net, i.e., deletes all incidenting edges.
 */
void svoGNGEuNodeDisconnect(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n);

/**
 * Connects new node at given position (is) and connects it with two
 * nearest nodes [ops.new_node(), ops.nearest()].
 */
svo_gng_eu_node_t *svoGNGEuNodeNewAtPos(svo_gng_eu_t *gng_eu, const void *is);




/**
 * Edge API
 * ^^^^^^^^^
 *
 * See svo_gng_eu_edge_t.
 */

/**
 * Creates and initializes new edge between {n1} and {n2}.
 */
svo_gng_eu_edge_t *svoGNGEuEdgeNew(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n1,
                                              svo_gng_eu_node_t *n2);

/**
 * Deletes edge
 */
void svoGNGEuEdgeDel(svo_gng_eu_t *gng_eu, svo_gng_eu_edge_t *edge);

/**
 * Returns age of edge.
 *
 * Always use this function instead of direct access to struct!
 */
_bor_inline int svoGNGEuEdgeAge(const svo_gng_eu_t *gng_eu, const svo_gng_eu_edge_t *edge);


/**
 * Returns edge connecting {n1} and {n2}.
 */
_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeBetween(svo_gng_eu_t *gng_eu,
                                              svo_gng_eu_node_t *n1,
                                              svo_gng_eu_node_t *n2);

/**
 * Deletes edge between {n1} and {n2}.
 */
void svoGNGEuEdgeBetweenDel(svo_gng_eu_t *gng_eu,
                          svo_gng_eu_node_t *n1, svo_gng_eu_node_t *n2);

/**
 * Returns (via {n1} and {n2}) incidenting nodes of edge
 */
_bor_inline void svoGNGEuEdgeNodes(svo_gng_eu_edge_t *e,
                                 svo_gng_eu_node_t **n1, svo_gng_eu_node_t **n2);





/**** INLINES ****/
_bor_inline bor_net_t *svoGNGEuNet(svo_gng_eu_t *gng_eu)
{
    return gng_eu->net;
}

_bor_inline bor_list_t *svoGNGEuNodes(svo_gng_eu_t *gng_eu)
{
    return borNetNodes(gng_eu->net);
}

_bor_inline size_t svoGNGEuNodesLen(const svo_gng_eu_t *gng_eu)
{
    return borNetNodesLen(gng_eu->net);
}

_bor_inline bor_list_t *svoGNGEuEdges(svo_gng_eu_t *gng_eu)
{
    return borNetEdges(gng_eu->net);
}

_bor_inline size_t svoGNGEuEdgesLen(const svo_gng_eu_t *gng_eu)
{
    return borNetEdgesLen(gng_eu->net);
}

_bor_inline svo_gng_eu_node_t *svoGNGEuNodeFromList(bor_list_t *item)
{
    bor_net_node_t *nn;
    svo_gng_eu_node_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
    n  = bor_container_of(nn, svo_gng_eu_node_t, node);
    return n;
}

_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeFromList(bor_list_t *item)
{
    bor_net_edge_t *nn;
    svo_gng_eu_edge_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_edge_t, list);
    n  = bor_container_of(nn, svo_gng_eu_edge_t, edge);
    return n;
}

_bor_inline svo_gng_eu_node_t *svoGNGEuNodeFromNet(bor_net_node_t *n)
{
    return bor_container_of(n, svo_gng_eu_node_t, node);
}

_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeFromNet(bor_net_edge_t *e)
{
    return bor_container_of(e, svo_gng_eu_edge_t, edge);
}

_bor_inline bor_net_node_t *svoGNGEuNodeToNet(svo_gng_eu_node_t *n)
{
    return &n->node;
}

_bor_inline bor_net_edge_t *svoGNGEuEdgeToNet(svo_gng_eu_edge_t *e)
{
    return &e->edge;
}



_bor_inline void svoGNGEuNodeAdd(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                 const bor_vec_t *w)
{
    n->err       = BOR_ZERO;
    n->err_cycle = gng_eu->cycle;
    borPairHeapAdd(gng_eu->err_heap, &n->err_heap);

    borNetAddNode(gng_eu->net, &n->node);

    if (gng_eu->params.dim == 2){
        n->w = (bor_vec_t *)borVec2Clone((const bor_vec2_t *)w);
    }else if (gng_eu->params.dim == 3){
        n->w = (bor_vec_t *)borVec3Clone((const bor_vec3_t *)w);
    }else{
        n->w = borVecClone(gng_eu->params.dim, (const bor_vec_t *)w);
    }

    if (gng_eu->nn){
        borNNElInit(gng_eu->nn, &n->nn, n->w);
        borNNAdd(gng_eu->nn, &n->nn);
    }
}

_bor_inline void svoGNGEuNodeRemove(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n)
{
    borPairHeapRemove(gng_eu->err_heap, &n->err_heap);

    if (borNetNodeEdgesLen(&n->node) != 0)
        svoGNGEuNodeDisconnect(gng_eu, n);
    borNetRemoveNode(gng_eu->net, &n->node);

    if (gng_eu->nn){
        borNNRemove(gng_eu->nn, &n->nn);
    }

    borVecDel(n->w);
}

_bor_inline void svoGNGEuNodeDel(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n)
{
    svoGNGEuNodeRemove(gng_eu, n);
    if (gng_eu->ops.del_node){
        gng_eu->ops.del_node(n, gng_eu->ops.del_node_data);
    }else{
        BOR_FREE(n);
    }
}

_bor_inline void svoGNGEuNodeFixError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n)
{
    unsigned long diff;

    diff = gng_eu->cycle - n->err_cycle;
    if (diff > 0 && diff <= gng_eu->beta_lambda_n_len){
        n->err *= gng_eu->beta_lambda_n[diff - 1];
    }else if (diff > 0){
        n->err *= gng_eu->beta_lambda_n[gng_eu->beta_lambda_n_len - 1];

        diff = diff - gng_eu->beta_lambda_n_len;
        n->err *= pow(gng_eu->beta_n[gng_eu->beta_lambda_n_len - 1], diff);
    }
    n->err_cycle = gng_eu->cycle;
}

_bor_inline void svoGNGEuNodeIncError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                    bor_real_t inc)
{
    svoGNGEuNodeFixError(gng_eu, n);
    n->err += inc;
    borPairHeapUpdate(gng_eu->err_heap, &n->err_heap);
}

_bor_inline void svoGNGEuNodeScaleError(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n,
                                      bor_real_t scale)
{
    svoGNGEuNodeFixError(gng_eu, n);
    n->err *= scale;
    borPairHeapUpdate(gng_eu->err_heap, &n->err_heap);
}



_bor_inline int svoGNGEuEdgeAge(const svo_gng_eu_t *gng_eu, const svo_gng_eu_edge_t *edge)
{
    return edge->age;
}

_bor_inline svo_gng_eu_edge_t *svoGNGEuEdgeBetween(svo_gng_eu_t *gng_eu,
                                              svo_gng_eu_node_t *n1,
                                              svo_gng_eu_node_t *n2)
{
    bor_net_edge_t *ne;
    svo_gng_eu_edge_t *e = NULL;

    ne = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (ne)
        e  = bor_container_of(ne, svo_gng_eu_edge_t, edge);
    return e;
}

_bor_inline void svoGNGEuEdgeNodes(svo_gng_eu_edge_t *e,
                                 svo_gng_eu_node_t **n1, svo_gng_eu_node_t **n2)
{
    bor_net_node_t *n;

    n   = borNetEdgeNode(&e->edge, 0);
    *n1 = bor_container_of(n, svo_gng_eu_node_t, node);

    n   = borNetEdgeNode(&e->edge, 1);
    *n2 = bor_container_of(n, svo_gng_eu_node_t, node);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GNG_EU_H__ */


