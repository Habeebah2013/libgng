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

#ifndef __SVO_GNG_H__
#define __SVO_GNG_H__

#include <boruvka/net.h>
#include <boruvka/pairheap.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Growing Neural Gas
 * ===================
 * Generalized implementation of Growing Neural Gas algorithm as was
 * described in:
 *
 *      B. Fritzke, "A growing neural gas network learns topologies,"
 *      Neural Inf. Process. Syst., vol. 7, pp. 625­632, 1995.
 *
 *
 * This implementation is not limited to 2-D, 3-D or any other dimensions.
 * In fact, nodes doesn't have assigned any weight vector - it is user's
 * responsibility to provide that.
 *
 * User must define several callbacks and fill *svo_gng_ops_t* structure,
 * svoGNG*() functions take care of the core of algorithm.
 *
 *
 * | **Algorithm works as follows:**
 * |
 * | 1. Initializes network by two random nodes. [*svoGNGInit()*]
 * | 2. Check termination condition
 * | 3. Learn topology. ['svoGNGLearn()*]
 * |     1. Get random input signal
 * |     2. Find two nearest nodes to input signal - {n1}, {n2}
 * |     3. Create connection between {n1} and {n2} if doesn't exist and set age
 * |        to zero
 * |     4. Increase error counter of winner node.
 * |     5. Adapt nodes to input signal using fractions {eb} and {en}
 * |     6. Increment age of all edges that incident with winner node by one
 * |     7. Remove all edges with age higher than {age_max}
 * | 4. If the number of input signals presented so far to the network is an
 * |    integer multiple of the parameter {lambda}, create new node. [*svoGNGNewNode()*]
 * |     1. Get node with highest error counter -> {q}
 * |     2. Get {q}'s neighbor node with highest error counter -> {f}
 * |     3. Create new node between {q} and {f} -> {r}
 * |     4. Create {q-r} and {f-r} edges and delete {q-f} edge.
 * |     5. Decrease error counter of {q} and {f} ({alpha} parameter).
 * |     6. Set error counter of {r} as average error counter of {q} and {f}.
 * | 5. Decrease error counters of all nodes [*svoGNGDecreaseErrCounters()*]
 * | 6. Go to 2.
 */


struct _svo_gng_node_t {
    bor_net_node_t node;

    bor_real_t err;               /*!< Overall error */
    unsigned long err_cycle;      /*!< Last cycle in which were .err changed */
    bor_pairheap_node_t err_heap; /*!< Connection to error heap */
};
typedef struct _svo_gng_node_t svo_gng_node_t;


struct _svo_gng_edge_t {
    bor_net_edge_t edge;

    int age;
};
typedef struct _svo_gng_edge_t svo_gng_edge_t;



/**
 * GNG Operations
 * ---------------
 *
 * See svo_gng_ops_t.
 */

/** vvvv */

/**
 * Return two nodes that will be used for network initialization.
 * If not specified (NULL) two random input ignals are use for
 * initialization.
 */
typedef void (*svo_gng_init)(svo_gng_node_t **n1,
                             svo_gng_node_t **n2,
                             void *);

/**
 * Create new node initialized weight vector to input_signal.
 */
typedef svo_gng_node_t *(*svo_gng_new_node)(const void *input_signal, void *);

/**
 * Create new node in between n1 and n2.
 */
typedef svo_gng_node_t *(*svo_gng_new_node_between)(const svo_gng_node_t *n1,
                                                    const svo_gng_node_t *n2,
                                                    void *);

/**
 * Deletes given node.
 */
typedef void (*svo_gng_del_node)(svo_gng_node_t *n, void *);

/**
 * Returns random input signal.
 */
typedef const void *(*svo_gng_input_signal)(void *);

/**
 * Returns (via n1 and n2) first and second nearest node to input signal.
 */
typedef void (*svo_gng_nearest)(const void *input_signal,
                                svo_gng_node_t **n1,
                                svo_gng_node_t **n2,
                                void *);

/**
 * Returns squared distance between input_signal and node.
 */
typedef bor_real_t (*svo_gng_dist2)(const void *input_signal,
                                    const svo_gng_node_t *node, void *);

/**
 * Move given node towards input_signal by given fraction.
 *
 * If position on node is w and position of input_signal is v, then:
 * w = w + ((v - w) * fraction)
 */
typedef void (*svo_gng_move_towards)(svo_gng_node_t *node,
                                     const void *input_signal,
                                     bor_real_t fraction,
                                     void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_gng_terminate)(void *);

/**
 * Callback that is peridically called from GNG.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_gng_callback)(void *);

/** ^^^^ */

struct _svo_gng_ops_t {
    svo_gng_init             init;
    svo_gng_new_node         new_node;
    svo_gng_new_node_between new_node_between;
    svo_gng_del_node         del_node;
    svo_gng_input_signal     input_signal;
    svo_gng_nearest          nearest;
    svo_gng_dist2            dist2;
    svo_gng_move_towards     move_towards;
    svo_gng_terminate        terminate;

    svo_gng_callback callback;
    unsigned long callback_period;

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */

    void *init_data;
    void *new_node_data;
    void *new_node_between_data;
    void *del_node_data;
    void *input_signal_data;
    void *nearest_data;
    void *dist2_data;
    void *move_towards_data;
    void *terminate_data;
    void *callback_data;
};
typedef struct _svo_gng_ops_t svo_gng_ops_t;


/**
 * Initializes ops struct to NULL values.
 */
void svoGNGOpsInit(svo_gng_ops_t *ops);



/**
 * GNG Parameters
 * ---------------
 */
struct _svo_gng_params_t {
    size_t lambda;    /*!< Number of steps between adding nodes */
    bor_real_t eb;    /*!< Winner node learning rate */
    bor_real_t en;    /*!< Winners' neighbors learning rate */
    bor_real_t alpha; /*!< Decrease error counter rate */
    bor_real_t beta;  /*!< Decrease error counter rate for all nodes */
    int age_max;      /*!< Maximal age of edge */
};
typedef struct _svo_gng_params_t svo_gng_params_t;

/**
 * Initializes params struct to default values.
 */
void svoGNGParamsInit(svo_gng_params_t *params);



/**
 * GNG Algorithm
 * --------------
 *
 * See svo_gng_t.
 */

struct _svo_gng_t {
    bor_net_t *net;
    bor_pairheap_t *err_heap;

    svo_gng_ops_t ops;
    svo_gng_params_t params;

    bor_real_t *beta_n; /*!< Precomputed beta^n for n = 1, ..., lambda */
    bor_real_t *beta_lambda_n; /*!< Precomputed beta^(n*lambda) */
    size_t beta_lambda_n_len;

    size_t step;
    unsigned long cycle;
};
typedef struct _svo_gng_t svo_gng_t;


/**
 * Creates new instance of GNG algorithm.
 */
svo_gng_t *svoGNGNew(const svo_gng_ops_t *ops,
                     const svo_gng_params_t *params);

/**
 * Deletes GNG.
 */
void svoGNGDel(svo_gng_t *gng);

/**
 * Runs GNG algorithm.
 *
 * This runs whole algorithm in loop until operation terminate() returns
 * true:
 * ~~~~~~
 * svoGNGinit()
 * do:
 *     for (step = 1 .. params.lambda):
 *         svoGNGLearn()
 *     svoGNGNewNode()
 * while not ops.terminate()
 */
void svoGNGRun(svo_gng_t *gng);


/**
 * Initialize gng net.
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
void svoGNGInit(svo_gng_t *gng);

/**
 * One competitive hebbian learning step.
 */
void svoGNGLearn(svo_gng_t *gng);

/**
 * Creates new node in place with highest error counter.
 */
void svoGNGNewNode(svo_gng_t *gng);

/**
 * Performs hebbian learning between two given nodes - connection
 * between these two nodes is strengthened, i.e., edge is eigher created or
 * age of the existing edge is set to zero.
 */
void svoGNGHebbianLearning(svo_gng_t *gng,
                           svo_gng_node_t *n1, svo_gng_node_t *n2);

/**
 * Returns node with highest error counter.
 */
svo_gng_node_t *svoGNGNodeWithHighestError(svo_gng_t *gng);

/**
 * Finds out node with highest error counter ({n1}) and its neighbor with
 * highest error counter ({n2}). Into {edge} is stored edge connecting
 * those nodes. {n1}, {n2} and {edge} are ignored if NULL is passed.
 */
void svoGNGNodeWithHighestError2(svo_gng_t *gng,
                                 svo_gng_node_t **n1, svo_gng_node_t **n2,
                                 svo_gng_edge_t **edge);



/**
 * Net Related API
 * ----------------
 *
 * See svo_gng_node_t.
 * See svo_gng_edge_t.
 */

/**
 * Returns net of nodes.
 */
_bor_inline bor_net_t *svoGNGNet(svo_gng_t *gng);

/**
 * Returns list of nodes.
 */
_bor_inline bor_list_t *svoGNGNodes(svo_gng_t *gng);

/**
 * Returns number of nodes in net.
 */
_bor_inline size_t svoGNGNodesLen(const svo_gng_t *gng);

/**
 * Returns list of edges.
 */
_bor_inline bor_list_t *svoGNGNodes(svo_gng_t *gng);

/**
 * Returns number of edges in net.
 */
_bor_inline size_t svoGNGEdgesLen(const svo_gng_t *gng);

/**
 * Returns list of nodes
 */
_bor_inline bor_list_t *svoGNGEdges(svo_gng_t *gng);

/**
 * Returns GNG node from list pointer.
 *
 * Usage:
 * ~~~~~
 * bor_list_t *list, *item;
 * svo_gng_node_t *n;
 *
 * list = svoGNGNodes(gng);
 * BOR_LIST_FOR_EACH(list, item){
 *     n = svoGNGNodeFromList(item);
 *     ....
 * }
 */
_bor_inline svo_gng_node_t *svoGNGNodeFromList(bor_list_t *item);

/**
 * Similar to *svoGNGNodeFromList()* but works with nodes.
 */
_bor_inline svo_gng_edge_t *svoGNGEdgeFromList(bor_list_t *item);

/**
 * Cast Net node to GNG node.
 */
_bor_inline svo_gng_node_t *svoGNGNodeFromNet(bor_net_node_t *n);

/**
 * Cast Net edge to GNG edge.
 */
_bor_inline svo_gng_edge_t *svoGNGEdgeFromNet(bor_net_edge_t *e);

/**
 * Cast GNG node to Net node.
 */
_bor_inline bor_net_node_t *svoGNGNodeToNet(svo_gng_node_t *n);

/**
 * Cast GNG edge to Net edge.
 */
_bor_inline bor_net_edge_t *svoGNGEdgeToNet(svo_gng_edge_t *e);



/**
 * Node API
 * ^^^^^^^^^
 *
 * See svo_gng_node_t.
 */

/**
 * Adds node into network
 */
_bor_inline void svoGNGNodeAdd(svo_gng_t *gng, svo_gng_node_t *n);

/**
 * Removes node from network
 */
_bor_inline void svoGNGNodeRemove(svo_gng_t *gng, svo_gng_node_t *n);

/**
 * Removes node from network and deletes it (ops.del_node is used).
 */
_bor_inline void svoGNGNodeDel(svo_gng_t *gng, svo_gng_node_t *n);

/**
 * Fixes node's error counter, i.e. applies correct beta^(n * lambda)
 */
_bor_inline void svoGNGNodeFixError(svo_gng_t *gng, svo_gng_node_t *n);

/**
 * Increment error counter
 */
_bor_inline void svoGNGNodeIncError(svo_gng_t *gng, svo_gng_node_t *n,
                                    bor_real_t inc);
/**
 * Scales error counter
 */
_bor_inline void svoGNGNodeScaleError(svo_gng_t *gng, svo_gng_node_t *n,
                                      bor_real_t scale);

/**
 * Disconnects node from net, i.e., deletes all incidenting edges.
 */
void svoGNGNodeDisconnect(svo_gng_t *gng, svo_gng_node_t *n);

/**
 * Connects new node at given position (is) and connects it with two
 * nearest nodes [ops.new_node(), ops.nearest()].
 */
svo_gng_node_t *svoGNGNodeNewAtPos(svo_gng_t *gng, const void *is);




/**
 * Edge API
 * ^^^^^^^^^
 *
 * See svo_gng_edge_t.
 */

/**
 * Creates and initializes new edge between {n1} and {n2}.
 */
svo_gng_edge_t *svoGNGEdgeNew(svo_gng_t *gng, svo_gng_node_t *n1,
                                              svo_gng_node_t *n2);

/**
 * Deletes edge
 */
void svoGNGEdgeDel(svo_gng_t *gng, svo_gng_edge_t *edge);

/**
 * Returns age of edge.
 *
 * Always use this function instead of direct access to struct!
 */
_bor_inline int svoGNGEdgeAge(const svo_gng_t *gng, const svo_gng_edge_t *edge);


/**
 * Returns edge connecting {n1} and {n2}.
 */
_bor_inline svo_gng_edge_t *svoGNGEdgeBetween(svo_gng_t *gng,
                                              svo_gng_node_t *n1,
                                              svo_gng_node_t *n2);

/**
 * Deletes edge between {n1} and {n2}.
 */
void svoGNGEdgeBetweenDel(svo_gng_t *gng,
                          svo_gng_node_t *n1, svo_gng_node_t *n2);

/**
 * Returns (via {n1} and {n2}) incidenting nodes of edge
 */
_bor_inline void svoGNGEdgeNodes(svo_gng_edge_t *e,
                                 svo_gng_node_t **n1, svo_gng_node_t **n2);





/**** INLINES ****/
_bor_inline bor_net_t *svoGNGNet(svo_gng_t *gng)
{
    return gng->net;
}

_bor_inline bor_list_t *svoGNGNodes(svo_gng_t *gng)
{
    return borNetNodes(gng->net);
}

_bor_inline size_t svoGNGNodesLen(const svo_gng_t *gng)
{
    return borNetNodesLen(gng->net);
}

_bor_inline bor_list_t *svoGNGEdges(svo_gng_t *gng)
{
    return borNetEdges(gng->net);
}

_bor_inline size_t svoGNGEdgesLen(const svo_gng_t *gng)
{
    return borNetEdgesLen(gng->net);
}

_bor_inline svo_gng_node_t *svoGNGNodeFromList(bor_list_t *item)
{
    bor_net_node_t *nn;
    svo_gng_node_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
    n  = bor_container_of(nn, svo_gng_node_t, node);
    return n;
}

_bor_inline svo_gng_edge_t *svoGNGEdgeFromList(bor_list_t *item)
{
    bor_net_edge_t *nn;
    svo_gng_edge_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_edge_t, list);
    n  = bor_container_of(nn, svo_gng_edge_t, edge);
    return n;
}

_bor_inline svo_gng_node_t *svoGNGNodeFromNet(bor_net_node_t *n)
{
    return bor_container_of(n, svo_gng_node_t, node);
}

_bor_inline svo_gng_edge_t *svoGNGEdgeFromNet(bor_net_edge_t *e)
{
    return bor_container_of(e, svo_gng_edge_t, edge);
}

_bor_inline bor_net_node_t *svoGNGNodeToNet(svo_gng_node_t *n)
{
    return &n->node;
}

_bor_inline bor_net_edge_t *svoGNGEdgeToNet(svo_gng_edge_t *e)
{
    return &e->edge;
}



_bor_inline void svoGNGNodeAdd(svo_gng_t *gng, svo_gng_node_t *n)
{
    n->err       = BOR_ZERO;
    n->err_cycle = gng->cycle;
    borPairHeapAdd(gng->err_heap, &n->err_heap);

    borNetAddNode(gng->net, &n->node);
}

_bor_inline void svoGNGNodeRemove(svo_gng_t *gng, svo_gng_node_t *n)
{
    borPairHeapRemove(gng->err_heap, &n->err_heap);

    if (borNetNodeEdgesLen(&n->node) != 0)
        svoGNGNodeDisconnect(gng, n);
    borNetRemoveNode(gng->net, &n->node);
}

_bor_inline void svoGNGNodeDel(svo_gng_t *gng, svo_gng_node_t *n)
{
    svoGNGNodeRemove(gng, n);
    gng->ops.del_node(n, gng->ops.del_node_data);
}

_bor_inline void svoGNGNodeFixError(svo_gng_t *gng, svo_gng_node_t *n)
{
    unsigned long diff;

    diff = gng->cycle - n->err_cycle;
    if (diff > 0 && diff <= gng->beta_lambda_n_len){
        n->err *= gng->beta_lambda_n[diff - 1];
    }else if (diff > 0){
        n->err *= gng->beta_lambda_n[gng->beta_lambda_n_len - 1];

        diff = diff - gng->beta_lambda_n_len;
        n->err *= pow(gng->beta_n[gng->beta_lambda_n_len - 1], diff);
    }
    n->err_cycle = gng->cycle;
}

_bor_inline void svoGNGNodeIncError(svo_gng_t *gng, svo_gng_node_t *n,
                                    bor_real_t inc)
{
    svoGNGNodeFixError(gng, n);
    n->err += inc;
    borPairHeapUpdate(gng->err_heap, &n->err_heap);
}

_bor_inline void svoGNGNodeScaleError(svo_gng_t *gng, svo_gng_node_t *n,
                                      bor_real_t scale)
{
    svoGNGNodeFixError(gng, n);
    n->err *= scale;
    borPairHeapUpdate(gng->err_heap, &n->err_heap);
}



_bor_inline int svoGNGEdgeAge(const svo_gng_t *gng, const svo_gng_edge_t *edge)
{
    return edge->age;
}

_bor_inline svo_gng_edge_t *svoGNGEdgeBetween(svo_gng_t *gng,
                                              svo_gng_node_t *n1,
                                              svo_gng_node_t *n2)
{
    bor_net_edge_t *ne;
    svo_gng_edge_t *e = NULL;

    ne = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (ne)
        e  = bor_container_of(ne, svo_gng_edge_t, edge);
    return e;
}

_bor_inline void svoGNGEdgeNodes(svo_gng_edge_t *e,
                                 svo_gng_node_t **n1, svo_gng_node_t **n2)
{
    bor_net_node_t *n;

    n   = borNetEdgeNode(&e->edge, 0);
    *n1 = bor_container_of(n, svo_gng_node_t, node);

    n   = borNetEdgeNode(&e->edge, 1);
    *n2 = bor_container_of(n, svo_gng_node_t, node);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GNG_H__ */

