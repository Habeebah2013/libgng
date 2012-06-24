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

#ifndef __SVO_GNG_T_H__
#define __SVO_GNG_T_H__

#include <boruvka/net.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Growing Neural Gas with Targeting (GNG-T)
 * ==========================================
 *
 * [1] H. Frezza-Buet, Following non-stationary distributions by
 *     controlling the vector quantization accuracy of a growing neural gas
 *     network, Neurocomputing 71 (7-9) (2008) 1191-1202.
 * 
 */


struct _svo_gngt_node_t {
    bor_net_node_t node;

    bor_real_t err; /*!< Overall error */
    int won;        /*!< True if node has won in last epoch */
};
typedef struct _svo_gngt_node_t svo_gngt_node_t;


struct _svo_gngt_edge_t {
    bor_net_edge_t edge;

    int age;
};
typedef struct _svo_gngt_edge_t svo_gngt_edge_t;



/**
 * GNG-T Operations
 * ---------------
 *
 * See svo_gngt_ops_t.
 */

/** vvvv */

/**
 * Return two nodes that will be used for network initialization.
 * If not specified (NULL) two random input signals are used for
 * initialization.
 */
typedef void (*svo_gngt_init)(svo_gngt_node_t **n1,
                              svo_gngt_node_t **n2,
                              void *);

/**
 * Create new node initialized with weight vector to input_signal.
 */
typedef svo_gngt_node_t *(*svo_gngt_new_node)(const void *input_signal, void *);

/**
 * Create a new node in between n1 and n2.
 */
typedef svo_gngt_node_t *(*svo_gngt_new_node_between)(const svo_gngt_node_t *n1,
                                                      const svo_gngt_node_t *n2,
                                                      void *);

/**
 * Deletes given node.
 */
typedef void (*svo_gngt_del_node)(svo_gngt_node_t *n, void *);

/**
 * Returns random input signal.
 */
typedef const void *(*svo_gngt_input_signal)(void *);

/**
 * Returns (via n1 and n2) first and second nearest node to input signal.
 */
typedef void (*svo_gngt_nearest)(const void *input_signal,
                                 svo_gngt_node_t **n1,
                                 svo_gngt_node_t **n2,
                                 void *);

/**
 * Returns squared distance between input_signal and node.
 */
typedef bor_real_t (*svo_gngt_dist2)(const void *input_signal,
                                     const svo_gngt_node_t *node, void *);

/**
 * Move given node towards input_signal by given fraction.
 *
 * If position on node is w and position of input_signal is v, then:
 * w = w + ((v - w) * fraction)
 */
typedef void (*svo_gngt_move_towards)(svo_gngt_node_t *node,
                                      const void *input_signal,
                                      bor_real_t fraction,
                                      void *);

/**
 * Returns true if algorithm should terminate.
 * This is called at the end of each epoch.
 */
typedef int (*svo_gngt_terminate)(void *);

/**
 * Callback that is peridically called from GNG-T.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_gngt_callback)(void *);

/** ^^^^ */

struct _svo_gngt_ops_t {
    svo_gngt_init             init;
    svo_gngt_new_node         new_node;
    svo_gngt_new_node_between new_node_between;
    svo_gngt_del_node         del_node;
    svo_gngt_input_signal     input_signal;
    svo_gngt_nearest          nearest;
    svo_gngt_dist2            dist2;
    svo_gngt_move_towards     move_towards;
    svo_gngt_terminate        terminate;

    svo_gngt_callback callback;
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
typedef struct _svo_gngt_ops_t svo_gngt_ops_t;


/**
 * Initializes ops struct to NULL values.
 */
void svoGNGTOpsInit(svo_gngt_ops_t *ops);



/**
 * GNG-T Parameters
 * -----------------
 */
struct _svo_gngt_params_t {
    size_t lambda;     /*!< Number of adaptation steps. Default: 200 */
    bor_real_t eb;     /*!< Winner node learning rate. Default: 0.05 */
    bor_real_t en;     /*!< Winners' neighbors learning rate. Default: 0.0006 */
    int age_max;       /*!< Maximal age of edge. Default: 200 */
    bor_real_t target; /*!< Target average error. Default: 100 */
};
typedef struct _svo_gngt_params_t svo_gngt_params_t;

/**
 * Initializes params struct to default values.
 */
void svoGNGTParamsInit(svo_gngt_params_t *params);



/**
 * GNG-T Algorithm
 * ----------------
 *
 * See svo_gngt_t.
 */

struct _svo_gngt_t {
    bor_net_t *net;
    svo_gngt_ops_t ops;
    svo_gngt_params_t params;

    bor_real_t avg_err; /*!< Last computed average error */
};
typedef struct _svo_gngt_t svo_gngt_t;


/**
 * Creates new instance of GNG algorithm.
 */
svo_gngt_t *svoGNGTNew(const svo_gngt_ops_t *ops,
                       const svo_gngt_params_t *params);

/**
 * Deletes GNG.
 */
void svoGNGTDel(svo_gngt_t *gng);

/**
 * Runs GNG algorithm.
 *
 * This runs whole algorithm in loop until operation terminate() returns
 * true:
 * ~~~~~~
 * svoGNGTInit()
 * do:
 *     svoGNGTReset()
 *     for (step = 1 .. params.lambda):
 *         svoGNGTAdapt()
 *     svoGNGTGrowShrink()
 * while not ops.terminate()
 */
void svoGNGTRun(svo_gngt_t *gng);


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
void svoGNGTInit(svo_gngt_t *gng);

/**
 * Resets errors of all nodes to zero.
 */
void svoGNGTReset(svo_gngt_t *gng);

/**
 * One competitive hebbian learning step.
 *
 * ~~~~
 * is = ops.input_signal()
 * n, m = ops.nearest()
 * refresh edge between n and m
 * increment age of all edges emanating from n by one
 * remove all edges with age > age_max
 * update n's error: e = e + ops.dist2(n, is)
 * update weights of nodes:
 *      wn = wn + eb * (is - wn)
 *      wi = wi + en * (is - wi) for neighbors of n
 */
void svoGNGTAdapt(svo_gngt_t *gng);

/**
 * Compares target error with average error and creates or deletes a node
 * according to it.
 */
void svoGNGTGrowShrink(svo_gngt_t *gng);

/**
 * Returns last computed average error
 */
_bor_inline bor_real_t svoGNGTAvgErr(const svo_gngt_t *gng);


/**
 * Net Related API
 * ----------------
 *
 * See svo_gngt_node_t.
 * See svo_gngt_edge_t.
 */

/**
 * Returns net of nodes.
 */
_bor_inline bor_net_t *svoGNGTNet(svo_gngt_t *gng);

/**
 * Returns list of nodes.
 */
_bor_inline bor_list_t *svoGNGTNodes(svo_gngt_t *gng);

/**
 * Returns number of nodes in net.
 */
_bor_inline size_t svoGNGTNodesLen(const svo_gngt_t *gng);

/**
 * Returns list of edges.
 */
_bor_inline bor_list_t *svoGNGTNodes(svo_gngt_t *gng);

/**
 * Returns number of edges in net.
 */
_bor_inline size_t svoGNGTEdgesLen(const svo_gngt_t *gng);

/**
 * Returns list of nodes
 */
_bor_inline bor_list_t *svoGNGTEdges(svo_gngt_t *gng);

/**
 * Returns GNG node from list pointer.
 *
 * Usage:
 * ~~~~~
 * bor_list_t *list, *item;
 * svo_gngt_node_t *n;
 *
 * list = svoGNGTNodes(gng);
 * BOR_LIST_FOR_EACH(list, item){
 *     n = svoGNGTNodeFromList(item);
 *     ....
 * }
 */
_bor_inline svo_gngt_node_t *svoGNGTNodeFromList(bor_list_t *item);

/**
 * Similar to *svoGNGTNodeFromList()* but works with nodes.
 */
_bor_inline svo_gngt_edge_t *svoGNGTEdgeFromList(bor_list_t *item);

/**
 * Cast Net node to GNG node.
 */
_bor_inline svo_gngt_node_t *svoGNGTNodeFromNet(bor_net_node_t *n);

/**
 * Cast Net edge to GNG edge.
 */
_bor_inline svo_gngt_edge_t *svoGNGTEdgeFromNet(bor_net_edge_t *e);

/**
 * Cast GNG node to Net node.
 */
_bor_inline bor_net_node_t *svoGNGTNodeToNet(svo_gngt_node_t *n);

/**
 * Cast GNG edge to Net edge.
 */
_bor_inline bor_net_edge_t *svoGNGTEdgeToNet(svo_gngt_edge_t *e);



/**
 * Node API
 * ^^^^^^^^^
 *
 * See svo_gngt_node_t.
 */

/**
 * Adds node into network
 */
_bor_inline void svoGNGTNodeAdd(svo_gngt_t *gng, svo_gngt_node_t *n);

/**
 * Removes node from network
 */
_bor_inline void svoGNGTNodeRemove(svo_gngt_t *gng, svo_gngt_node_t *n);

/**
 * Removes node from network and deletes it (ops.del_node is used).
 */
_bor_inline void svoGNGTNodeDel(svo_gngt_t *gng, svo_gngt_node_t *n);

/**
 * Disconnects node from net, i.e., deletes all incidenting edges.
 */
void svoGNGTNodeDisconnect(svo_gngt_t *gng, svo_gngt_node_t *n);

/**
 * Connects new node at given position (is) and connects it with two
 * nearest nodes [ops.new_node(), ops.nearest()].
 */
svo_gngt_node_t *svoGNGTNodeNewAtPos(svo_gngt_t *gng, const void *is);




/**
 * Edge API
 * ^^^^^^^^^
 *
 * See svo_gngt_edge_t.
 */

/**
 * Creates and initializes new edge between {n1} and {n2}.
 */
svo_gngt_edge_t *svoGNGTEdgeNew(svo_gngt_t *gng, svo_gngt_node_t *n1,
                                                 svo_gngt_node_t *n2);

/**
 * Deletes edge
 */
void svoGNGTEdgeDel(svo_gngt_t *gng, svo_gngt_edge_t *edge);

/**
 * Returns age of edge.
 *
 * Always use this function instead of direct access to struct!
 */
_bor_inline int svoGNGTEdgeAge(const svo_gngt_t *gng, const svo_gngt_edge_t *edge);


/**
 * Returns edge connecting {n1} and {n2}.
 */
_bor_inline svo_gngt_edge_t *svoGNGTEdgeBetween(svo_gngt_t *gng,
                                                svo_gngt_node_t *n1,
                                                svo_gngt_node_t *n2);

/**
 * Deletes edge between {n1} and {n2}.
 */
void svoGNGTEdgeBetweenDel(svo_gngt_t *gng,
                           svo_gngt_node_t *n1, svo_gngt_node_t *n2);

/**
 * Returns (via {n1} and {n2}) incidenting nodes of edge
 */
_bor_inline void svoGNGTEdgeNodes(svo_gngt_edge_t *e,
                                  svo_gngt_node_t **n1, svo_gngt_node_t **n2);





/**** INLINES ****/
_bor_inline bor_real_t svoGNGTAvgErr(const svo_gngt_t *gng)
{
    return gng->avg_err;
}

_bor_inline bor_net_t *svoGNGTNet(svo_gngt_t *gng)
{
    return gng->net;
}

_bor_inline bor_list_t *svoGNGTNodes(svo_gngt_t *gng)
{
    return borNetNodes(gng->net);
}

_bor_inline size_t svoGNGTNodesLen(const svo_gngt_t *gng)
{
    return borNetNodesLen(gng->net);
}

_bor_inline bor_list_t *svoGNGTEdges(svo_gngt_t *gng)
{
    return borNetEdges(gng->net);
}

_bor_inline size_t svoGNGTEdgesLen(const svo_gngt_t *gng)
{
    return borNetEdgesLen(gng->net);
}

_bor_inline svo_gngt_node_t *svoGNGTNodeFromList(bor_list_t *item)
{
    bor_net_node_t *nn;
    svo_gngt_node_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
    n  = bor_container_of(nn, svo_gngt_node_t, node);
    return n;
}

_bor_inline svo_gngt_edge_t *svoGNGTEdgeFromList(bor_list_t *item)
{
    bor_net_edge_t *nn;
    svo_gngt_edge_t *n;

    nn = BOR_LIST_ENTRY(item, bor_net_edge_t, list);
    n  = bor_container_of(nn, svo_gngt_edge_t, edge);
    return n;
}

_bor_inline svo_gngt_node_t *svoGNGTNodeFromNet(bor_net_node_t *n)
{
    return bor_container_of(n, svo_gngt_node_t, node);
}

_bor_inline svo_gngt_edge_t *svoGNGTEdgeFromNet(bor_net_edge_t *e)
{
    return bor_container_of(e, svo_gngt_edge_t, edge);
}

_bor_inline bor_net_node_t *svoGNGTNodeToNet(svo_gngt_node_t *n)
{
    return &n->node;
}

_bor_inline bor_net_edge_t *svoGNGTEdgeToNet(svo_gngt_edge_t *e)
{
    return &e->edge;
}



_bor_inline void svoGNGTNodeAdd(svo_gngt_t *gng, svo_gngt_node_t *n)
{
    n->err = BOR_ZERO;
    n->won = 0;
    borNetAddNode(gng->net, &n->node);
}

_bor_inline void svoGNGTNodeRemove(svo_gngt_t *gng, svo_gngt_node_t *n)
{
    if (borNetNodeEdgesLen(&n->node) != 0)
        svoGNGTNodeDisconnect(gng, n);
    borNetRemoveNode(gng->net, &n->node);
}

_bor_inline void svoGNGTNodeDel(svo_gngt_t *gng, svo_gngt_node_t *n)
{
    svoGNGTNodeRemove(gng, n);
    gng->ops.del_node(n, gng->ops.del_node_data);
}


_bor_inline int svoGNGTEdgeAge(const svo_gngt_t *gng, const svo_gngt_edge_t *edge)
{
    return edge->age;
}

_bor_inline svo_gngt_edge_t *svoGNGTEdgeBetween(svo_gngt_t *gng,
                                              svo_gngt_node_t *n1,
                                              svo_gngt_node_t *n2)
{
    bor_net_edge_t *ne;
    svo_gngt_edge_t *e = NULL;

    ne = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (ne)
        e  = bor_container_of(ne, svo_gngt_edge_t, edge);
    return e;
}

_bor_inline void svoGNGTEdgeNodes(svo_gngt_edge_t *e,
                                 svo_gngt_node_t **n1, svo_gngt_node_t **n2)
{
    bor_net_node_t *n;

    n   = borNetEdgeNode(&e->edge, 0);
    *n1 = bor_container_of(n, svo_gngt_node_t, node);

    n   = borNetEdgeNode(&e->edge, 1);
    *n2 = bor_container_of(n, svo_gngt_node_t, node);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GNG_T_H__ */


