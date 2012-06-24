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

#include <svoboda/gng.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>

/**
 * Here is implemented Growing Neural Gas algorithm  completely in static
 * functions. The purpose is to be able to get rid of some indirect calls
 * of functions (see svo_gng_t.ops struct).
 *
 * Indirect calls can be very expensive, so if you need to speed up
 * an algorithm and you really know what you are doing you can use these
 * functions.
 *
 * All functions marked as "public API" corresponds to same GNGs' functions
 * without initial underscore.
 *
 * Before including this file macros OPS() and OPS_DATA() must be defined
 * because these macros are used to expand to "operation" calls as defined
 * in svo_gng_ops_t struct.
 * Operations that are covered by OPS() and OPS_DATA() macros are:
 *    - new_node
 *    - new_node_between
 *    - input_signal
 *    - nearest
 *    - dist2
 *    - move_towards
 *
 * For example how to use this file look into gng.c.
 */


/** For public API */
static void _svoGNGRun(svo_gng_t *gng);
static void _svoGNGInit(svo_gng_t *gng);
static void _svoGNGLearn(svo_gng_t *gng, size_t step);
static void _svoGNGNewNode(svo_gng_t *gng);
#ifndef NO_CONNECT_NEW_NODE
static svo_gng_node_t *_svoGNGConnectNewNode(svo_gng_t *gng, const void *is);
#endif

/** Node functions */
/** Adds node into network */
_bor_inline void nodeAdd(svo_gng_t *gng, svo_gng_node_t *n);
/** Removes node from network */
_bor_inline void nodeRemove(svo_gng_t *gng, svo_gng_node_t *n);
/** Fixes node's error counter, i.e. applies correct beta^(n * lambda) */
_bor_inline void nodeFixError(svo_gng_t *gng, svo_gng_node_t *n);
/** Increment error counter */
_bor_inline void nodeIncError(svo_gng_t *gng, svo_gng_node_t *n,
                              bor_real_t inc);
/** Scales error counter */
_bor_inline void nodeScaleError(svo_gng_t *gng, svo_gng_node_t *n,
                                bor_real_t scale);

/** Edge functions */
/** Creates and initializes new edge between n1 and n2 */
_bor_inline svo_gng_edge_t *edgeNew(svo_gng_t *gng, svo_gng_node_t *n1,
                                                    svo_gng_node_t *n2);
/** Deletes edge */
_bor_inline void edgeDel(svo_gng_t *gng, svo_gng_edge_t *edge);

/** Returns node with highest error counter */
static svo_gng_node_t *nodeWithHighestErr(svo_gng_t *gng);
/** Returns q's neighbor with highest error counter and edge that connects
 *  that node with q. */
static svo_gng_node_t *nodeWithHighestErr2(svo_gng_t *gng, svo_gng_node_t *q,
                                           svo_gng_edge_t **edge);

/** Should return true if n1 > n2 - this is used for err-heap */
static int errHeapLT(const bor_pairheap_node_t *n1,
                     const bor_pairheap_node_t *n2, void *);

static void _svoGNGRun(svo_gng_t *gng)
{
    unsigned long cycle;

    cycle = 0;
    _svoGNGInit(gng);

    do {
        for (gng->step = 1; gng->step <= gng->params.lambda; gng->step++){
            _svoGNGLearn(gng, gng->step);
        }
        _svoGNGNewNode(gng);

        cycle++;
        if (gng->ops.callback && gng->ops.callback_period == cycle){
            gng->ops.callback(gng->ops.callback_data);
            cycle = 0L;
        }

        gng->cycle++;
    } while (!gng->ops.terminate(gng->ops.terminate_data));
}

static void _svoGNGInit(svo_gng_t *gng)
{
    const void *is;
    svo_gng_node_t *n1 = NULL, *n2 = NULL;
    size_t i;
    bor_real_t maxbeta;

    gng->cycle = 1L;

    // initialize error heap
    if (gng->err_heap)
        borPairHeapDel(gng->err_heap);
    gng->err_heap = borPairHeapNew(errHeapLT, (void *)gng);

    // precompute beta^n
    if (gng->beta_n)
        BOR_FREE(gng->beta_n);
    gng->beta_n = BOR_ALLOC_ARR(bor_real_t, gng->params.lambda);
    gng->beta_n[0] = gng->params.beta;
    for (i = 1; i < gng->params.lambda; i++){
        gng->beta_n[i] = gng->beta_n[i - 1] * gng->params.beta;
    }

    // precompute beta^(n * lambda)
    if (gng->beta_lambda_n)
        BOR_FREE(gng->beta_lambda_n);

    maxbeta = gng->beta_n[gng->params.lambda - 1];

    gng->beta_lambda_n_len = 1000;
    gng->beta_lambda_n = BOR_ALLOC_ARR(bor_real_t, gng->beta_lambda_n_len);
    gng->beta_lambda_n[0] = maxbeta;
    for (i = 1; i < gng->beta_lambda_n_len; i++){
        gng->beta_lambda_n[i] = gng->beta_lambda_n[i - 1] * maxbeta;
    }


    if (gng->ops.init){
        OPS(gng, init)(&n1, &n2, OPS_DATA(gng, init));
    }else{
        is = OPS(gng, input_signal)(OPS_DATA(gng, input_signal));
        n1 = OPS(gng, new_node)(is, OPS_DATA(gng, new_node));

        is = OPS(gng, input_signal)(OPS_DATA(gng, input_signal));
        n2 = OPS(gng, new_node)(is, OPS_DATA(gng, new_node));
    }

    nodeAdd(gng, n1);
    nodeAdd(gng, n2);
    edgeNew(gng, n1, n2);
}

static void _svoGNGLearn(svo_gng_t *gng, size_t step)
{
    const void *input_signal;
    bor_net_node_t *nn;
    svo_gng_node_t *n1, *n2, *n;
    bor_net_edge_t *nedge;
    svo_gng_edge_t *edge;
    bor_real_t dist2;
    bor_list_t *list, *item, *item_tmp;

    // 1. Get input signal
    input_signal = OPS(gng, input_signal)(OPS_DATA(gng, input_signal));

    // 2. Find two nearest nodes to input signal
    OPS(gng, nearest)(input_signal, &n1, &n2, OPS_DATA(gng, nearest));

    // 3. Create connection between n1 and n2 if doesn't exist and set age
    //    to zero
    nedge = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (!nedge){
        edge = edgeNew(gng, n1, n2);
    }else{
        edge = bor_container_of(nedge, svo_gng_edge_t, edge);
    }
    edge->age = 0;

    // 4. Increase error counter of winner node
    dist2 = OPS(gng, dist2)(input_signal, n1, OPS_DATA(gng, dist2));
    nodeIncError(gng, n1, dist2 * gng->beta_n[gng->params.lambda - step]);

    // 5. Adapt nodes to input signal using fractions eb and en
    // + 6. Increment age of all edges by one
    // + 7. Remove edges with age higher than age_max
    OPS(gng, move_towards)(n1, input_signal, gng->params.eb,
                           OPS_DATA(gng, move_towards));
    // adapt also direct topological neighbors of winner node
    list = borNetNodeEdges(&n1->node);
    BOR_LIST_FOR_EACH_SAFE(list, item, item_tmp){
        nedge = borNetEdgeFromNodeList(item);
        edge  = bor_container_of(nedge, svo_gng_edge_t, edge);
        nn   = borNetEdgeOtherNode(&edge->edge, &n1->node);
        n    = bor_container_of(nn, svo_gng_node_t, node);

        // increase age (6.)
        edge->age += 1;

        // remove edge if it has age higher than age_max (7.)
        if (edge->age > gng->params.age_max){
            edgeDel(gng, edge);

            if (borNetNodeEdgesLen(nn) == 0){
                // remove node if not connected into net anymore
                nodeRemove(gng, n);
                n = NULL;
            }
        }

        // move node (5.)
        if (n){
            OPS(gng, move_towards)(n, input_signal, gng->params.en,
                OPS_DATA(gng, move_towards));
        }
    }

    // remove winning node if not connected into net
    if (borNetNodeEdgesLen(&n1->node) == 0){
        // remove node if not connected into net anymore
        nodeRemove(gng, n1);
    }
}

static void _svoGNGNewNode(svo_gng_t *gng)
{
    svo_gng_node_t *q, *f, *r;
    svo_gng_edge_t *eqf;

    // 1. Get node with highest error counter
    q = nodeWithHighestErr(gng);

    // 2. Get q's neighbor with highest error counter
    f = nodeWithHighestErr2(gng, q, &eqf);
    if (!f){
        ERR2("Node with highest error counter doesn't have any neighbors! "
             "This shouldn't happen - something's wrong with algorithm.");
        return;
    }

    // 3. Create new node between q and f
    r = OPS(gng, new_node_between)(q, f, OPS_DATA(gng, new_node_between));
    nodeAdd(gng, r);

    // 4. Create q-r and f-r edges and remove q-f edge (which is eqf)
    edgeDel(gng, eqf);
    edgeNew(gng, q, r);
    edgeNew(gng, f, r);

    // 5. Decrease error counters of q and f
    nodeScaleError(gng, q, gng->params.alpha);
    nodeScaleError(gng, f, gng->params.alpha);

    // 6. Set error counter of new node (r)
    r->err  = q->err + f->err;
    r->err /= BOR_REAL(2.);
    r->err_cycle = gng->cycle;
    borPairHeapUpdate(gng->err_heap, &r->err_heap);
}

#ifndef NO_CONNECT_NEW_NODE
static svo_gng_node_t *_svoGNGConnectNewNode(svo_gng_t *gng, const void *is)
{
    svo_gng_node_t *r, *n1, *n2;
    svo_gng_edge_t *edge;

    OPS(gng, nearest)(is, &n1, &n2, OPS_DATA(gng, nearest));

    r = OPS(gng, new_node)(is, OPS_DATA(gng, new_node));
    nodeAdd(gng, r);

    edge = edgeNew(gng, r, n1);
    edge->age = 0;
    edge = edgeNew(gng, r, n2);
    edge->age = 0;

    return r;
}
#endif


/*** Node functions ***/

_bor_inline void nodeAdd(svo_gng_t *gng, svo_gng_node_t *n)
{
    n->err       = BOR_ZERO;
    n->err_cycle = gng->cycle;
    borPairHeapAdd(gng->err_heap, &n->err_heap);

    borNetAddNode(gng->net, &n->node);
}

_bor_inline void nodeRemove(svo_gng_t *gng, svo_gng_node_t *n)
{
    borPairHeapRemove(gng->err_heap, &n->err_heap);
    borNetRemoveNode(gng->net, &n->node);
    gng->ops.del_node(n, gng->ops.del_node_data);
}

_bor_inline void nodeFixError(svo_gng_t *gng, svo_gng_node_t *n)
{
    unsigned long diff;

    diff = gng->cycle - n->err_cycle;
    if (diff > 0 && diff <= gng->beta_lambda_n_len){
        n->err *= gng->beta_lambda_n[diff - 1];
    }else if (diff > 0){
        n->err *= gng->beta_lambda_n[gng->params.lambda - 1];

        diff = diff - gng->beta_lambda_n_len;
        n->err *= pow(gng->beta_n[gng->params.lambda - 1], diff);
    }
    n->err_cycle = gng->cycle;
}

_bor_inline void nodeIncError(svo_gng_t *gng, svo_gng_node_t *n,
                              bor_real_t inc)
{
    nodeFixError(gng, n);
    n->err += inc;
    borPairHeapUpdate(gng->err_heap, &n->err_heap);
}

_bor_inline void nodeScaleError(svo_gng_t *gng, svo_gng_node_t *n,
                                bor_real_t scale)
{
    nodeFixError(gng, n);
    n->err *= scale;
    borPairHeapUpdate(gng->err_heap, &n->err_heap);
}



/*** Edge functions ***/
_bor_inline svo_gng_edge_t *edgeNew(svo_gng_t *gng, svo_gng_node_t *n1,
                                                    svo_gng_node_t *n2)
{
    svo_gng_edge_t *e;

    e = BOR_ALLOC(svo_gng_edge_t);
    e->age = 0;

    borNetAddEdge(gng->net, &e->edge, &n1->node, &n2->node);

    return e;
}

_bor_inline void edgeDel(svo_gng_t *gng, svo_gng_edge_t *e)
{
    borNetRemoveEdge(gng->net, &e->edge);
    BOR_FREE(e);
}



static svo_gng_node_t *nodeWithHighestErr(svo_gng_t *gng)
{
    bor_pairheap_node_t *max;
    svo_gng_node_t *maxn;

    max  = borPairHeapMin(gng->err_heap);
    maxn = bor_container_of(max, svo_gng_node_t, err_heap);

    return maxn;
}

static svo_gng_node_t *nodeWithHighestErr2(svo_gng_t *gng, svo_gng_node_t *q,
                                           svo_gng_edge_t **edge)
{
    bor_list_t *list, *item;
    bor_net_edge_t *ne;
    svo_gng_edge_t *e_highest;
    bor_net_node_t *nn;
    svo_gng_node_t *n, *n_highest;
    bor_real_t err_highest;

    err_highest = -BOR_ONE;
    n_highest = NULL;
    e_highest = NULL;

    list = borNetNodeEdges(&q->node);
    BOR_LIST_FOR_EACH(list, item){
        ne = borNetEdgeFromNodeList(item);
        nn = borNetEdgeOtherNode(ne, &q->node);
        n  = bor_container_of(nn, svo_gng_node_t, node);

        nodeFixError(gng, n);

        if (n->err > err_highest){
            err_highest = n->err;
            n_highest   = n;
            e_highest   = bor_container_of(ne, svo_gng_edge_t, edge);
        }
    }

    *edge = e_highest;
    return n_highest;
}

static int errHeapLT(const bor_pairheap_node_t *_n1,
                     const bor_pairheap_node_t *_n2,
                     void *data)
{
    svo_gng_t *gng = (svo_gng_t *)data;
    svo_gng_node_t *n1, *n2;

    n1 = bor_container_of(_n1, svo_gng_node_t, err_heap);
    n2 = bor_container_of(_n2, svo_gng_node_t, err_heap);

    nodeFixError(gng, n1);
    nodeFixError(gng, n2);
    return n1->err > n2->err;
}
