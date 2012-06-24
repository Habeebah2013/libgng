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


/** Should return true if n1 > n2 - this is used for err-heap */
static int errHeapLT(const bor_pairheap_node_t *n1,
                     const bor_pairheap_node_t *n2, void *);


/** Delete callbacks */
static void nodeFinalDel(bor_net_node_t *node, void *data);
static void delEdge(bor_net_edge_t *edge, void *data);

void svoGNGOpsInit(svo_gng_ops_t *ops)
{
    bzero(ops, sizeof(svo_gng_ops_t));
}

void svoGNGParamsInit(svo_gng_params_t *params)
{
    params->lambda  = 200;
    params->eb      = 0.05;
    params->en      = 0.0006;
    params->alpha   = 0.95;
    params->beta    = 0.9995;
    params->age_max = 200;
}


svo_gng_t *svoGNGNew(const svo_gng_ops_t *ops,
                     const svo_gng_params_t *params)
{
    svo_gng_t *gng;
    size_t i;
    bor_real_t maxbeta;

    gng = BOR_ALLOC(svo_gng_t);

    gng->net = borNetNew();

    gng->ops    = *ops;
    gng->params = *params;

    // set up ops data pointers
    if (!gng->ops.init_data)
        gng->ops.init_data = gng->ops.data;
    if (!gng->ops.new_node_data)
        gng->ops.new_node_data = gng->ops.data;
    if (!gng->ops.new_node_between_data)
        gng->ops.new_node_between_data = gng->ops.data;
    if (!gng->ops.del_node_data)
        gng->ops.del_node_data = gng->ops.data;
    if (!gng->ops.input_signal_data)
        gng->ops.input_signal_data = gng->ops.data;
    if (!gng->ops.nearest_data)
        gng->ops.nearest_data = gng->ops.data;
    if (!gng->ops.dist2_data)
        gng->ops.dist2_data = gng->ops.data;
    if (!gng->ops.move_towards_data)
        gng->ops.move_towards_data = gng->ops.data;
    if (!gng->ops.terminate_data)
        gng->ops.terminate_data = gng->ops.data;
    if (!gng->ops.callback_data)
        gng->ops.callback_data = gng->ops.data;


    // initialize error heap
    gng->err_heap = borPairHeapNew(errHeapLT, (void *)gng);

    // precompute beta^n
    gng->beta_n = BOR_ALLOC_ARR(bor_real_t, gng->params.lambda);
    gng->beta_n[0] = gng->params.beta;
    for (i = 1; i < gng->params.lambda; i++){
        gng->beta_n[i] = gng->beta_n[i - 1] * gng->params.beta;
    }

    // precompute beta^(n * lambda)
    maxbeta = gng->beta_n[gng->params.lambda - 1];
    gng->beta_lambda_n_len = 1000;
    gng->beta_lambda_n = BOR_ALLOC_ARR(bor_real_t, gng->beta_lambda_n_len);
    gng->beta_lambda_n[0] = maxbeta;
    for (i = 1; i < gng->beta_lambda_n_len; i++){
        gng->beta_lambda_n[i] = gng->beta_lambda_n[i - 1] * maxbeta;
    }

    gng->cycle = 1L;
    gng->step  = 1;

    return gng;
}

void svoGNGDel(svo_gng_t *gng)
{
    if (gng->beta_n)
        BOR_FREE(gng->beta_n);
    if (gng->beta_lambda_n)
        BOR_FREE(gng->beta_lambda_n);

    if (gng->net){
        borNetDel2(gng->net, nodeFinalDel, gng,
                              delEdge, gng);
    }

    if (gng->err_heap)
        borPairHeapDel(gng->err_heap);

    BOR_FREE(gng);
}


void svoGNGRun(svo_gng_t *gng)
{
    unsigned long cycle;
    size_t i;

    cycle = 0;
    svoGNGInit(gng);

    do {
        for (i = 0; i < gng->params.lambda; i++){
            svoGNGLearn(gng);
        }
        svoGNGNewNode(gng);

        cycle++;
        if (gng->ops.callback && gng->ops.callback_period == cycle){
            gng->ops.callback(gng->ops.callback_data);
            cycle = 0L;
        }
    } while (!gng->ops.terminate(gng->ops.terminate_data));
}

void svoGNGInit(svo_gng_t *gng)
{
    const void *is;
    svo_gng_node_t *n1 = NULL, *n2 = NULL;

    gng->cycle = 1L;
    gng->step  = 1;

    if (gng->ops.init){
        gng->ops.init(&n1, &n2, gng->ops.init_data);
    }else{
        is = gng->ops.input_signal(gng->ops.input_signal_data);
        n1 = gng->ops.new_node(is, gng->ops.new_node_data);

        is = gng->ops.input_signal(gng->ops.input_signal_data);
        n2 = gng->ops.new_node(is, gng->ops.new_node_data);
    }

    svoGNGNodeAdd(gng, n1);
    svoGNGNodeAdd(gng, n2);
    svoGNGEdgeNew(gng, n1, n2);
}

void svoGNGLearn(svo_gng_t *gng)
{
    const void *input_signal;
    bor_net_node_t *nn;
    svo_gng_node_t *n1, *n2, *n;
    bor_net_edge_t *nedge;
    svo_gng_edge_t *edge;
    bor_real_t dist2;
    bor_list_t *list, *item, *item_tmp;

    if (gng->step > gng->params.lambda){
        gng->cycle += 1L;
        gng->step = 1;
    }

    // 1. Get input signal
    input_signal = gng->ops.input_signal(gng->ops.input_signal_data);

    // 2. Find two nearest nodes to input signal
    gng->ops.nearest(input_signal, &n1, &n2, gng->ops.nearest_data);

    // 3. Create connection between n1 and n2 if doesn't exist and set age
    //    to zero
    svoGNGHebbianLearning(gng, n1, n2);

    // 4. Increase error counter of winner node
    dist2 = gng->ops.dist2(input_signal, n1, gng->ops.dist2_data);
    svoGNGNodeIncError(gng, n1, dist2 * gng->beta_n[gng->params.lambda - gng->step]);

    // 5. Adapt nodes to input signal using fractions eb and en
    // + 6. Increment age of all edges by one
    // + 7. Remove edges with age higher than age_max
    gng->ops.move_towards(n1, input_signal, gng->params.eb,
                           gng->ops.move_towards_data);
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
            svoGNGEdgeDel(gng, edge);

            if (borNetNodeEdgesLen(nn) == 0){
                // remove node if not connected into net anymore
                svoGNGNodeDel(gng, n);
                n = NULL;
            }
        }

        // move node (5.)
        if (n){
            gng->ops.move_towards(n, input_signal, gng->params.en,
                gng->ops.move_towards_data);
        }
    }

    // remove winning node if not connected into net
    if (borNetNodeEdgesLen(&n1->node) == 0){
        // remove node if not connected into net anymore
        svoGNGNodeDel(gng, n1);
    }

    ++gng->step;
}

void svoGNGNewNode(svo_gng_t *gng)
{
    svo_gng_node_t *q, *f, *r;
    svo_gng_edge_t *eqf;

    // 1. Get node with highest error counter and its neighbor with
    // highest error counter
    svoGNGNodeWithHighestError2(gng, &q, &f, &eqf);

    // 3. Create new node between q and f
    r = gng->ops.new_node_between(q, f, gng->ops.new_node_between_data);
    svoGNGNodeAdd(gng, r);

    // 4. Create q-r and f-r edges and remove q-f edge (which is eqf)
    svoGNGEdgeDel(gng, eqf);
    svoGNGEdgeNew(gng, q, r);
    svoGNGEdgeNew(gng, f, r);

    // 5. Decrease error counters of q and f
    svoGNGNodeScaleError(gng, q, gng->params.alpha);
    svoGNGNodeScaleError(gng, f, gng->params.alpha);

    // 6. Set error counter of new node (r)
    r->err  = q->err + f->err;
    r->err /= BOR_REAL(2.);
    r->err_cycle = gng->cycle;
    borPairHeapUpdate(gng->err_heap, &r->err_heap);
}


svo_gng_node_t *svoGNGNodeWithHighestError(svo_gng_t *gng)
{
    bor_pairheap_node_t *max;
    svo_gng_node_t *maxn;

    max  = borPairHeapMin(gng->err_heap);
    maxn = bor_container_of(max, svo_gng_node_t, err_heap);

    /*
    if (maxn->err_cycle != gng->cycle){
        DBG2("");
    }

    {
        bor_list_t *list, *item;
        bor_net_node_t *nn;
        svo_gng_node_t *n, *__maxn = NULL;
        bor_real_t max;

        max = -BOR_REAL_MAX;
        list = borNetNodes(gng->net);
        BOR_LIST_FOR_EACH(list, item){
            nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
            n  = bor_container_of(nn, svo_gng_node_t, node);

            svoGNGNodeFixError(gng, n);
            if (n->err > max){
                max = n->err;
                __maxn = n;
            }
        }

        if (maxn != __maxn){
            DBG("%.30f %.30f", max, maxn->err);
        }
    }
    */

    return maxn;
}

void svoGNGHebbianLearning(svo_gng_t *gng,
                           svo_gng_node_t *n1, svo_gng_node_t *n2)
{
    bor_net_edge_t *nedge;
    svo_gng_edge_t *edge;

    nedge = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (!nedge){
        edge = svoGNGEdgeNew(gng, n1, n2);
    }else{
        edge = bor_container_of(nedge, svo_gng_edge_t, edge);
    }
    edge->age = 0;
}

static void __svoGNGNodeWithHighestError2(svo_gng_t *gng,
                                          svo_gng_node_t **n1,
                                          svo_gng_node_t **n2,
                                          svo_gng_edge_t **edge)
{
    svo_gng_node_t *q;
    bor_list_t *list, *item;
    bor_net_edge_t *ne;
    svo_gng_edge_t *e_highest;
    bor_net_node_t *nn;
    svo_gng_node_t *n, *n_highest;
    bor_real_t err_highest;

    q = svoGNGNodeWithHighestError(gng);

    err_highest = -BOR_ONE;
    n_highest = NULL;
    e_highest = NULL;

    list = borNetNodeEdges(&q->node);
    BOR_LIST_FOR_EACH(list, item){
        ne = borNetEdgeFromNodeList(item);
        nn = borNetEdgeOtherNode(ne, &q->node);
        n  = bor_container_of(nn, svo_gng_node_t, node);

        svoGNGNodeFixError(gng, n);
        borPairHeapUpdate(gng->err_heap, &n->err_heap);

        if (n->err > err_highest){
            err_highest = n->err;
            n_highest   = n;
            e_highest   = bor_container_of(ne, svo_gng_edge_t, edge);
        }
    }

    if (n1)
        *n1 = q;
    if (n2)
        *n2 = n_highest;
    if (edge)
        *edge = e_highest;
}

void svoGNGNodeWithHighestError2(svo_gng_t *gng,
                                 svo_gng_node_t **n1, svo_gng_node_t **n2,
                                 svo_gng_edge_t **edge)
{
    do {
        // 1. Get node with highest error counter and its neighbor with
        // highest error counter
        __svoGNGNodeWithHighestError2(gng, n1, n2, edge);

        // Node with highest error counter doesn't have any neighbors!
        // Generally, this shouldn't happen but if it does, it means that
        // user had to delete some node from outside. In this case delete
        // the {n1} node and try to find next node with highest error
        // counter.
        if (n2 && !*n2 && n1 && *n1){
            svoGNGNodeDel(gng, *n1);
        }
    } while (n2 && !*n2);
}







/*** Node functions ***/
void svoGNGNodeDisconnect(svo_gng_t *gng, svo_gng_node_t *node)
{
    bor_list_t *edges, *item, *itemtmp;
    bor_net_edge_t *ne;
    svo_gng_edge_t *edge;

    // remove incidenting edges
    edges = borNetNodeEdges(&node->node);
    BOR_LIST_FOR_EACH_SAFE(edges, item, itemtmp){
        ne = borNetEdgeFromNodeList(item);
        edge = svoGNGEdgeFromNet(ne);
        svoGNGEdgeDel(gng, edge);
    }
}

svo_gng_node_t *svoGNGNodeNewAtPos(svo_gng_t *gng, const void *is)
{
    svo_gng_node_t *r, *n1, *n2;
    svo_gng_edge_t *edge;

    gng->ops.nearest(is, &n1, &n2, gng->ops.nearest_data);

    r = gng->ops.new_node(is, gng->ops.new_node_data);
    svoGNGNodeAdd(gng, r);

    edge = svoGNGEdgeNew(gng, r, n1);
    edge->age = 0;
    edge = svoGNGEdgeNew(gng, r, n2);
    edge->age = 0;

    return r;
}


/*** Edge functions ***/
svo_gng_edge_t *svoGNGEdgeNew(svo_gng_t *gng, svo_gng_node_t *n1,
                                              svo_gng_node_t *n2)
{
    svo_gng_edge_t *e;

    e = BOR_ALLOC(svo_gng_edge_t);
    e->age = 0;

    borNetAddEdge(gng->net, &e->edge, &n1->node, &n2->node);

    return e;
}

void svoGNGEdgeDel(svo_gng_t *gng, svo_gng_edge_t *e)
{
    borNetRemoveEdge(gng->net, &e->edge);
    BOR_FREE(e);
}

void svoGNGEdgeBetweenDel(svo_gng_t *gng,
                          svo_gng_node_t *n1, svo_gng_node_t *n2)
{
    svo_gng_edge_t *e;

    if ((e = svoGNGEdgeBetween(gng, n1, n2)) != NULL)
        svoGNGEdgeDel(gng, e);
}







static int errHeapLT(const bor_pairheap_node_t *_n1,
                     const bor_pairheap_node_t *_n2,
                     void *data)
{
    svo_gng_t *gng = (svo_gng_t *)data;
    svo_gng_node_t *n1, *n2;

    n1 = bor_container_of(_n1, svo_gng_node_t, err_heap);
    n2 = bor_container_of(_n2, svo_gng_node_t, err_heap);

    svoGNGNodeFixError(gng, n1);
    svoGNGNodeFixError(gng, n2);
    return n1->err > n2->err;
}


static void nodeFinalDel(bor_net_node_t *node, void *data)
{
    svo_gng_t *gng = (svo_gng_t *)data;
    svo_gng_node_t *n;

    n = bor_container_of(node, svo_gng_node_t, node);
    gng->ops.del_node(n, gng->ops.del_node_data);
}

static void delEdge(bor_net_edge_t *edge, void *data)
{
    BOR_FREE(edge);
}
