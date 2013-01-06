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

#include <stdio.h>
#include <boruvka/nearest-linear.h>
#include <boruvka/vec3.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>
#include "gng/gng-eu.h"

/** Operations for svo_gng_ops_t struct */
static svo_gng_eu_node_t *svoGNGEuNodeNew(svo_gng_eu_t *gng, const bor_vec_t *is);
static svo_gng_eu_node_t *svoGNGEuNodeNewBetween(svo_gng_eu_t *gng,
                                                 const svo_gng_eu_node_t *n1,
                                                 const svo_gng_eu_node_t *n2);

static const void *svoGNGEuInputSignal(void *);
static void svoGNGEuNearest(svo_gng_eu_t *gng,
                            const bor_vec_t *is,
                            svo_gng_eu_node_t **n1,
                            svo_gng_eu_node_t **n2);

_bor_inline bor_real_t svoGNGEuDist2(svo_gng_eu_t *gng,
                                     const bor_vec_t *is,
                                     const svo_gng_eu_node_t *node);

_bor_inline void svoGNGEuMoveTowards(svo_gng_eu_t *gng,
                                     svo_gng_eu_node_t *node,
                                     const bor_vec_t *is,
                                     bor_real_t fraction);






/** Should return true if n1 > n2 - this is used for err-heap */
static int errHeapLT(const bor_pairheap_node_t *n1,
                     const bor_pairheap_node_t *n2, void *);


/** Delete callbacks */
static void nodeFinalDel(bor_net_node_t *node, void *data);
static void delEdge(bor_net_edge_t *edge, void *data);

void svoGNGEuOpsInit(svo_gng_eu_ops_t *ops)
{
    bzero(ops, sizeof(svo_gng_eu_ops_t));
}

void svoGNGEuParamsInit(svo_gng_eu_params_t *params)
{
    params->dim = 2;

    params->lambda  = 200;
    params->eb      = 0.05;
    params->en      = 0.0006;
    params->alpha   = 0.95;
    params->beta    = 0.9995;
    params->age_max = 200;

    borNNParamsInit(&params->nn);
    params->nn.type = BOR_NN_GUG;
}


svo_gng_eu_t *svoGNGEuNew(const svo_gng_eu_ops_t *ops,
                     const svo_gng_eu_params_t *params)
{
    svo_gng_eu_t *gng_eu;
    bor_nn_params_t nnp;
    size_t i;
    bor_real_t maxbeta;

    gng_eu = BOR_ALLOC(svo_gng_eu_t);

    gng_eu->net = borNetNew();

    gng_eu->ops    = *ops;
    gng_eu->params = *params;

    // set up ops data pointers
    if (!gng_eu->ops.new_node_data)
        gng_eu->ops.new_node_data = gng_eu->ops.data;
    if (!gng_eu->ops.del_node_data)
        gng_eu->ops.del_node_data = gng_eu->ops.data;
    if (!gng_eu->ops.input_signal_data)
        gng_eu->ops.input_signal_data = gng_eu->ops.data;
    if (!gng_eu->ops.terminate_data)
        gng_eu->ops.terminate_data = gng_eu->ops.data;
    if (!gng_eu->ops.callback_data)
        gng_eu->ops.callback_data = gng_eu->ops.data;


    // initialize error heap
    gng_eu->err_heap = borPairHeapNew(errHeapLT, (void *)gng_eu);

    // precompute beta^n
    gng_eu->beta_n = BOR_ALLOC_ARR(bor_real_t, gng_eu->params.lambda);
    gng_eu->beta_n[0] = gng_eu->params.beta;
    for (i = 1; i < gng_eu->params.lambda; i++){
        gng_eu->beta_n[i] = gng_eu->beta_n[i - 1] * gng_eu->params.beta;
    }

    // precompute beta^(n * lambda)
    maxbeta = gng_eu->beta_n[gng_eu->params.lambda - 1];
    gng_eu->beta_lambda_n_len = 1000;
    gng_eu->beta_lambda_n = BOR_ALLOC_ARR(bor_real_t, gng_eu->beta_lambda_n_len);
    gng_eu->beta_lambda_n[0] = maxbeta;
    for (i = 1; i < gng_eu->beta_lambda_n_len; i++){
        gng_eu->beta_lambda_n[i] = gng_eu->beta_lambda_n[i - 1] * maxbeta;
    }

    gng_eu->cycle = 1L;
    gng_eu->step  = 1;


    // initialize nncells
    nnp = params->nn;
    nnp.gug.dim = params->dim;
    nnp.vptree.dim = params->dim;
    nnp.linear.dim = params->dim;
    gng_eu->nn = borNNNew(&nnp);

    // initialize temporary vector
    if (gng_eu->params.dim == 2){
        gng_eu->tmpv = (bor_vec_t *)borVec2New(BOR_ZERO, BOR_ZERO);
    }else if (gng_eu->params.dim == 3){
        gng_eu->tmpv = (bor_vec_t *)borVec3New(BOR_ZERO, BOR_ZERO, BOR_ZERO);
    }else{
        gng_eu->tmpv = borVecNew(gng_eu->params.dim);
    }

    return gng_eu;
}

void svoGNGEuDel(svo_gng_eu_t *gng_eu)
{
    if (gng_eu->beta_n)
        BOR_FREE(gng_eu->beta_n);
    if (gng_eu->beta_lambda_n)
        BOR_FREE(gng_eu->beta_lambda_n);

    if (gng_eu->net){
        borNetDel2(gng_eu->net, nodeFinalDel, gng_eu,
                              delEdge, gng_eu);
    }

    if (gng_eu->err_heap)
        borPairHeapDel(gng_eu->err_heap);

    if (gng_eu->nn)
        borNNDel(gng_eu->nn);

    if (gng_eu->params.dim == 2){
        borVec2Del((bor_vec2_t *)gng_eu->tmpv);
    }else if (gng_eu->params.dim == 3){
        borVec3Del((bor_vec3_t *)gng_eu->tmpv);
    }else{
        borVecDel(gng_eu->tmpv);
    }

    BOR_FREE(gng_eu);
}


void svoGNGEuRun(svo_gng_eu_t *gng_eu)
{
    unsigned long cycle;
    size_t i;

    cycle = 0;
    svoGNGEuInit(gng_eu);

    do {
        for (i = 0; i < gng_eu->params.lambda; i++){
            svoGNGEuLearn(gng_eu);
        }
        svoGNGEuNewNode(gng_eu);

        cycle++;
        if (gng_eu->ops.callback && gng_eu->ops.callback_period == cycle){
            gng_eu->ops.callback(gng_eu->ops.callback_data);
            cycle = 0L;
        }
    } while (!gng_eu->ops.terminate(gng_eu->ops.terminate_data));
}

void svoGNGEuInit(svo_gng_eu_t *gng_eu)
{
    const bor_vec_t *is;
    svo_gng_eu_node_t *n1 = NULL, *n2 = NULL;

    gng_eu->cycle = 1L;
    gng_eu->step  = 1;

    is = gng_eu->ops.input_signal(gng_eu->ops.input_signal_data);
    n1 = svoGNGEuNodeNew(gng_eu, is);

    is = gng_eu->ops.input_signal(gng_eu->ops.input_signal_data);
    n2 = svoGNGEuNodeNew(gng_eu, is);

    svoGNGEuEdgeNew(gng_eu, n1, n2);
}

void svoGNGEuLearn(svo_gng_eu_t *gng_eu)
{
    const bor_vec_t *input_signal;
    bor_net_node_t *nn;
    svo_gng_eu_node_t *n1, *n2, *n;
    bor_net_edge_t *nedge;
    svo_gng_eu_edge_t *edge;
    bor_real_t dist2;
    bor_list_t *list, *item, *item_tmp;

    if (gng_eu->step > gng_eu->params.lambda){
        gng_eu->cycle += 1L;
        gng_eu->step = 1;
    }

    // 1. Get input signal
    input_signal = gng_eu->ops.input_signal(gng_eu->ops.input_signal_data);

    // 2. Find two nearest nodes to input signal
    svoGNGEuNearest(gng_eu, input_signal, &n1, &n2);

    // 3. Create connection between n1 and n2 if doesn't exist and set age
    //    to zero
    svoGNGEuHebbianLearning(gng_eu, n1, n2);

    // 4. Increase error counter of winner node
    dist2 = svoGNGEuDist2(gng_eu, input_signal, n1);
    svoGNGEuNodeIncError(gng_eu, n1, dist2 * gng_eu->beta_n[gng_eu->params.lambda - gng_eu->step]);

    // 5. Adapt nodes to input signal using fractions eb and en
    // + 6. Increment age of all edges by one
    // + 7. Remove edges with age higher than age_max
    svoGNGEuMoveTowards(gng_eu, n1, input_signal, gng_eu->params.eb);
    // adapt also direct topological neighbors of winner node
    list = borNetNodeEdges(&n1->node);
    BOR_LIST_FOR_EACH_SAFE(list, item, item_tmp){
        nedge = borNetEdgeFromNodeList(item);
        edge  = bor_container_of(nedge, svo_gng_eu_edge_t, edge);
        nn   = borNetEdgeOtherNode(&edge->edge, &n1->node);
        n    = bor_container_of(nn, svo_gng_eu_node_t, node);

        // increase age (6.)
        edge->age += 1;

        // remove edge if it has age higher than age_max (7.)
        if (edge->age > gng_eu->params.age_max){
            svoGNGEuEdgeDel(gng_eu, edge);

            if (borNetNodeEdgesLen(nn) == 0){
                // remove node if not connected into net anymore
                svoGNGEuNodeDel(gng_eu, n);
                n = NULL;
            }
        }

        // move node (5.)
        if (n){
            svoGNGEuMoveTowards(gng_eu, n, input_signal, gng_eu->params.en);
        }
    }

    // remove winning node if not connected into net
    if (borNetNodeEdgesLen(&n1->node) == 0){
        // remove node if not connected into net anymore
        svoGNGEuNodeDel(gng_eu, n1);
    }

    ++gng_eu->step;
}

void svoGNGEuNewNode(svo_gng_eu_t *gng_eu)
{
    svo_gng_eu_node_t *q, *f, *r;
    svo_gng_eu_edge_t *eqf;

    // 1. Get node with highest error counter and its neighbor with
    // highest error counter
    svoGNGEuNodeWithHighestError2(gng_eu, &q, &f, &eqf);

    // 3. Create new node between q and f
    r = svoGNGEuNodeNewBetween(gng_eu, q, f);

    // 4. Create q-r and f-r edges and remove q-f edge (which is eqf)
    svoGNGEuEdgeDel(gng_eu, eqf);
    svoGNGEuEdgeNew(gng_eu, q, r);
    svoGNGEuEdgeNew(gng_eu, f, r);

    // 5. Decrease error counters of q and f
    svoGNGEuNodeScaleError(gng_eu, q, gng_eu->params.alpha);
    svoGNGEuNodeScaleError(gng_eu, f, gng_eu->params.alpha);

    // 6. Set error counter of new node (r)
    r->err  = q->err + f->err;
    r->err /= BOR_REAL(2.);
    r->err_cycle = gng_eu->cycle;
    borPairHeapUpdate(gng_eu->err_heap, &r->err_heap);
}


svo_gng_eu_node_t *svoGNGEuNodeWithHighestError(svo_gng_eu_t *gng_eu)
{
    bor_pairheap_node_t *max;
    svo_gng_eu_node_t *maxn;

    max  = borPairHeapMin(gng_eu->err_heap);
    maxn = bor_container_of(max, svo_gng_eu_node_t, err_heap);

    /*
    if (maxn->err_cycle != gng_eu->cycle){
        DBG2("");
    }

    {
        bor_list_t *list, *item;
        bor_net_node_t *nn;
        svo_gng_eu_node_t *n, *__maxn = NULL;
        bor_real_t max;

        max = -BOR_REAL_MAX;
        list = borNetNodes(gng_eu->net);
        BOR_LIST_FOR_EACH(list, item){
            nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
            n  = bor_container_of(nn, svo_gng_eu_node_t, node);

            svoGNGEuNodeFixError(gng_eu, n);
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

void svoGNGEuHebbianLearning(svo_gng_eu_t *gng_eu,
                             svo_gng_eu_node_t *n1, svo_gng_eu_node_t *n2)
{
    bor_net_edge_t *nedge;
    svo_gng_eu_edge_t *edge;

    nedge = borNetNodeCommonEdge(&n1->node, &n2->node);
    if (!nedge){
        edge = svoGNGEuEdgeNew(gng_eu, n1, n2);
    }else{
        edge = bor_container_of(nedge, svo_gng_eu_edge_t, edge);
    }
    edge->age = 0;
}

static void __svoGNGEuNodeWithHighestError2(svo_gng_eu_t *gng_eu,
                                          svo_gng_eu_node_t **n1,
                                          svo_gng_eu_node_t **n2,
                                          svo_gng_eu_edge_t **edge)
{
    svo_gng_eu_node_t *q;
    bor_list_t *list, *item;
    bor_net_edge_t *ne;
    svo_gng_eu_edge_t *e_highest;
    bor_net_node_t *nn;
    svo_gng_eu_node_t *n, *n_highest;
    bor_real_t err_highest;

    q = svoGNGEuNodeWithHighestError(gng_eu);

    err_highest = -BOR_ONE;
    n_highest = NULL;
    e_highest = NULL;

    list = borNetNodeEdges(&q->node);
    BOR_LIST_FOR_EACH(list, item){
        ne = borNetEdgeFromNodeList(item);
        nn = borNetEdgeOtherNode(ne, &q->node);
        n  = bor_container_of(nn, svo_gng_eu_node_t, node);

        svoGNGEuNodeFixError(gng_eu, n);
        borPairHeapUpdate(gng_eu->err_heap, &n->err_heap);

        if (n->err > err_highest){
            err_highest = n->err;
            n_highest   = n;
            e_highest   = bor_container_of(ne, svo_gng_eu_edge_t, edge);
        }
    }

    if (n1)
        *n1 = q;
    if (n2)
        *n2 = n_highest;
    if (edge)
        *edge = e_highest;
}

void svoGNGEuNodeWithHighestError2(svo_gng_eu_t *gng_eu,
                                 svo_gng_eu_node_t **n1, svo_gng_eu_node_t **n2,
                                 svo_gng_eu_edge_t **edge)
{
    do {
        // 1. Get node with highest error counter and its neighbor with
        // highest error counter
        __svoGNGEuNodeWithHighestError2(gng_eu, n1, n2, edge);

        // Node with highest error counter doesn't have any neighbors!
        // Generally, this shouldn't happen but if it does, it means that
        // user had to delete some node from outside. In this case delete
        // the {n1} node and try to find next node with highest error
        // counter.
        if (n2 && !*n2 && n1 && *n1){
            svoGNGEuNodeDel(gng_eu, *n1);
        }
    } while (n2 && !*n2);
}


void svoGNGEuDumpSVT(svo_gng_eu_t *gng_eu, FILE *out, const char *name)
{
    bor_list_t *list, *item;
    bor_net_node_t *nn;
    svo_gng_eu_node_t *n;
    bor_net_edge_t *e;
    size_t i, id1, id2;

    if (gng_eu->params.dim != 2 && gng_eu->params.dim != 3)
        return;

    fprintf(out, "--------\n");

    if (name)
        fprintf(out, "Name: %s\n", name);

    fprintf(out, "Points:\n");
    list = svoGNGEuNodes(gng_eu);
    i = 0;
    BOR_LIST_FOR_EACH(list, item){
        n = svoGNGEuNodeFromList(item);

        n->_id = i++;
        if (gng_eu->params.dim == 2){
            borVec2Print((const bor_vec2_t *)n->w, out);
        }else{
            borVec3Print((const bor_vec3_t *)n->w, out);
        }
        fprintf(out, "\n");
    }


    fprintf(out, "Edges:\n");
    list = svoGNGEuEdges(gng_eu);
    BOR_LIST_FOR_EACH(list, item){
        e = BOR_LIST_ENTRY(item, bor_net_edge_t, list);

        nn = borNetEdgeNode(e, 0);
        n  = svoGNGEuNodeFromNet(nn);
        id1 = n->_id;

        nn = borNetEdgeNode(e, 1);
        n  = svoGNGEuNodeFromNet(nn);
        id2 = n->_id;
        fprintf(out, "%d %d\n", (int)id1, (int)id2);
    }

    fprintf(out, "--------\n");
}






/*** Node functions ***/
void svoGNGEuNodeDisconnect(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *node)
{
    bor_list_t *edges, *item, *itemtmp;
    bor_net_edge_t *ne;
    svo_gng_eu_edge_t *edge;

    // remove incidenting edges
    edges = borNetNodeEdges(&node->node);
    BOR_LIST_FOR_EACH_SAFE(edges, item, itemtmp){
        ne = borNetEdgeFromNodeList(item);
        edge = svoGNGEuEdgeFromNet(ne);
        svoGNGEuEdgeDel(gng_eu, edge);
    }
}

svo_gng_eu_node_t *svoGNGEuNodeNewAtPos(svo_gng_eu_t *gng_eu, const void *is)
{
    svo_gng_eu_node_t *r, *n1, *n2;
    svo_gng_eu_edge_t *edge;

    svoGNGEuNearest(gng_eu, is, &n1, &n2);

    r = svoGNGEuNodeNew(gng_eu, (const bor_vec_t *)is);

    edge = svoGNGEuEdgeNew(gng_eu, r, n1);
    edge->age = 0;
    edge = svoGNGEuEdgeNew(gng_eu, r, n2);
    edge->age = 0;

    return r;
}


/*** Edge functions ***/
svo_gng_eu_edge_t *svoGNGEuEdgeNew(svo_gng_eu_t *gng_eu, svo_gng_eu_node_t *n1,
                                              svo_gng_eu_node_t *n2)
{
    svo_gng_eu_edge_t *e;

    e = BOR_ALLOC(svo_gng_eu_edge_t);
    e->age = 0;

    borNetAddEdge(gng_eu->net, &e->edge, &n1->node, &n2->node);

    return e;
}

void svoGNGEuEdgeDel(svo_gng_eu_t *gng_eu, svo_gng_eu_edge_t *e)
{
    borNetRemoveEdge(gng_eu->net, &e->edge);
    BOR_FREE(e);
}

void svoGNGEuEdgeBetweenDel(svo_gng_eu_t *gng_eu,
                          svo_gng_eu_node_t *n1, svo_gng_eu_node_t *n2)
{
    svo_gng_eu_edge_t *e;

    if ((e = svoGNGEuEdgeBetween(gng_eu, n1, n2)) != NULL)
        svoGNGEuEdgeDel(gng_eu, e);
}







static int errHeapLT(const bor_pairheap_node_t *_n1,
                     const bor_pairheap_node_t *_n2,
                     void *data)
{
    svo_gng_eu_t *gng_eu = (svo_gng_eu_t *)data;
    svo_gng_eu_node_t *n1, *n2;

    n1 = bor_container_of(_n1, svo_gng_eu_node_t, err_heap);
    n2 = bor_container_of(_n2, svo_gng_eu_node_t, err_heap);

    svoGNGEuNodeFixError(gng_eu, n1);
    svoGNGEuNodeFixError(gng_eu, n2);
    return n1->err > n2->err;
}


static void nodeFinalDel(bor_net_node_t *node, void *data)
{
    svo_gng_eu_t *gng_eu = (svo_gng_eu_t *)data;
    svo_gng_eu_node_t *n;

    n = bor_container_of(node, svo_gng_eu_node_t, node);
    svoGNGEuNodeDel(gng_eu, n);
}

static void delEdge(bor_net_edge_t *edge, void *data)
{
    BOR_FREE(edge);
}




static svo_gng_eu_node_t *svoGNGEuNodeNew(svo_gng_eu_t *gng, const bor_vec_t *is)
{
    svo_gng_eu_node_t *n;

    if (gng->ops.new_node){
        n = gng->ops.new_node(is, gng->ops.new_node_data);
    }else{
        n = BOR_ALLOC(svo_gng_eu_node_t);
    }
    svoGNGEuNodeAdd(gng, n, is);

    return n;
}

static svo_gng_eu_node_t *svoGNGEuNodeNewBetween(svo_gng_eu_t *gng,
                                                 const svo_gng_eu_node_t *n1,
                                                 const svo_gng_eu_node_t *n2)
{
    if (gng->params.dim == 2){
        borVec2Add2((bor_vec2_t *)gng->tmpv, (const bor_vec2_t *)n1->w,
                                             (const bor_vec2_t *)n2->w);
        borVec2Scale((bor_vec2_t *)gng->tmpv, BOR_REAL(0.5));
    }else if (gng->params.dim == 3){
        borVec3Add2((bor_vec3_t *)gng->tmpv, (const bor_vec3_t *)n1->w,
                                             (const bor_vec3_t *)n2->w);
        borVec3Scale((bor_vec3_t *)gng->tmpv, BOR_REAL(0.5));
    }else{
        borVecAdd2(gng->params.dim, gng->tmpv, n1->w, n2->w);
        borVecScale(gng->params.dim, gng->tmpv, BOR_REAL(0.5));
    }

    return svoGNGEuNodeNew(gng, gng->tmpv);
}

static const void *svoGNGEuInputSignal(void *data)
{
    /*
    svo_gng_eu_t *gng = (svo_gng_eu_t *)data;
    const bor_vec_t *v;

    if (borPCItEnd(&gng->pcit)){
        borPCPermutate(gng->pc);
        borPCItInit(&gng->pcit, gng->pc);
    }

    v = borPCItGet(&gng->pcit);
    borPCItNext(&gng->pcit);

    return v;
    */
    // TODO
    return NULL;
}


static void svoGNGEuNearest(svo_gng_eu_t *gng,
                            const bor_vec_t *is,
                            svo_gng_eu_node_t **n1,
                            svo_gng_eu_node_t **n2)
{
    bor_nn_el_t *els[2];

    *n1 = *n2 = NULL;

    borNNNearest(gng->nn, is, 2, els);

    *n1 = bor_container_of(els[0], svo_gng_eu_node_t, nn);
    *n2 = bor_container_of(els[1], svo_gng_eu_node_t, nn);
}



_bor_inline bor_real_t svoGNGEuDist2(svo_gng_eu_t *gng,
                                     const bor_vec_t *is,
                                     const svo_gng_eu_node_t *n)
{
    if (gng->params.dim == 2){
        return borVec2Dist2((const bor_vec2_t *)is, (const bor_vec2_t *)n->w);
    }else if (gng->params.dim == 3){
        return borVec3Dist2((const bor_vec3_t *)is, (const bor_vec3_t *)n->w);
    }else{
        return borVecDist2(gng->params.dim, is, n->w);
    }
}

_bor_inline void svoGNGEuMoveTowards(svo_gng_eu_t *gng,
                                     svo_gng_eu_node_t *n,
                                     const bor_vec_t *is,
                                     bor_real_t fraction)
{
    if (gng->params.dim == 2){
        borVec2Sub2((bor_vec2_t *)gng->tmpv, (const bor_vec2_t *)is,
                                             (const bor_vec2_t *)n->w);
        borVec2Scale((bor_vec2_t *)gng->tmpv, fraction);
        borVec2Add((bor_vec2_t *)n->w, (const bor_vec2_t *)gng->tmpv);
    }else if (gng->params.dim == 3){
        borVec3Sub2((bor_vec3_t *)gng->tmpv, (const bor_vec3_t *)is,
                                             (const bor_vec3_t *)n->w);
        borVec3Scale((bor_vec3_t *)gng->tmpv, fraction);
        borVec3Add((bor_vec3_t *)n->w, (const bor_vec3_t *)gng->tmpv);
    }else{
        borVecSub2(gng->params.dim, gng->tmpv, is, n->w);
        borVecScale(gng->params.dim, gng->tmpv, fraction);
        borVecAdd(gng->params.dim, n->w, gng->tmpv);
    }

    borNNUpdate(gng->nn, &n->nn);
}
