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

#include <svoboda/kohonen.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>

/** Returns nearest node to the input signal */
static svo_kohonen_node_t *nearest(svo_kohonen_t *k, const bor_vec_t *is);
/** Updates weights of nodes in the network */
static void updateWeights(svo_kohonen_t *k, const bor_vec_t *is,
                                            svo_kohonen_node_t *wn);
/** n->w = n->w + rate * (w - n->w) */
static void svoKohonenNodeMoveTowards(svo_kohonen_t *k,
                                      svo_kohonen_node_t *n,
                                      const bor_vec_t *w,
                                      bor_real_t rate);

static void netNodeDel(bor_net_node_t *n, void *);
static void netEdgeDel(bor_net_edge_t *e, void *);

#define OPS_DATA(name) \
    if (!k->ops.name ## _data) \
        k->ops.name ## _data = k->ops.data;

#define OPS_CHECK(name) \
    if (!k->ops.name){ \
        fprintf(stderr, "Fermat :: Kohonen :: No " #name " callback set.\n"); \
        exit(-1); \
    }

#define OPS_CHECK_DATA(name) \
    OPS_DATA(name) \
    OPS_CHECK(name)


void svoKohonenOpsInit(svo_kohonen_ops_t *ops)
{
    bzero(ops, sizeof(*ops));
    ops->callback_period = 100;
}


void svoKohonenParamsInit(svo_kohonen_params_t *p)
{
    p->dim = 2;
    p->learn_rate = 0.1;
    borNNParamsInit(&p->nn);
}

svo_kohonen_t *svoKohonenNew(const svo_kohonen_ops_t *ops,
                             const svo_kohonen_params_t *params)
{
    svo_kohonen_t *k;

    k = BOR_ALLOC(svo_kohonen_t);

    k->ops = *ops;
    OPS_CHECK_DATA(input_signal)
    OPS_CHECK_DATA(neighborhood)
    OPS_CHECK_DATA(terminate)
    OPS_DATA(callback)

    k->params = *params;

    k->net = borNetNew();
    k->nn  = borNNNew(&k->params.nn);

    k->tmpv = borVecNew(k->params.dim);

    return k;
}

void svoKohonenDel(svo_kohonen_t *k)
{
    borNetDel2(k->net, netNodeDel, (void *)k, netEdgeDel, (void *)k);
    borNNDel(k->nn);
    borVecDel(k->tmpv);
    BOR_FREE(k);
}

static void svoKohonenReset(svo_kohonen_t *k)
{
    bor_list_t *list, *item;
    bor_net_node_t *nn;
    svo_kohonen_node_t *n;

    k->update = 1;

    list = borNetNodes(k->net);
    BOR_LIST_FOR_EACH(list, item){
        nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n  = bor_container_of(nn, svo_kohonen_node_t, net);
        n->update = 0;
    }
}

void svoKohonenRun(svo_kohonen_t *k)
{
    unsigned long counter;
    const bor_vec_t *is;
    svo_kohonen_node_t *win;

    if (borNetNodesLen(k->net) == 0){
        fprintf(stderr, "Fermat :: Kohonen :: No nodes in the map!\n");
        return;
    }

    svoKohonenReset(k);

    counter = 0UL;
    while (!k->ops.terminate(k, k->ops.terminate_data)){
        // get input signal
        is = k->ops.input_signal(k, k->ops.input_signal_data);

        // determine winning node
        win = nearest(k, is);

        // update weights
        updateWeights(k, is, win);

        counter += 1UL;
        if (k->ops.callback && counter == k->ops.callback_period){
            k->ops.callback(k, k->ops.callback_data);
            counter = 0UL;
        }
    }
}

void svoKohonenDumpSVT(const svo_kohonen_t *k, FILE *out, const char *name)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e;
    bor_net_node_t *netn;
    svo_kohonen_node_t *n;
    int i;

    fprintf(out, "-----\n");

    if (name){
        fprintf(out, "Name: %s\n", name);
    }

    fprintf(out, "Points:\n");
    list = borNetNodes(k->net);
    i = 0;
    BOR_LIST_FOR_EACH(list, item){
        netn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(netn, svo_kohonen_node_t, net);

        n->_id = i;
        borVecPrint(k->params.dim, n->w, out);
        fprintf(out, "\n");

        i++;
    }

    fprintf(out, "Edges:\n");
    list = borNetEdges(k->net);
    BOR_LIST_FOR_EACH(list, item){
        e = BOR_LIST_ENTRY(item, bor_net_edge_t, list);

        netn = borNetEdgeNode(e, 0);
        n    = bor_container_of(netn, svo_kohonen_node_t, net);
        fprintf(out, "%d ", n->_id);

        netn = borNetEdgeNode(e, 1);
        n    = bor_container_of(netn, svo_kohonen_node_t, net);
        fprintf(out, "%d\n", n->_id);
    }

    fprintf(out, "-----\n");
}

svo_kohonen_node_t *svoKohonenNodeNew(svo_kohonen_t *k, const bor_vec_t *init)
{
    svo_kohonen_node_t *n;

    n = BOR_ALLOC(svo_kohonen_node_t);
    n->w = borVecNew(k->params.dim);
    if (init)
        borVecCopy(k->params.dim, n->w, init);

    borNetAddNode(k->net, &n->net);
    borNNElInit(k->nn, &n->nn, n->w);
    borNNAdd(k->nn, &n->nn);
    return n;
}

void svoKohonenNodeDel(svo_kohonen_t *k, svo_kohonen_node_t *n)
{
    BOR_FREE(n->w);
    borNetRemoveNode(k->net, &n->net);
    borNNRemove(k->nn, &n->nn);
    BOR_FREE(n);
}

void svoKohonenNodeConnect(svo_kohonen_t *k,
                           svo_kohonen_node_t *n1,
                           svo_kohonen_node_t *n2)
{
    bor_net_edge_t *e;
    e = borNetEdgeNew();
    borNetAddEdge(k->net, e, &n1->net, &n2->net);
}


static svo_kohonen_node_t *nearest(svo_kohonen_t *k, const bor_vec_t *is)
{
    bor_nn_el_t *el;
    svo_kohonen_node_t *n;

    borNNNearest(k->nn, is, 1, &el);
    n = bor_container_of(el, svo_kohonen_node_t, nn);
    return n;
}

static void updateWeightsUpdateFifo(svo_kohonen_node_t *n,
                                    bor_list_t *fifo,
                                    unsigned int update,
                                    unsigned int depth)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e;
    bor_net_node_t *no;
    svo_kohonen_node_t *o;

    list = borNetNodeEdges(&n->net);
    BOR_LIST_FOR_EACH(list, item){
        e = borNetEdgeFromNodeList(item);
        no = borNetEdgeOtherNode(e, &n->net);
        o  = bor_container_of(no, svo_kohonen_node_t, net);

        if (o->update != update){
            borListAppend(fifo, &o->fifo);
            o->update = update;
            o->depth  = depth;
        }
    }
}

static void updateWeights(svo_kohonen_t *k, const bor_vec_t *is,
                                            svo_kohonen_node_t *wn)
{
    bor_list_t fifo, *item;
    svo_kohonen_node_t *n;
    bor_real_t rate;
    int neigh;

    // update winner's weight
    svoKohonenNodeMoveTowards(k, wn, is, k->params.learn_rate);

    // initialize fifo list
    borListInit(&fifo);

    wn->update = k->update;
    wn->depth  = 0;
    updateWeightsUpdateFifo(wn, &fifo, k->update, wn->depth);
    while (!borListEmpty(&fifo)){
        item = borListNext(&fifo);
        borListDel(item);
        n = BOR_LIST_ENTRY(item, svo_kohonen_node_t, fifo);

        neigh = k->ops.neighborhood(k, wn, n, n->depth, &rate,
                                    k->ops.neighborhood_data);

        if (neigh){
            // move node towards input signal
            svoKohonenNodeMoveTowards(k, n, is, k->params.learn_rate * rate);

            // add n's children to fifo
            updateWeightsUpdateFifo(n, &fifo, k->update, n->depth + 1);
        }
    }

    k->update = k->update + 1u;
}

static void svoKohonenNodeMoveTowards(svo_kohonen_t *k,
                                      svo_kohonen_node_t *n,
                                      const bor_vec_t *w,
                                      bor_real_t rate)
{
    if (!svoKohonenNodeFixed(n)){
        borVecSub2(k->params.dim, k->tmpv, w, n->w);
        borVecScale(k->params.dim, k->tmpv, rate);
        borVecAdd(k->params.dim, n->w, k->tmpv);
        borNNUpdate(k->nn, &n->nn);
    }
}


static void netNodeDel(bor_net_node_t *n, void *_k)
{
    svo_kohonen_t *k = (svo_kohonen_t *)_k;
    svo_kohonen_node_t *node = bor_container_of(n, svo_kohonen_node_t, net);
    svoKohonenNodeDel(k, node);
}

static void netEdgeDel(bor_net_edge_t *e, void *_k)
{
    borNetEdgeDel(e);
}
