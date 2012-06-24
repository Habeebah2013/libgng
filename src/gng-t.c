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

#include <svoboda/gng-t.h>
#include <boruvka/dbg.h>
#include <boruvka/alloc.h>

static void svoGNGTHebbianLearning(svo_gngt_t *gng,
                                   svo_gngt_node_t *n1,
                                   svo_gngt_node_t *n2);
static svo_gngt_node_t *svoGNGTNodeNeighborWithHighestErr(svo_gngt_t *gng,
                                                          svo_gngt_node_t *n);

/** Delete callbacks */
static void nodeFinalDel(bor_net_node_t *node, void *data);
static void delEdge(bor_net_edge_t *edge, void *data);

void svoGNGTOpsInit(svo_gngt_ops_t *ops)
{
    bzero(ops, sizeof(svo_gngt_ops_t));
}

void svoGNGTParamsInit(svo_gngt_params_t *params)
{
    params->lambda  = 200;
    params->eb      = 0.05;
    params->en      = 0.0006;
    params->age_max = 200;
    params->target  = 100.;
}

svo_gngt_t *svoGNGTNew(const svo_gngt_ops_t *ops,
                       const svo_gngt_params_t *params)
{
    svo_gngt_t *gng;

    gng = BOR_ALLOC(svo_gngt_t);

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

    return gng;
}

void svoGNGTDel(svo_gngt_t *gng)
{
    if (gng->net){
        borNetDel2(gng->net, nodeFinalDel, gng,
                             delEdge, gng);
    }

    BOR_FREE(gng);
}

void svoGNGTRun(svo_gngt_t *gng)
{
    unsigned long cycle = 0L;
    size_t i;

    svoGNGTInit(gng);

    do {
        svoGNGTReset(gng);
        for (i = 0; i < gng->params.lambda; i++){
            svoGNGTAdapt(gng);
        }

        svoGNGTGrowShrink(gng);

        cycle++;
        if (gng->ops.callback && gng->ops.callback_period == cycle){
            gng->ops.callback(gng->ops.callback_data);
            cycle = 0L;
        }
    } while (!gng->ops.terminate(gng->ops.terminate_data));
}

void svoGNGTInit(svo_gngt_t *gng)
{
    const void *is;
    svo_gngt_node_t *n1 = NULL, *n2 = NULL;

    if (gng->ops.init){
        gng->ops.init(&n1, &n2, gng->ops.init_data);
    }else{
        is = gng->ops.input_signal(gng->ops.input_signal_data);
        n1 = gng->ops.new_node(is, gng->ops.new_node_data);

        is = gng->ops.input_signal(gng->ops.input_signal_data);
        n2 = gng->ops.new_node(is, gng->ops.new_node_data);
    }

    svoGNGTNodeAdd(gng, n1);
    svoGNGTNodeAdd(gng, n2);
    svoGNGTEdgeNew(gng, n1, n2);
}

void svoGNGTReset(svo_gngt_t *gng)
{
    bor_list_t *list, *item;
    svo_gngt_node_t *n;

    list = svoGNGTNodes(gng);
    BOR_LIST_FOR_EACH(list, item){
        n = svoGNGTNodeFromList(item);
        n->err = 0;
        n->won = 0;
    }
}

void svoGNGTAdapt(svo_gngt_t *gng)
{
    const void *is;
    bor_net_edge_t *ne;
    bor_net_node_t *nn;
    svo_gngt_node_t *n1, *n2;
    svo_gngt_edge_t *e;
    bor_real_t dist2;
    bor_list_t *list, *item, *item_tmp;

    // 1. Get input signal
    is = gng->ops.input_signal(gng->ops.input_signal_data);

    // 2. Find two nearest nodes to input signal
    gng->ops.nearest(is, &n1, &n2, gng->ops.nearest_data);
    n1->won = 1;

    // 3. Create (or refresh) an edge between n1 and n2
    svoGNGTHebbianLearning(gng, n1, n2);

    // 4. Update accumulator
    dist2 = gng->ops.dist2(is, n1, gng->ops.dist2_data);
    n1->err += dist2;

    // 5. Move winner node towards is
    gng->ops.move_towards(n1, is, gng->params.eb,
                          gng->ops.move_towards_data);

    // 6. Move n1's neighbors towards is
    // + 7. Increment age of all edges emanating from n1
    // + 8. Remove edges with age > age_max
    list = borNetNodeEdges(&n1->node);
    BOR_LIST_FOR_EACH_SAFE(list, item, item_tmp){
        ne = borNetEdgeFromNodeList(item);
        e  = bor_container_of(ne, svo_gngt_edge_t, edge);
        nn = borNetEdgeOtherNode(ne, &n1->node);
        n2 = bor_container_of(nn, svo_gngt_node_t, node);

        // increment age (7.)
        e->age += 1;

        // delete edge (8.)
        if (e->age > gng->params.age_max){
            svoGNGTEdgeDel(gng, e);

            if (borNetNodeEdgesLen(&n2->node) == 0){
                // remove node if not connected into net anymore
                svoGNGTNodeDel(gng, n2);
            }
        }else{
            // move node (6.)
            gng->ops.move_towards(n2, is, gng->params.en,
                                  gng->ops.move_towards_data);
        }
    }

    // remove winning node if not connected into net
    if (borNetNodeEdgesLen(&n1->node) == 0){
        // remove node if not connected into net anymore
        svoGNGTNodeDel(gng, n1);
    }
}

void svoGNGTGrowShrink(svo_gngt_t *gng)
{
    bor_list_t *list, *item, *item_tmp;
    svo_gngt_node_t *n, *max, *min, *max2;
    svo_gngt_edge_t *e;
    bor_real_t avg, num;

    avg = num = BOR_ZERO;
    max = min = NULL;

    list = svoGNGTNodes(gng);
    BOR_LIST_FOR_EACH_SAFE(list, item, item_tmp){
        n = svoGNGTNodeFromList(item);

        /*
        if (!n->won){
            svoGNGTNodeDel(gng, n);
            continue;
        }
        */

        avg += n->err;
        num += BOR_ONE;

        if (!max || max->err < n->err)
            max = n;
        if (!min || min->err > n->err)
            min = n;
    }

    // compute average error
    avg /= num;
    gng->avg_err = avg;

    if (gng->params.target < avg){
        // more accuracy required
        if (max && (max2 = svoGNGTNodeNeighborWithHighestErr(gng, max))){
            n = gng->ops.new_node_between(max, max2,
                                          gng->ops.new_node_between_data);
            svoGNGTNodeAdd(gng, n);
            svoGNGTEdgeNew(gng, n, max);
            svoGNGTEdgeNew(gng, n, max2);

            e = svoGNGTEdgeBetween(gng, max, max2);
            svoGNGTEdgeDel(gng, e);
        }
    }else{
        // too much accuracy, remove the node with the smallest error
        if (min)
            svoGNGTNodeDel(gng, min);
    }

    if (svoGNGTNodesLen(gng) < 2){
        fprintf(stderr, "GNG-T Error: Check the parameters! The network shrinks too much.\n");
        exit(-1);
    }
}




void svoGNGTNodeDisconnect(svo_gngt_t *gng, svo_gngt_node_t *n)
{
    bor_list_t *edges, *item, *itemtmp;
    bor_net_edge_t *ne;
    svo_gngt_edge_t *edge;

    // remove incidenting edges
    edges = borNetNodeEdges(&n->node);
    BOR_LIST_FOR_EACH_SAFE(edges, item, itemtmp){
        ne = borNetEdgeFromNodeList(item);
        edge = svoGNGTEdgeFromNet(ne);
        svoGNGTEdgeDel(gng, edge);
    }
}

svo_gngt_node_t *svoGNGTNodeNewAtPos(svo_gngt_t *gng, const void *is)
{
    svo_gngt_node_t *r, *n1, *n2;

    gng->ops.nearest(is, &n1, &n2, gng->ops.nearest_data);

    r = gng->ops.new_node(is, gng->ops.new_node_data);
    svoGNGTNodeAdd(gng, r);

    svoGNGTEdgeNew(gng, r, n1);
    svoGNGTEdgeNew(gng, r, n2);

    return r;
}

svo_gngt_edge_t *svoGNGTEdgeNew(svo_gngt_t *gng, svo_gngt_node_t *n1,
                                                 svo_gngt_node_t *n2)
{
    svo_gngt_edge_t *e;

    e = BOR_ALLOC(svo_gngt_edge_t);
    e->age = 0;

    borNetAddEdge(gng->net, &e->edge, &n1->node, &n2->node);

    return e;
}

void svoGNGTEdgeDel(svo_gngt_t *gng, svo_gngt_edge_t *edge)
{
    borNetRemoveEdge(gng->net, &edge->edge);
    BOR_FREE(edge);
}

void svoGNGTEdgeBetweenDel(svo_gngt_t *gng,
                           svo_gngt_node_t *n1, svo_gngt_node_t *n2)
{
    svo_gngt_edge_t *e;

    if ((e = svoGNGTEdgeBetween(gng, n1, n2)) != NULL)
        svoGNGTEdgeDel(gng, e);
}



static void svoGNGTHebbianLearning(svo_gngt_t *gng,
                                   svo_gngt_node_t *n1,
                                   svo_gngt_node_t *n2)
{
    svo_gngt_edge_t *e;

    e = svoGNGTEdgeBetween(gng, n1, n2);
    if (!e)
        e = svoGNGTEdgeNew(gng, n1, n2);
    e->age = 0;
}

static svo_gngt_node_t *svoGNGTNodeNeighborWithHighestErr(svo_gngt_t *gng,
                                                          svo_gngt_node_t *q)
{
    bor_list_t *list, *item;
    bor_net_edge_t *ne;
    bor_net_node_t *nn;
    svo_gngt_node_t *n, *max;
    bor_real_t err;

    max = NULL;
    err = -BOR_REAL_MAX;

    list = borNetNodeEdges(&q->node);
    BOR_LIST_FOR_EACH(list, item){
        ne = borNetEdgeFromNodeList(item);
        nn = borNetEdgeOtherNode(ne, &q->node);
        n  = bor_container_of(nn, svo_gngt_node_t, node);

        if (n->err > err){
            err = n->err;
            max = n;
        }
    }

    return max;
}


static void nodeFinalDel(bor_net_node_t *node, void *data)
{
    svo_gngt_t *gng = (svo_gngt_t *)data;
    svo_gngt_node_t *n;

    n = bor_container_of(node, svo_gngt_node_t, node);
    gng->ops.del_node(n, gng->ops.del_node_data);
}

static void delEdge(bor_net_edge_t *edge, void *data)
{
    BOR_FREE(edge);
}
