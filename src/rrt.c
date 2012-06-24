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

#include <string.h>
#include <svoboda/rrt.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>

struct _conf_item_t {
    int d;
    bor_vec_t *conf;
    bor_list_t list;
};
typedef struct _conf_item_t conf_item_t;

static conf_item_t *confItemNew(int d, const bor_vec_t *conf);
static void confItemDel(conf_item_t *i);

static svo_rrt_node_t *nodeNew(svo_rrt_t *rrt, const bor_vec_t *p);
//static void nodeDel(svo_rrt_t *rrt, svo_rrt_node_t *n);
static void edgeNew(svo_rrt_t *rrt, svo_rrt_node_t *n1, svo_rrt_node_t *n2);
//static void edgeDel(svo_rrt_t *rrt, bor_net_edge_t *n);
static void nodeNetDel(bor_net_node_t *n, void *);
static void edgeNetDel(bor_net_edge_t *n, void *);
/*
static svo_rrt_node_t *connectNewNode(svo_rrt_t *rrt, const bor_vec_t *c);
static void nodeDelWithEdges(svo_rrt_t *rrt, svo_rrt_node_t *n);
*/

void svoRRTOpsInit(svo_rrt_ops_t *ops)
{
    memset(ops, 0, sizeof(svo_rrt_ops_t));
}

void svoRRTExpandAdd(int dim, const bor_vec_t *conf, bor_list_t *list)
{
    conf_item_t *c;

    c = confItemNew(dim, conf);
    borListAppend(list, &c->list);
}

void svoRRTParamsInit(svo_rrt_params_t *params)
{
    params->dim = 2;
    borNNParamsInit(&params->nn);
    borNNParamsSetDim(&params->nn, 2);
}

svo_rrt_t *svoRRTNew(const svo_rrt_ops_t *ops,
                     const svo_rrt_params_t *params)
{
    svo_rrt_t *rrt;

    rrt = BOR_ALLOC(svo_rrt_t);

    rrt->params = *params;
    borNNParamsSetDim(&rrt->params.nn, rrt->params.dim);

    rrt->ops    = *ops;
    if (rrt->ops.random_data == NULL)
        rrt->ops.random_data = rrt->ops.data;
    if (rrt->ops.nearest_data == NULL)
        rrt->ops.nearest_data = rrt->ops.data;
    if (rrt->ops.expand_data == NULL)
        rrt->ops.expand_data = rrt->ops.data;
    if (rrt->ops.expand_all_data == NULL)
        rrt->ops.expand_all_data = rrt->ops.data;
    if (rrt->ops.terminate_data == NULL)
        rrt->ops.terminate_data = rrt->ops.data;
    if (rrt->ops.terminate_expand_data == NULL)
        rrt->ops.terminate_expand_data = rrt->ops.data;
    if (rrt->ops.filter_blossom_data == NULL)
        rrt->ops.filter_blossom_data = rrt->ops.data;
    if (rrt->ops.callback_data == NULL)
        rrt->ops.callback_data = rrt->ops.data;

    rrt->net = borNetNew();
    rrt->nn = borNNNew(&params->nn);

    rrt->node_init = NULL;
    rrt->node_last = NULL;

    return rrt;
}

void svoRRTDel(svo_rrt_t *rrt)
{
    if (rrt->net)
        borNetDel2(rrt->net, nodeNetDel, (void *)rrt,
                             edgeNetDel, (void *)rrt);

    if (rrt->nn)
        borNNDel(rrt->nn);

    BOR_FREE(rrt);
}

static void svoRRTRunBasicConnect(svo_rrt_t *rrt, const bor_vec_t *init,
                                  int connect)
{
    svo_rrt_node_t *n, *new;
    const svo_rrt_node_t *cnear, *start;
    const bor_vec_t *rand, *new_conf;
    unsigned long counter;

    // create inital node
    n = nodeNew(rrt, init);
    rrt->node_init = rrt->node_last = n;

    counter = 1;
    while (!rrt->ops.terminate(rrt, rrt->ops.terminate_data)){
        // get random configuration
        rand = rrt->ops.random(rrt, rrt->ops.random_data);

        // get nearest node from net
        if (rrt->ops.nearest){
            cnear = rrt->ops.nearest(rrt, rand, rrt->ops.nearest_data);
        }else{
            cnear = svoRRTNearest(rrt, rand);
        }

        start = cnear;

        do {
            // get new configuration
            new_conf = rrt->ops.expand(rrt, cnear, rand, rrt->ops.expand_data);

            new = NULL;
            if (new_conf){
                // add node to net
                new = nodeNew(rrt, new_conf);
                rrt->node_last = new;

                // and connect in with nearest node
                edgeNew(rrt, (svo_rrt_node_t *)cnear, new);
            }

            cnear = new;
        } while (cnear && connect
                    && !rrt->ops.terminate_expand(rrt, start, cnear, rand,
                                                  rrt->ops.terminate_expand_data));


        if (rrt->ops.callback && counter == rrt->ops.callback_period){
            rrt->ops.callback(rrt, rrt->ops.callback_data);
            counter = 0L;
        }
        counter += 1L;
    }
}

void svoRRTRunBasic(svo_rrt_t *rrt, const bor_vec_t *init)
{
    svoRRTRunBasicConnect(rrt, init, 0);
}

void svoRRTRunConnect(svo_rrt_t *rrt, const bor_vec_t *init)
{
    svoRRTRunBasicConnect(rrt, init, 1);
}

void svoRRTRunBlossom(svo_rrt_t *rrt, const bor_vec_t *init)
{
    svo_rrt_node_t *n, *new;
    const svo_rrt_node_t *cnear, *cnear2;
    const bor_vec_t *rand;
    unsigned long counter;
    bor_list_t expand, *item, *item_tmp;
    conf_item_t *conf;

    // create inital node
    n = nodeNew(rrt, init);
    rrt->node_init = rrt->node_last = n;

    // clear expand list
    borListInit(&expand);
    cnear = NULL;

    counter = 1;
    while (!rrt->ops.terminate(rrt, rrt->ops.terminate_data)){
        // expand if expand list is empty
        if (borListEmpty(&expand)){
            // get random configuration
            rand = rrt->ops.random(rrt, rrt->ops.random_data);

            // get nearest node from tree
            if (rrt->ops.nearest){
                cnear = rrt->ops.nearest(rrt, rand, rrt->ops.nearest_data);
            }else{
                cnear = svoRRTNearest(rrt, rand);
            }

            // expand cnear
            borListInit(&expand);
            rrt->ops.expand_all(rrt, cnear, rand, rrt->ops.expand_all_data, &expand);

            // filter configurations if callback is set
            if (rrt->ops.filter_blossom){
                BOR_LIST_FOR_EACH_SAFE(&expand, item, item_tmp){
                    conf = BOR_LIST_ENTRY(item, conf_item_t, list);

                    // find nearest node to expansion
                    cnear2 = NULL;
                    if (rrt->ops.nearest){
                        cnear2 = rrt->ops.nearest(rrt, conf->conf,
                                                  rrt->ops.nearest_data);
                    }else{
                        cnear2 = svoRRTNearest(rrt, conf->conf);
                    }

                    // check configuration by the callback
                    if (!rrt->ops.filter_blossom(rrt, conf->conf, cnear, cnear2,
                                                 rrt->ops.filter_blossom_data)){
                        borListDel(item);
                        confItemDel(conf);
                    }
                }
            }
        }

        // all nodes in expand list are always expanded from cnear node
        if (cnear && !borListEmpty(&expand)){
            // get next configuration
            item = borListNext(&expand);
            conf = BOR_LIST_ENTRY(item, conf_item_t, list);

            // add node to net
            new = nodeNew(rrt, conf->conf);
            rrt->node_last = new;

            // and connect in with nearest node
            edgeNew(rrt, (svo_rrt_node_t *)cnear, new);

            // remove configuration
            borListDel(item);
            confItemDel(conf);
        }


        if (rrt->ops.callback && counter == rrt->ops.callback_period){
            rrt->ops.callback(rrt, rrt->ops.callback_data);
            counter = 0L;
        }
        counter += 1L;
    }
}

const svo_rrt_node_t *svoRRTNodeNew(svo_rrt_t *rrt, const bor_vec_t *conf,
                                    const svo_rrt_node_t *_n)
{
    svo_rrt_node_t *n = (svo_rrt_node_t *)_n;
    svo_rrt_node_t *new;

    new = nodeNew(rrt, conf);
    edgeNew(rrt, new, n);

    return new;
}

const svo_rrt_node_t *svoRRTNearest(const svo_rrt_t *rrt, const bor_vec_t *c)
{
    bor_nn_el_t *el;
    const svo_rrt_node_t *near = NULL;

    if (borNNNearest(rrt->nn, c, 1, &el) == 1){
        near = bor_container_of(el, svo_rrt_node_t, nn);
    }

    return near;
}

void svoRRTDumpSVT(svo_rrt_t *rrt, FILE *out, const char *name)
{
    bor_list_t *list, *item;
    bor_net_node_t *nn;
    svo_rrt_node_t *n;
    bor_net_edge_t *e;
    size_t i, id1, id2;

    if (rrt->params.dim > 3)
        return;

    fprintf(out, "--------\n");

    if (name){
        fprintf(out, "Name: %s\n", name);
    }

    fprintf(out, "Point size: 1\n");
    fprintf(out, "Points:\n");
    list = borNetNodes(rrt->net);
    i = 0;
    BOR_LIST_FOR_EACH(list, item){
        nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n  = bor_container_of(nn, svo_rrt_node_t, node);

        n->_id = i++;
        borVecPrint(rrt->params.dim, n->conf, out);
        fprintf(out, "\n");
    }


    fprintf(out, "Edges:\n");
    list = borNetEdges(rrt->net);
    BOR_LIST_FOR_EACH(list, item){
        e = BOR_LIST_ENTRY(item, bor_net_edge_t, list);

        nn = borNetEdgeNode(e, 0);
        n  = bor_container_of(nn, svo_rrt_node_t, node);
        id1 = n->_id;

        nn = borNetEdgeNode(e, 1);
        n  = bor_container_of(nn, svo_rrt_node_t, node);
        id2 = n->_id;

        fprintf(out, "%d %d\n", (int)id1, (int)id2);
    }

    fprintf(out, "--------\n");
}



/*** Find path ***/
static void findPathExpand(bor_dij_node_t *_n, bor_list_t *expand, void *data)
{
    svo_rrt_t *rrt = (svo_rrt_t *)data;
    bor_list_t *list, *item;
    svo_rrt_node_t *n, *o;
    bor_net_edge_t *edge;
    bor_net_node_t *node;
    bor_real_t dist;

    n = bor_container_of(_n, svo_rrt_node_t, dij);

    list = borNetNodeEdges(&n->node);
    BOR_LIST_FOR_EACH(list, item){
        edge = borNetEdgeFromNodeList(item);
        node = borNetEdgeOtherNode(edge, &n->node);
        o    = bor_container_of(node, svo_rrt_node_t, node);

        if (!borDijNodeClosed(&o->dij)){
            dist = borVecDist(rrt->params.dim, n->conf, o->conf);
            borDijNodeAdd(&o->dij, expand, dist);
        }
    }
}

/** Initializes all nodes in net for dijkstra search */
static void findPathDijInit(svo_rrt_t *rrt)
{
    bor_list_t *list, *item;
    bor_net_node_t *node;
    svo_rrt_node_t *n;

    list = borNetNodes(rrt->net);
    BOR_LIST_FOR_EACH(list, item){
        node = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(node, svo_rrt_node_t, node);
        borDijNodeInit(&n->dij);
    }
}

/** Fills given list by path from s to g.
 *  It is assumed that the path exists! */
static void obtainPath(svo_rrt_node_t *s, svo_rrt_node_t *g,
                       bor_list_t *list)
{
    svo_rrt_node_t *n;
    bor_dij_node_t *dn;

    borListPrepend(list, &g->path);
    dn = g->dij.prev;
    while (dn != &s->dij){
        n = bor_container_of(dn, svo_rrt_node_t, dij);
        borListPrepend(list, &n->path);

        dn = dn->prev;
    }
    n = bor_container_of(dn, svo_rrt_node_t, dij);
    borListPrepend(list, &n->path);
}
int svoRRTFindPath(svo_rrt_t *rrt,
                   const svo_rrt_node_t *_init, const svo_rrt_node_t *_goal,
                   bor_list_t *list)
{
    bor_dij_ops_t ops;
    bor_dij_t *dij;
    svo_rrt_node_t *init, *goal;
    int result;

    init = (svo_rrt_node_t *)_init;
    goal = (svo_rrt_node_t *)_goal;

    // initialize whole net
    findPathDijInit(rrt);

    // initialize operations
    borDijOpsInit(&ops);
    ops.expand = findPathExpand;
    ops.data   = (void *)rrt;

    // create dijkstra algorithm
    dij = borDijNew(&ops);

    // run dijkstra
    result = borDijRun(dij, &init->dij, &goal->dij);

    if (result == 0){
        obtainPath(init, goal, list);
        borDijDel(dij);
        return 0;
    }

    borDijDel(dij);

    return -1;
}

static conf_item_t *confItemNew(int d, const bor_vec_t *conf)
{
    conf_item_t *c;

    c = BOR_ALLOC(conf_item_t);
    c->d = d;
    c->conf = borVecClone(d, conf);
    return c;
}

static void confItemDel(conf_item_t *c)
{
    borVecDel(c->conf);
    BOR_FREE(c);
}


static svo_rrt_node_t *nodeNew(svo_rrt_t *rrt, const bor_vec_t *p)
{
    svo_rrt_node_t *n;

    n = BOR_ALLOC(svo_rrt_node_t);
    n->conf = borVecClone(rrt->params.dim, p);

    borNetAddNode(rrt->net, &n->node);

    borNNElInit(rrt->nn, &n->nn, n->conf);
    borNNAdd(rrt->nn, &n->nn);

    return n;
}

/*
static void nodeDel(svo_rrt_t *rrt, svo_rrt_node_t *n)
{
    borVecDel(n->conf);
    BOR_FREE(n);
}
*/

static void edgeNew(svo_rrt_t *rrt, svo_rrt_node_t *n1, svo_rrt_node_t *n2)
{
    bor_net_edge_t *e;

    e = BOR_ALLOC(bor_net_edge_t);
    borNetAddEdge(rrt->net, e, &n1->node, &n2->node);
}

/*
static void edgeDel(svo_rrt_t *rrt, bor_net_edge_t *e)
{
    BOR_FREE(e);
}
*/

static void nodeNetDel(bor_net_node_t *_n, void *_)
{
    svo_rrt_node_t *n;

    n = bor_container_of(_n, svo_rrt_node_t, node);

    borVecDel(n->conf);
    BOR_FREE(n);
}

static void edgeNetDel(bor_net_edge_t *n, void *_)
{
    BOR_FREE(n);
}
