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
#include <svoboda/prm.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>

static svo_prm_node_t *nodeNew(svo_prm_t *prm, const bor_vec_t *p);
static void nodeDel(svo_prm_t *prm, svo_prm_node_t *n);
static void edgeNew(svo_prm_t *prm, svo_prm_node_t *n1, svo_prm_node_t *n2);
static void edgeDel(svo_prm_t *prm, bor_net_edge_t *n);
static void nodeNetDel(bor_net_node_t *n, void *);
static void edgeNetDel(bor_net_edge_t *n, void *);
static svo_prm_node_t *connectNewNode(svo_prm_t *prm, const bor_vec_t *c);
static void nodeDelWithEdges(svo_prm_t *prm, svo_prm_node_t *n);

/** Finds maximaly prm->params.max_neighbors nearest to given node.
 *  Number of found nodes is returned. */
static size_t findNearest(svo_prm_t *prm, const bor_vec_t *conf,
                          svo_prm_node_t **nearest, bor_gug_el_t **els);
/** Creates new component consisting of one node */
static void componentNew(svo_prm_t *prm, svo_prm_node_t *n);
/** Returns true if two nodes belong to same component */
static int sameComponent(const svo_prm_node_t *n1, const svo_prm_node_t *n2);
/** Returns top component of node */
static svo_prm_component_t *topComponent(const svo_prm_node_t *n);


void svoPRMOpsInit(svo_prm_ops_t *ops)
{
    memset(ops, 0, sizeof(svo_prm_ops_t));
}

void svoPRMParamsInit(svo_prm_params_t *params)
{
    params->d = 2;

    params->max_dist = 0.001;
    params->max_neighbors = 10;

    borGUGParamsInit(&params->gug);
}


svo_prm_t *svoPRMNew(const svo_prm_ops_t *ops,
                     const svo_prm_params_t *params)
{
    svo_prm_t *prm;
    bor_gug_params_t pcells;

    prm = BOR_ALLOC(svo_prm_t);

    prm->params = *params;
    prm->ops    = *ops;
    if (prm->ops.conf_data == NULL)
        prm->ops.conf_data = prm->ops.data;
    if (prm->ops.terminate_data == NULL)
        prm->ops.terminate_data = prm->ops.data;
    if (prm->ops.eval_data == NULL)
        prm->ops.eval_data = prm->ops.data;
    if (prm->ops.find_path_data == NULL)
        prm->ops.find_path_data = prm->ops.data;
    if (prm->ops.callback_data == NULL)
        prm->ops.callback_data = prm->ops.data;

    prm->net = borNetNew();

    pcells   = params->gug;
    pcells.dim = params->d;
    prm->gug = borGUGNew(&pcells);

    borListInit(&prm->components);

    return prm;
}

void svoPRMDel(svo_prm_t *prm)
{
    bor_list_t *item;
    svo_prm_component_t *comp;

    if (prm->net)
        borNetDel2(prm->net,
                   nodeNetDel, (void *)prm,
                   edgeNetDel, (void *)prm);
    if (prm->gug)
        borGUGDel(prm->gug);

    while (!borListEmpty(&prm->components)){
        item = borListNext(&prm->components);
        borListDel(item);
        comp = bor_container_of(item, svo_prm_component_t, list);
        BOR_FREE(comp);
    }

    BOR_FREE(prm);
}

void svoPRMRun(svo_prm_t *prm)
{
    const bor_vec_t *c;
    svo_prm_node_t *cn;
    svo_prm_node_t **nearest;
    bor_gug_el_t **tmp_nearest;
    size_t i, nearest_len;
    unsigned long counter = 1;

    nearest = BOR_ALLOC_ARR(svo_prm_node_t *, prm->params.max_neighbors);
    tmp_nearest = BOR_ALLOC_ARR(bor_gug_el_t *, prm->params.max_neighbors);

    while (!prm->ops.terminate(prm->ops.terminate_data)){
        // obtain random configuration
        c = prm->ops.conf(prm->ops.conf_data);

        // evaluate configuration
        if (prm->ops.eval(c, prm->ops.eval_data) == SVO_PRM_FREE){
            // we have configuration in free space

            // obtain nearest nodes
            nearest_len = findNearest(prm, c, nearest, tmp_nearest);

            // create new node
            cn = nodeNew(prm, c);

            // try to connect to found nodes
            for (i = 0; i < nearest_len; i++){
                // skip those that belongs to same component
                if (sameComponent(nearest[i], cn))
                    continue;

                // check if there is path between nodes
                if (prm->ops.find_path(nearest[i]->conf, cn->conf,
                                       prm->ops.find_path_data)){
                    edgeNew(prm, nearest[i], cn);
                }
            }

            if (cn->comp == NULL){
                componentNew(prm, cn);
            }

            if (counter == prm->ops.callback_period){
                prm->ops.callback(prm->ops.callback_data);
                counter = 0;
            }
            counter++;
        }
    }

    BOR_FREE(nearest);
    BOR_FREE(tmp_nearest);
}

void svoPRMDumpSVT(svo_prm_t *prm, FILE *out, const char *name)
{
    bor_list_t *list, *item;
    bor_net_node_t *nn;
    svo_prm_node_t *n;
    bor_net_edge_t *e;
    size_t i, id1, id2;

    fprintf(out, "--------\n");

    if (name){
        fprintf(out, "Name: %s\n", name);
    }

    fprintf(out, "Points:\n");
    list = borNetNodes(prm->net);
    i = 0;
    BOR_LIST_FOR_EACH(list, item){
        nn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n  = bor_container_of(nn, svo_prm_node_t, node);

        n->_id = i++;
        borVec2Print((const bor_vec2_t *)n->conf, out);
        fprintf(out, "\n");
    }


    fprintf(out, "Edges:\n");
    list = borNetEdges(prm->net);
    BOR_LIST_FOR_EACH(list, item){
        e = BOR_LIST_ENTRY(item, bor_net_edge_t, list);

        nn = borNetEdgeNode(e, 0);
        n  = bor_container_of(nn, svo_prm_node_t, node);
        id1 = n->_id;

        nn = borNetEdgeNode(e, 1);
        n  = bor_container_of(nn, svo_prm_node_t, node);
        id2 = n->_id;

        fprintf(out, "%d %d\n", (int)id1, (int)id2);
    }

    fprintf(out, "--------\n");
}


/*** Find path ***/
static void findPathExpand(bor_dij_node_t *_n, bor_list_t *expand, void *data)
{
    svo_prm_t *prm = (svo_prm_t *)data;
    bor_list_t *list, *item;
    svo_prm_node_t *n, *o;
    bor_net_edge_t *edge;
    bor_net_node_t *node;
    bor_real_t dist;

    n = bor_container_of(_n, svo_prm_node_t, dij);

    list = borNetNodeEdges(&n->node);
    BOR_LIST_FOR_EACH(list, item){
        edge = borNetEdgeFromNodeList(item);
        node = borNetEdgeOtherNode(edge, &n->node);
        o    = bor_container_of(node, svo_prm_node_t, node);

        if (!borDijNodeClosed(&o->dij)){
            dist = borVecDist(prm->params.d, n->conf, o->conf);
            borDijNodeAdd(&o->dij, expand, dist);
        }
    }
}

/** Initializes all nodes in net for dijkstra search */
static void findPathDijInit(svo_prm_t *prm)
{
    bor_list_t *list, *item;
    bor_net_node_t *node;
    svo_prm_node_t *n;

    list = borNetNodes(prm->net);
    BOR_LIST_FOR_EACH(list, item){
        node = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(node, svo_prm_node_t, node);
        borDijNodeInit(&n->dij);
    }
}

/** Fills given list by path from s to g.
 *  It is assumed that the path exists! */
static void obtainPath(svo_prm_node_t *s, svo_prm_node_t *g,
                       bor_list_t *list)
{
    svo_prm_node_t *n;
    bor_dij_node_t *dn;

    borListPrepend(list, &g->path);
    dn = g->dij.prev;
    while (dn != &s->dij){
        n = bor_container_of(dn, svo_prm_node_t, dij);
        borListPrepend(list, &n->path);

        dn = dn->prev;
    }
}

int svoPRMFindPath(svo_prm_t *prm,
                   const bor_vec_t *cstart, const bor_vec_t *cgoal,
                   bor_list_t *list)
{
    bor_dij_ops_t ops;
    bor_dij_t *dij;
    svo_prm_node_t *start, *goal;
    int result;

    // create start and goal nodes
    start = connectNewNode(prm, cstart);
    goal  = connectNewNode(prm, cgoal);

    // initialize whole net
    findPathDijInit(prm);

    // initialize operations
    borDijOpsInit(&ops);
    ops.expand = findPathExpand;
    ops.data   = (void *)prm;

    // create dijkstra algorithm
    dij = borDijNew(&ops);

    // run dijkstra
    result = borDijRun(dij, &start->dij, &goal->dij);

    if (result == 0){
        obtainPath(start, goal, list);
        borDijDel(dij);
        return 0;
    }

    // remove previously created nodes
    nodeDelWithEdges(prm, start);
    nodeDelWithEdges(prm, goal);

    borDijDel(dij);

    return -1;
}



static svo_prm_node_t *nodeNew(svo_prm_t *prm, const bor_vec_t *p)
{
    svo_prm_node_t *n;

    n = BOR_ALLOC(svo_prm_node_t);
    n->conf = borVecClone(prm->params.d, p);
    n->comp = NULL;
    borNetAddNode(prm->net, &n->node);

    borGUGElInit(&n->gug, n->conf);
    borGUGAdd(prm->gug, &n->gug);

    return n;
}

static void nodeDel(svo_prm_t *prm, svo_prm_node_t *n)
{
    if (n->conf)
        borVecDel(n->conf);
    borNetRemoveNode(prm->net, &n->node);
    borGUGRemove(prm->gug, &n->gug);
    BOR_FREE(n);
}

static void edgeNew(svo_prm_t *prm, svo_prm_node_t *n1, svo_prm_node_t *n2)
{
    bor_net_edge_t *e;
    svo_prm_component_t *c1, *c2;

    e = BOR_ALLOC(bor_net_edge_t);
    borNetAddEdge(prm->net, e, &n1->node, &n2->node);

    if (n1->comp == NULL){
        n1->comp = n2->comp;
    }else if (n2->comp == NULL){
        n2->comp = n1->comp;
    }else{
        c1 = topComponent(n1);
        c2 = topComponent(n2);
        c1->parent = c2;
    }
}

static void edgeDel(svo_prm_t *prm, bor_net_edge_t *e)
{
    borNetRemoveEdge(prm->net, e);
    BOR_FREE(e);
}

static void nodeNetDel(bor_net_node_t *_n, void *_)
{
    svo_prm_node_t *n;

    n = bor_container_of(_n, svo_prm_node_t, node);
    if (n->conf)
        borVecDel(n->conf);
    BOR_FREE(n);
}

static void edgeNetDel(bor_net_edge_t *n, void *_)
{
    BOR_FREE(n);
}


static size_t findNearest(svo_prm_t *prm, const bor_vec_t *conf,
                          svo_prm_node_t **nearest, bor_gug_el_t **els)
{
    size_t size, found;
    svo_prm_node_t *m;

    size = borGUGNearest(prm->gug, conf,
                              prm->params.max_neighbors, els);

    for (found = 0; found < size; found++){
        m = bor_container_of(els[found], svo_prm_node_t, gug);

        if (borVecDist(prm->params.d, m->conf, conf) < prm->params.max_dist){
            nearest[found] = m;
        }else{
            break;
        }
    }

    return found;
}

static void componentNew(svo_prm_t *prm, svo_prm_node_t *n)
{
    svo_prm_component_t *comp;

    comp = BOR_ALLOC(svo_prm_component_t);
    comp->parent = NULL;
    borListAppend(&prm->components, &comp->list);

    n->comp = comp;
}

static int sameComponent(const svo_prm_node_t *n1, const svo_prm_node_t *n2)
{
    svo_prm_component_t *c1, *c2;
    if (n1->comp == NULL || n2->comp == NULL)
        return 0;

    c1 = topComponent(n1);
    c2 = topComponent(n2);

    return c1 == c2;
}

static svo_prm_component_t *topComponent(const svo_prm_node_t *n)
{
    svo_prm_component_t *c;

    if (n->comp == NULL)
        return NULL;

    c = n->comp;
    while (c->parent != NULL)
        c = c->parent;

    return c;
}

static svo_prm_node_t *connectNewNode(svo_prm_t *prm, const bor_vec_t *c)
{
    svo_prm_node_t **nearest;
    bor_gug_el_t **tmp_nearest;
    size_t nearest_len, i;
    svo_prm_node_t *n;

    nearest = BOR_ALLOC_ARR(svo_prm_node_t *, prm->params.max_neighbors);
    tmp_nearest = BOR_ALLOC_ARR(bor_gug_el_t *, prm->params.max_neighbors);

    nearest_len = findNearest(prm, c, nearest, tmp_nearest);

    n = nodeNew(prm, c);

    for (i = 0; i < nearest_len; i++){
        // skip those that belongs to same component
        if (sameComponent(nearest[i], n))
            continue;

        // check if there is path between nodes
        if (prm->ops.find_path(nearest[i]->conf, n->conf,
                               prm->ops.find_path_data)){
            edgeNew(prm, nearest[i], n);
        }
    }

    BOR_FREE(nearest);
    BOR_FREE(tmp_nearest);

    return n;
}

static void nodeDelWithEdges(svo_prm_t *prm, svo_prm_node_t *n)
{
    bor_list_t *list, *item, *item_tmp;
    bor_net_edge_t *edge;

    list = borNetNodeEdges(&n->node);
    BOR_LIST_FOR_EACH_SAFE(list, item, item_tmp){
        edge = borNetEdgeFromNodeList(item);
        edgeDel(prm, edge);
    }
    nodeDel(prm, n);
}
