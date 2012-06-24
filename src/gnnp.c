/***
 * Svoboda
 * --------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
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

#include <limits.h>
#include <svoboda/gnnp.h>
#include <boruvka/sort.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>


#define IS_FIXED(n) ((n)->fixed != 0)
#define IS_FREE(n) ((n)->fixed == 1)
#define IS_OBST(n) ((n)->fixed == 2)
#define SET_FREE(n) (n)->fixed = 1
#define SET_OBST(n) (n)->fixed = 2

#define PATH_NONE 0
#define PATH_INIT 1
#define PATH_GOAL 2

#define PATH_IS_NONE(n) ((n)->prev_type == PATH_NONE)
#define PATH_IS_NONE_TYPE(type) ((type) == PATH_NONE)
#define PATH_IS_INIT(n) ((n)->prev_type == PATH_INIT)
#define PATH_IS_GOAL(n) ((n)->prev_type == PATH_GOAL)
#define PATH_EQ(n1, n2) ((n1)->prev_type == (n2)->prev_type)
#define PATH_EQ2(n1, type) ((n1)->prev_type == type)
#define PATH_SET_NONE(nn, n) svoGNNPNodeSetPathType((nn), (n), PATH_NONE)
#define PATH_SET_INIT(nn, n) svoGNNPNodeSetPathType((nn), (n), PATH_INIT)
#define PATH_SET_GOAL(nn, n) svoGNNPNodeSetPathType((nn), (n), PATH_GOAL)
#define PATH_COPY(nn, dst, src) svoGNNPNodeSetPathType((nn), (dst), (src)->prev_type)

#define __PATH_SET(n, type) (n)->prev_type = type

static void netNodeDel(bor_net_node_t *n, void *);
static void netEdgeDel(bor_net_edge_t *e, void *);

static svo_gnnp_node_t *svoGNNPNodeNew(svo_gnnp_t *nn, const bor_vec_t *w);
static void svoGNNPNodeDel(svo_gnnp_t *nn, svo_gnnp_node_t *node);
/** Removes the longest edges incidenting with the node */
static void svoGNNPNodeRemoveLongestEdge(svo_gnnp_t *nn, svo_gnnp_node_t *node);
/** Moves the node towards input signal: w = w + e * (is - w) */
static void svoGNNPNodeMoveTowards(svo_gnnp_t *nn, svo_gnnp_node_t *node,
                                   const bor_vec_t *is, bor_real_t r);
/** Changes node's path type */
_bor_inline void svoGNNPNodeSetPathType(svo_gnnp_t *nn, svo_gnnp_node_t *n,
                                        int type);

static int init(svo_gnnp_t *nn, const bor_vec_t *init, const bor_vec_t *goal,
                bor_list_t *path);
static void nearest(svo_gnnp_t *nn, const bor_vec_t *is,
                    svo_gnnp_node_t **n1,
                    svo_gnnp_node_t **n2);
static svo_gnnp_node_t *nearestPath(svo_gnnp_t *nn, const bor_vec_t *is);
static int hebbianLearning(svo_gnnp_t *nn,
                           svo_gnnp_node_t *n1, svo_gnnp_node_t *n2,
                           bor_list_t *path);
static svo_gnnp_node_t *newNode(svo_gnnp_t *nn, svo_gnnp_node_t *wn,
                                const bor_vec_t *is);
static void move(svo_gnnp_t *nn, svo_gnnp_node_t *wn, const bor_vec_t *is);

static int learnPath(svo_gnnp_t *nn, svo_gnnp_node_t *n1,
                                     bor_list_t *path);
static void obtainPath(svo_gnnp_t *nn,
                       svo_gnnp_node_t *n1, svo_gnnp_node_t *n2,
                       bor_list_t *path);
static int prunePath(svo_gnnp_t *nn, bor_list_t *path);


_bor_inline void pathSetNone(svo_gnnp_t *nn, svo_gnnp_node_t *n);
_bor_inline void pathConnect(svo_gnnp_t *nn,
                             svo_gnnp_node_t *from,
                             svo_gnnp_node_t *to,
                             svo_gnnp_node_t **ig1,
                             svo_gnnp_node_t **ig2);
static void pathRetype(svo_gnnp_t *nn, svo_gnnp_node_t *root,
                       svo_gnnp_node_t **ig1,
                       svo_gnnp_node_t **ig2);


#define OPS_DATA(name) \
    if (!nn->ops.name ## _data) \
        nn->ops.name ## _data = nn->ops.data;

#define OPS_CHECK(name) \
    if (!nn->ops.name){ \
        fprintf(stderr, "Fermat :: GNNP :: No " #name " callback set.\n"); \
        exit(-1); \
    }

#define OPS_CHECK_DATA(name) \
    OPS_DATA(name) \
    OPS_CHECK(name)


void svoGNNPParamsInit(svo_gnnp_params_t *p)
{
    p->dim  = 2;
    p->ew   = 0.05;
    p->en   = 0.0005;
    p->rmax = 4;
    p->h    = 0.1;

    borNNParamsInit(&p->nn);
    p->nn.gug.dim    = 2;
    p->nn.vptree.dim = 2;
    p->nn.linear.dim = 2;
}

void svoGNNPOpsInit(svo_gnnp_ops_t *ops)
{
    bzero(ops, sizeof(*ops));
}

svo_gnnp_t *svoGNNPNew(const svo_gnnp_ops_t *ops, const svo_gnnp_params_t *p)
{
    svo_gnnp_t *nn;

    nn = BOR_ALLOC(svo_gnnp_t);
    nn->params = *p;
    nn->params.h *= nn->params.h;

    nn->ops    = *ops;
    OPS_CHECK_DATA(input_signal)
    OPS_CHECK_DATA(terminate)
    OPS_CHECK_DATA(eval)
    OPS_DATA(callback)

    nn->net = borNetNew();
    nn->nn  = borNNNew(&nn->params.nn);
    nn->nn_path = borNNNew(&nn->params.nn);

    nn->init = nn->goal = NULL;
    nn->tmpv = borVecNew(nn->params.dim);

    return nn;
}

void svoGNNPDel(svo_gnnp_t *nn)
{
    borNetDel2(nn->net, netNodeDel, (void *)nn, netEdgeDel, (void *)nn);
    borNNDel(nn->nn);
    borNNDel(nn->nn_path);

    borVecDel(nn->tmpv);
    BOR_FREE(nn);
}

int svoGNNPFindPath(svo_gnnp_t *nn,
                    const bor_vec_t *s, const bor_vec_t *goal,
                    bor_list_t *path)
{
    const bor_vec_t *is;
    svo_gnnp_node_t *n[2], *np;
    bor_real_t dist;
    unsigned int cb = 0U;
    unsigned long c = 0UL;

    // Init
    if (init(nn, s, goal, path) == 1)
        return 0;

    while (!nn->ops.terminate(nn, nn->ops.terminate_data)){
        cb += 1U;

        // get input signal
        is = nn->ops.input_signal(nn, nn->ops.input_signal_data);

        // find two nearest nodes
        nearest(nn, is, &n[0], &n[1]);

        // competitive hebbian learning with removing edges
        if (hebbianLearning(nn, n[0], n[1], path) == 0){
            if (!prunePath(nn, path))
                return 0;
        }

        // learn path
        np = nearestPath(nn, is);
        if (learnPath(nn, np, path) == 0){
            // prune the path if found
            if (!prunePath(nn, path))
                return 0;
        }

        if (IS_FREE(n[0])){
            // the nearest node is fixed (free or obstacle)
            // compute distance between input signal and the winner node
            dist = borVecDist2(nn->params.dim, is, n[0]->w);

            // if input signal is nearest than a resolution, create a new
            // node
            if (dist > nn->params.h){
                newNode(nn, n[0], is);
            }
        }else{
            // move winner node and its neighborhood towards input signal
            move(nn, n[0], is);
        }

        if (nn->ops.callback && nn->ops.callback_period == cb){
            nn->ops.callback(nn, nn->ops.callback_data);
            cb = 0L;
        }

        c += 1UL;
    }

    return -1;
}

static void dumpNodes(const svo_gnnp_t *nn, FILE *out, int type)
{
    bor_list_t *list, *item;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n;

    fprintf(out, "---\n");

    fprintf(out, "Name: Nodes %d\n", type);
    if (type == 0)
        fprintf(out, "Point color: 0.1 0.1 0.1\n");
    if (type == 1)
        fprintf(out, "Point color: 0.1 0.1 0.8\n");
    if (type == 2)
        fprintf(out, "Point color: 0.8 0.1 0.1\n");
    fprintf(out, "Point size: 1\n");
    fprintf(out, "Points:\n");
    list = borNetNodes(nn->net);
    BOR_LIST_FOR_EACH(list, item){
        netn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);
        if (n->fixed == type){
            borVecPrint(nn->params.dim, n->w, out);
            fprintf(out, "\n");
        }
    }

    fprintf(out, "---\n");
}

static void dumpPath(const svo_gnnp_t *nn, FILE *out, int type)
{
    bor_list_t *list, *item;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n;

    fprintf(out, "---\n");

    fprintf(out, "Name: paths %d\n", type);
    if (type == 1)
        fprintf(out, "Point color: 0.8 0.1 0.8\n");
    if (type == 2)
        fprintf(out, "Point color: 0.1 0.8 0.8\n");
    fprintf(out, "Point size: 3\n");
    fprintf(out, "Points:\n");
    list = borNetNodes(nn->net);
    BOR_LIST_FOR_EACH(list, item){
        netn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);
        if (PATH_EQ2(n, type)){
            borVecPrint(nn->params.dim, n->w, out);
            fprintf(out, "\n");
        }
    }

    fprintf(out, "---\n");
}

static void dumpNet(const svo_gnnp_t *nn, FILE *out)
{
    bor_list_t *list, *item;
    size_t i;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n;
    bor_net_edge_t *e;
    size_t id1, id2;

    fprintf(out, "----\n");
    fprintf(out, "Name: net\n");

    fprintf(out, "Points off: 1\n");
    fprintf(out, "Edge color: 0.5 0.5 0.5\n");
    fprintf(out, "Edge width: 1\n");
    fprintf(out, "Points:\n");
    list = borNetNodes(nn->net);
    i = 0;
    BOR_LIST_FOR_EACH(list, item){
        netn = BOR_LIST_ENTRY(item, bor_net_node_t, list);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);
        n->_id = i++;
        borVecPrint(nn->params.dim, n->w, out);
        fprintf(out, "\n");
    }


    fprintf(out, "Edges:\n");
    list = borNetEdges(nn->net);
    BOR_LIST_FOR_EACH(list, item){
        e = BOR_LIST_ENTRY(item, bor_net_edge_t, list);

        netn = borNetEdgeNode(e, 0);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);
        id1 = n->_id;

        netn = borNetEdgeNode(e, 1);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);
        id2 = n->_id;
        fprintf(out, "%d %d\n", (int)id1, (int)id2);
    }

    fprintf(out, "--------\n");
}

void svoGNNPDumpSVT(const svo_gnnp_t *nn, FILE *out, const char *name)
{
    if (nn->params.dim != 2 && nn->params.dim != 3)
        return;

    dumpPath(nn, out, 1);
    dumpPath(nn, out, 2);
    dumpNodes(nn, out, 0);
    dumpNodes(nn, out, 1);
    dumpNodes(nn, out, 2);
    //dumpNet(nn, out);
}

static void netNodeDel(bor_net_node_t *n, void *_nn)
{
    svo_gnnp_t *nn = (svo_gnnp_t *)_nn;
    svo_gnnp_node_t *node = bor_container_of(n, svo_gnnp_node_t, net);
    svoGNNPNodeDel(nn, node);
}

static void netEdgeDel(bor_net_edge_t *e, void *_nn)
{
    borNetEdgeDel(e);
}

static svo_gnnp_node_t *svoGNNPNodeNew(svo_gnnp_t *nn, const bor_vec_t *w)
{
    svo_gnnp_node_t *n;

    n = BOR_ALLOC(svo_gnnp_node_t);

    n->fixed = 0;
    borListInit(&n->path);
    n->prev = NULL;

    n->w = borVecClone(nn->params.dim, w);

    borNetAddNode(nn->net, &n->net);

    borNNElInit(nn->nn, &n->nn, n->w);
    borNNAdd(nn->nn, &n->nn);

    n->prev_type = PATH_NONE;
    borNNElInit(nn->nn_path, &n->nn_path, n->w);

    return n;
}

static void svoGNNPNodeDel(svo_gnnp_t *nn, svo_gnnp_node_t *node)
{
    borNetRemoveNode(nn->net, &node->net);
    borNNRemove(nn->nn, &node->nn);
    borVecDel(node->w);
    BOR_FREE(node);

    /* Nodes are deleted only at the end of the run, so no need to remove
     * it from nn->nodes[] array */
}

static void svoGNNPNodeRemoveLongestEdge(svo_gnnp_t *nn, svo_gnnp_node_t *node)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e, *maxe;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n2;
    bor_real_t len, max;

    max  = -BOR_REAL_MAX;
    maxe = NULL;

    list = borNetNodeEdges(&node->net);
    BOR_LIST_FOR_EACH(list, item){
        e = borNetEdgeFromNodeList(item);
        netn = borNetEdgeOtherNode(e, &node->net);
        n2   = bor_container_of(netn, svo_gnnp_node_t, net);

        len = borVecDist2(nn->params.dim, node->w, n2->w);
        if (len > max){
            max  = len;
            maxe = e;
        }
    }

    if (maxe){
        netn = borNetEdgeOtherNode(maxe, &node->net);
        n2   = bor_container_of(netn, svo_gnnp_node_t, net);

        if (node->prev == n2){
            pathSetNone(nn, node);
        }
        if (n2->prev == node){
            pathSetNone(nn, n2);
        }

        borNetRemoveEdge(nn->net, maxe);
        borNetEdgeDel(maxe);
    }

}

static void svoGNNPNodeMoveTowards(svo_gnnp_t *nn, svo_gnnp_node_t *node,
                                   const bor_vec_t *is, bor_real_t r)
{
    borVecSub2(nn->params.dim, nn->tmpv, is, node->w);
    borVecScale(nn->params.dim, nn->tmpv, r);
    borVecAdd(nn->params.dim, node->w, nn->tmpv);
    borNNUpdate(nn->nn, &node->nn);
    if (!PATH_IS_NONE(node))
        borNNUpdate(nn->nn_path, &node->nn_path);
}

_bor_inline void svoGNNPNodeSetPathType(svo_gnnp_t *nn, svo_gnnp_node_t *n,
                                        int type)
{
    if (!PATH_EQ2(n, type)){
        if (PATH_IS_NONE(n)){
            borNNAdd(nn->nn_path, &n->nn_path);
        }else if (PATH_IS_NONE_TYPE(type)){
            borNNRemove(nn->nn_path, &n->nn_path);
        }
        __PATH_SET(n, type);
    }
}

static int init(svo_gnnp_t *nn, const bor_vec_t *init, const bor_vec_t *goal,
                bor_list_t *path)
{
    bor_net_edge_t *e;

    nn->init = svoGNNPNodeNew(nn, init);
    SET_FREE(nn->init);
    nn->goal = svoGNNPNodeNew(nn, goal);
    SET_FREE(nn->goal);

    PATH_SET_INIT(nn, nn->init);
    PATH_SET_GOAL(nn, nn->goal);

    e = borNetEdgeNew();
    borNetAddEdge(nn->net, e, &nn->init->net, &nn->goal->net);

    borListInit(path);
    borListAppend(path, &nn->init->path);
    borListAppend(path, &nn->goal->path);
    if (!prunePath(nn, path))
        return 1;
    return 0;
}

static void nearest(svo_gnnp_t *nn, const bor_vec_t *is,
                    svo_gnnp_node_t **n1,
                    svo_gnnp_node_t **n2)
{
    bor_nn_el_t *els[2];

    borNNNearest(nn->nn, is, 2, els);
    *n1 = bor_container_of(els[0], svo_gnnp_node_t, nn);
    *n2 = bor_container_of(els[1], svo_gnnp_node_t, nn);
}

static svo_gnnp_node_t *nearestPath(svo_gnnp_t *nn, const bor_vec_t *is)
{
    bor_nn_el_t *els;
    svo_gnnp_node_t *np;

    borNNNearest(nn->nn_path, is, 1, &els);
    np = bor_container_of(els, svo_gnnp_node_t, nn_path);

    return np;
}

static void obtainPath(svo_gnnp_t *nn,
                       svo_gnnp_node_t *n1, svo_gnnp_node_t *n2,
                       bor_list_t *path)
{
    svo_gnnp_node_t *o;

    // obtain path
    borListInit(path);

    // first init node
    o = n2;
    if (PATH_IS_INIT(n1))
        o = n1;
    while (o != nn->init){
        borListPrepend(path, &o->path);
        o = o->prev;
    }
    borListPrepend(path, &nn->init->path);

    // goal node
    o = n2;
    if (PATH_IS_GOAL(n1))
        o = n1;
    while (o != nn->goal){
        borListAppend(path, &o->path);
        o = o->prev;
    }
    borListAppend(path, &nn->goal->path);
}

static int learnPath(svo_gnnp_t *nn, svo_gnnp_node_t *wn,
                                     bor_list_t *path)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e;
    bor_net_node_t *netn;
    svo_gnnp_node_t *o;
    svo_gnnp_node_t *ig[2];


    ig[0] = ig[1] = NULL;

    /** wn is not OBST and also non-PATH_NONE */

    list = borNetNodeEdges(&wn->net);
    BOR_LIST_FOR_EACH(list, item){
        e = borNetEdgeFromNodeList(item);
        netn = borNetEdgeOtherNode(e, &wn->net);
        o    = bor_container_of(netn, svo_gnnp_node_t, net);

        if (IS_OBST(o))
            continue;

        if (PATH_IS_NONE(o)
                || (o != nn->init && o != nn->goal && o->prev == NULL)){
            // {o} is not connected init neither to goal node
            pathConnect(nn, o, wn, &ig[0], &ig[1]);
        }else if (!PATH_EQ(o, wn)){
            // we found a connection between init and goal node
            ig[0] = wn;
            ig[1] = o;
        }
    }

    // it is enough to check ig[0] because ig[1] is always non-NULL iff
    // ig[0] is non-NULL
    if (ig[0]){
        obtainPath(nn, ig[0], ig[1], path);
        return 0;
    }

    return -1;
}

static int hebbianLearning(svo_gnnp_t *nn,
                           svo_gnnp_node_t *n1, svo_gnnp_node_t *n2,
                           bor_list_t *path)
{
    bor_net_edge_t *e;

    // get common edge
    e = borNetNodeCommonEdge(&n1->net, &n2->net);

    if (e == NULL){
        // remove longest edge if adding a new edge would exceeds the limit
        if (borNetNodeEdgesLen(&n1->net) >= nn->params.rmax){
            svoGNNPNodeRemoveLongestEdge(nn, n1);
        }
        if (borNetNodeEdgesLen(&n2->net) >= nn->params.rmax){
            svoGNNPNodeRemoveLongestEdge(nn, n2);
        }

        // add new edge
        e = borNetEdgeNew();
        borNetAddEdge(nn->net, e, &n1->net, &n2->net);
    }

    if (!PATH_IS_NONE(n1) && !PATH_IS_NONE(n2) && !PATH_EQ(n1, n2)){
        obtainPath(nn, n1, n2, path);
        return 0;
    }

    return 1;
}

static svo_gnnp_node_t *newNode(svo_gnnp_t *nn, svo_gnnp_node_t *wn,
                                const bor_vec_t *is)
{
    svo_gnnp_node_t *n;
    bor_net_edge_t *e;

    n = svoGNNPNodeNew(nn, is);

    e = borNetEdgeNew();
    borNetAddEdge(nn->net, e, &wn->net, &n->net);

    if (IS_FREE(wn))
        pathConnect(nn, n, wn, NULL, NULL);

    return n;
}

static void move(svo_gnnp_t *nn, svo_gnnp_node_t *wn, const bor_vec_t *is)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n;

    // move winner node
    if (!IS_FIXED(wn))
        svoGNNPNodeMoveTowards(nn, wn, is, nn->params.ew);

    // move neighbor nodes
    list = borNetNodeEdges(&wn->net);
    BOR_LIST_FOR_EACH(list, item){
        e = borNetEdgeFromNodeList(item);
        netn = borNetEdgeOtherNode(e, &wn->net);
        n    = bor_container_of(netn, svo_gnnp_node_t, net);

        if (!IS_FIXED(n))
            svoGNNPNodeMoveTowards(nn, n, is, nn->params.en);
    }
}


static int _pruneEval(svo_gnnp_t *nn, svo_gnnp_node_t *n)
{
    int eval;

    if (IS_FREE(n)){
        return 0;
    }else if (IS_OBST(n)){
        return 1;
    }

    eval = nn->ops.eval(nn, n->w, nn->ops.eval_data);
    if (eval){
        SET_FREE(n);
        return 0;
    }else{
        SET_OBST(n);
        return 1;
    }
}

static int _pruneBetween(svo_gnnp_t *nn,
                         svo_gnnp_node_t *n1, svo_gnnp_node_t *n2,
                         bor_list_t *path)
{
    svo_gnnp_node_t *n;
    bor_net_edge_t *e;
    bor_real_t dist;
    int ret = 0;

    dist = borVecDist2(nn->params.dim, n1->w, n2->w);
    if (dist < nn->params.h)
        return 0;

    // create new node half way between n1 and n2
    borVecAdd2(nn->params.dim, nn->tmpv, n1->w, n2->w);
    borVecScale(nn->params.dim, nn->tmpv, BOR_REAL(0.5));
    n = svoGNNPNodeNew(nn, nn->tmpv);

    // remove an old edge
    e = borNetNodeCommonEdge(&n1->net, &n2->net);
    borNetRemoveEdge(nn->net, e);
    borNetEdgeDel(e);

    // create two new edges n1-n and n-n2
    e = borNetEdgeNew();
    borNetAddEdge(nn->net, e, &n1->net, &n->net);
    e = borNetEdgeNew();
    borNetAddEdge(nn->net, e, &n->net, &n2->net);

    // evaluate node
    /*
    if (_pruneEval(nn, n))
        return 1;
    if (_pruneBetween(nn, n1, n))
        return 1;
    if (_pruneBetween(nn, n, n2))
        return 1;
    return 0;
    */
    ret |= _pruneEval(nn, n);

    if (dist * BOR_REAL(0.5) < nn->params.h){
        borListAppend(path, &n->path);
        return ret;
    }

    ret |= _pruneBetween(nn, n1, n, path);
    borListAppend(path, &n->path);
    ret |= _pruneBetween(nn, n, n2, path);
    return ret;
}

static void _prunePath(svo_gnnp_t *nn, bor_list_t *path)
{
    bor_list_t *item;
    svo_gnnp_node_t *n, *p;
    svo_gnnp_node_t *reset_f = NULL, *reset_t = NULL;

    n = NULL;
    BOR_LIST_FOR_EACH(path, item){
        p = n;
        n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);

        if (!IS_FREE(n)){
            reset_f = n;
            break;
        }

        if (p){
            pathConnect(nn, n, p, NULL, NULL);
        }
    }


    n = NULL;
    item = borListPrev(path);
    while (item != path){
        p = n;
        n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);

        if (!IS_FREE(n)){
            reset_t = n;
            break;
        }

        if (p){
            pathConnect(nn, n, p, NULL, NULL);
        }

        item = borListPrev(item);
    }

    if (reset_t == NULL)
        return;

    while (reset_f != reset_t){
        pathSetNone(nn, reset_f);
        if (IS_OBST(reset_f))
            reset_f->prev = NULL;
        item = borListNext(&reset_f->path);
        reset_f = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);
    }
    pathSetNone(nn, reset_f);
}

static int prunePath(svo_gnnp_t *nn, bor_list_t *path)
{
    bor_list_t *item, prune_path;
    svo_gnnp_node_t *n1, *n2;
    int ret = 0;

    prune_path = *path;
    prune_path.next->prev = &prune_path;
    prune_path.prev->next = &prune_path;

    borListInit(path);

    n2 = NULL;
    n1 = NULL;
    while (!borListEmpty(&prune_path)){
        n1 = n2;
        item = borListNext(&prune_path);
        borListDel(item);
        n2 = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);

        /*
        if (_pruneEval(nn, n2))
            return 1;

        if (n1 && n2){
            if (_pruneBetween(nn, n1, n2))
                return 1;
        }
        */
        ret |= _pruneEval(nn, n2);

        if (n1 && n2){
            ret |= _pruneBetween(nn, n1, n2, path);
        }

        borListAppend(path, &n2->path);
    }

    if (ret){
        _prunePath(nn, path);
    }

    //return 0;
    return ret;
}

_bor_inline void pathSetNone(svo_gnnp_t *nn, svo_gnnp_node_t *n)
{
    PATH_SET_NONE(nn, n);
    pathRetype(nn, n, NULL, NULL);
}

_bor_inline void pathConnect(svo_gnnp_t *nn,
                             svo_gnnp_node_t *from,
                             svo_gnnp_node_t *to,
                             svo_gnnp_node_t **ig1,
                             svo_gnnp_node_t **ig2)
{
    from->prev = to;
    PATH_COPY(nn, from, to);
    pathRetype(nn, from, ig1, ig2);
}

static void pathRetype(svo_gnnp_t *nn, svo_gnnp_node_t *root,
                       svo_gnnp_node_t **ig1,
                       svo_gnnp_node_t **ig2)
{
    bor_list_t *list, *item;
    bor_net_edge_t *e;
    bor_net_node_t *netn;
    svo_gnnp_node_t *n2;

    list = borNetNodeEdges(&root->net);
    BOR_LIST_FOR_EACH(list, item){
        e = borNetEdgeFromNodeList(item);
        netn = borNetEdgeOtherNode(e, &root->net);
        n2   = bor_container_of(netn, svo_gnnp_node_t, net);

        if (n2->prev == root && !PATH_EQ(n2, root)){
            PATH_COPY(nn, n2, root);
            pathRetype(nn, n2, ig1, ig2);
        }

        if (ig1 && ig2
                && !PATH_IS_NONE(root)
                && !PATH_IS_NONE(n2)
                && n2->prev_type != root->prev_type){
            *ig1 = root;
            *ig2 = n2;
        }
    }
}
