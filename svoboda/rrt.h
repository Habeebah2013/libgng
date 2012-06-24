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

#ifndef __SVO_RRT_H__
#define __SVO_RRT_H__

#include <boruvka/vec.h>
#include <boruvka/net.h>
#include <boruvka/nn.h>
#include <boruvka/dij.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define SVO_RRT_FREE 1
#define SVO_RRT_OBST 2

struct _svo_rrt_ops_t;
struct _svo_rrt_params_t;
struct _svo_rrt_node_t;
struct _svo_rrt_t;

/**
 * RRT - Rapidly-Exploring Random Trees
 * =====================================
 */

/**
 * RRT Operations
 * ---------------
 *
 * See svo_rrt_ops_t.
 */

/** vvvv */

/**
 * Returns random configuration.
 */
typedef const bor_vec_t *(*svo_rrt_random)(const struct _svo_rrt_t *rrt, void *);

/**
 * Returns nearest node to given configuration.
 * Note that returned node must be somehow obtained from {rrt}.
 */
typedef const struct _svo_rrt_node_t *(*svo_rrt_nearest)(const struct _svo_rrt_t *rrt,
                                                         const bor_vec_t *conf,
                                                         void *);

/**
 * Expands node {n} towards configuration {conf}.
 */
typedef const bor_vec_t *(*svo_rrt_expand)(const struct _svo_rrt_t *rrt,
                                           const struct _svo_rrt_node_t *n,
                                           const bor_vec_t *conf,
                                           void *);

/**
 * Expands node {n} towards configuration {conf} - fills list {list_out}
 * with all possible configurations.
 * Use svoRRTExpandAdd() function for adding configurations into
 * {list_out}.
 */
typedef void (*svo_rrt_expand_all)(const struct _svo_rrt_t *rrt,
                                   const struct _svo_rrt_node_t *n,
                                   const bor_vec_t *conf,
                                   void * data,
                                   bor_list_t *list_out);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_rrt_terminate)(const struct _svo_rrt_t *rrt, void *);

/**
 * Return true if expansion chain should be terminated. Start node (where
 * starts expansion), last created node and random configuration are
 * provided.
 * See *RunConnect() for more info.
 */
typedef int (*svo_rrt_terminate_expand)(const struct _svo_rrt_t *rrt,
                                        const struct _svo_rrt_node_t *start,
                                        const struct _svo_rrt_node_t *last,
                                        const bor_vec_t *rand_conf,
                                        void *);

/**
 * Return true if {candidate} should be used for expansion.
 * {src} is node frome which were expansion performed, {nearest} is node
 * nearest to {candidate}.
 */
typedef int (*svo_rrt_filter_blossom)(const struct _svo_rrt_t *rrt,
                                      const bor_vec_t *candidate,
                                      const struct _svo_rrt_node_t *src,
                                      const struct _svo_rrt_node_t *nearest,
                                      void *);
/**
 * Callback that is periodically called from RRT.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_rrt_callback)(const struct _svo_rrt_t *rrt, void *);

/** ^^^^ */

struct _svo_rrt_ops_t {
    svo_rrt_random random;
    svo_rrt_nearest nearest;
    svo_rrt_expand expand;
    svo_rrt_expand_all expand_all;
    svo_rrt_terminate terminate;
    svo_rrt_terminate_expand terminate_expand;
    svo_rrt_filter_blossom filter_blossom;

    svo_rrt_callback callback;
    unsigned long callback_period;

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */

    void *random_data;
    void *nearest_data;
    void *expand_data;
    void *expand_all_data;
    void *terminate_data;
    void *terminate_expand_data;
    void *filter_blossom_data;
    void *callback_data;
};
typedef struct _svo_rrt_ops_t svo_rrt_ops_t;

/**
 * Initializes ops struct to NULL values.
 */
void svoRRTOpsInit(svo_rrt_ops_t *ops);

/**
 * Adds given configuration into list
 */
void svoRRTExpandAdd(int dim, const bor_vec_t *conf, bor_list_t *list);

/**
 * RRT Parameters
 * ---------------
 */
struct _svo_rrt_params_t {
    int dim; /*!< Dimension of problem */

    bor_nn_params_t nn;
};
typedef struct _svo_rrt_params_t svo_rrt_params_t;

/**
 * Initializes params struct to default values.
 */
void svoRRTParamsInit(svo_rrt_params_t *params);


/**
 * RRT Algorithm
 * --------------
 *
 * See svo_rrt_t.
 */

struct _svo_rrt_node_t {
    bor_vec_t *conf;
    bor_net_node_t node;
    bor_nn_el_t nn;

    bor_dij_node_t dij;
    bor_list_t path;

    int _id;
};
typedef struct _svo_rrt_node_t svo_rrt_node_t;

struct _svo_rrt_t {
    svo_rrt_ops_t ops;
    svo_rrt_params_t params;

    bor_net_t *net;
    bor_nn_t *nn;

    svo_rrt_node_t *node_init; /*!< Initial node */
    svo_rrt_node_t *node_last; /*!< Last generated node */
};
typedef struct _svo_rrt_t svo_rrt_t;

/**
 * Creates new instance of algorithm
 */
svo_rrt_t *svoRRTNew(const svo_rrt_ops_t *ops,
                     const svo_rrt_params_t *params);

/**
 * Deletes RRT instance.
 */
void svoRRTDel(svo_rrt_t *rrt);

/**
 * Runs basic algorithm:
 * ~~~~~~
 * while !ops.terminate():
 *     r = ops.random()
 *     n = ops.nearest(r)
 *     e = ops.expand(n, r)
 *     if e != NULL:
 *         create edge between n and e
 */
void svoRRTRunBasic(svo_rrt_t *rrt, const bor_vec_t *init);

/**
 * Runs RRT-Connect:
 * ~~~~~~
 * while !ops.terminate():
 *     r = ops.random()
 *     n = ops.nearest(r)
 *     repeat:
 *         e = ops.expand(n, r)
 *         if e != NULL:
 *             create edge between n and e
 *         n = e
 *     until n != NULL && !ops.terminate_expand(n, r)
 */
void svoRRTRunConnect(svo_rrt_t *rrt, const bor_vec_t *init);

/**
 * Runs RRT-Blossom:
 * ~~~~~
 * while !ops.terminate():
 *     r = ops.random()
 *     n = ops.nearest(r)
 *     E = ops.expand_all(n, r)
 *     for e in E:
 *         m = ops.nearest(e)
 *         if ops.filter_blossom(e, n, m):
 *             create edge between n and e
 */
void svoRRTRunBlossom(svo_rrt_t *rrt, const bor_vec_t *init);

/**
 * Returns number of nodes in roadmap.
 */
_bor_inline size_t svoRRTNodesLen(const svo_rrt_t *rrt);

/**
 * Returns initial node.
 *
 * The one with configuration passed to *Run*() function.
 */
_bor_inline const svo_rrt_node_t *svoRRTNodeInitial(const svo_rrt_t *rrt);

/**
 * Returns last newly created node.
 */
_bor_inline const svo_rrt_node_t *svoRRTNodeLast(const svo_rrt_t *rrt);

/**
 * Creates new node in tree with configuration {conf} and this node will be
 * connected with node {n}. Node {n} must be already in RRT's net.
 */
const svo_rrt_node_t *svoRRTNodeNew(svo_rrt_t *rrt, const bor_vec_t *conf,
                                    const svo_rrt_node_t *n);

/**
 * Returns nearest node to given configuration {c}. Nearest node is
 * meassured in euclidean distance metric.
 * This function is used if ops.nearest is set to NULL.
 */
const svo_rrt_node_t *svoRRTNearest(const svo_rrt_t *rrt, const bor_vec_t *c);

/**
 * Tries to find path in net from init to goal.
 * If path was found 0 is returned and argument list is filled by nodes
 * representing path. Nodes are connected into this list by member .path.
 * If path wasn't found -1 is returned.
 */
int svoRRTFindPath(svo_rrt_t *rrt,
                   const svo_rrt_node_t *init, const svo_rrt_node_t *goal,
                   bor_list_t *list);

/**
 * Dumps net in SVT format.
 */
void svoRRTDumpSVT(svo_rrt_t *rrt, FILE *out, const char *name);

/**
 * Node Functions
 * ---------------
 *
 * See svo_rrt_node_t.
 */

/**
 * Returns configuration (state) of node.
 */
_bor_inline const bor_vec_t *svoRRTNodeConf(const svo_rrt_node_t *n);


/**** INLINES ****/
_bor_inline size_t svoRRTNodesLen(const svo_rrt_t *rrt)
{
    return borNetNodesLen(rrt->net);
}

_bor_inline const svo_rrt_node_t *svoRRTNodeInitial(const svo_rrt_t *rrt)
{
    return rrt->node_init;
}

_bor_inline const svo_rrt_node_t *svoRRTNodeLast(const svo_rrt_t *rrt)
{
    return rrt->node_last;
}


_bor_inline const bor_vec_t *svoRRTNodeConf(const svo_rrt_node_t *n)
{
    return n->conf;
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_RRT_H__ */
