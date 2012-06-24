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

#ifndef __FER_RRT_H__
#define __FER_RRT_H__

#include <fermat/vec.h>
#include <fermat/net.h>
#include <fermat/nn.h>
#include <fermat/dij.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define FER_RRT_FREE 1
#define FER_RRT_OBST 2

struct _fer_rrt_ops_t;
struct _fer_rrt_params_t;
struct _fer_rrt_node_t;
struct _fer_rrt_t;

/**
 * RRT - Rapidly-Exploring Random Trees
 * =====================================
 */

/**
 * RRT Operations
 * ---------------
 *
 * See fer_rrt_ops_t.
 */

/** vvvv */

/**
 * Returns random configuration.
 */
typedef const fer_vec_t *(*fer_rrt_random)(const struct _fer_rrt_t *rrt, void *);

/**
 * Returns nearest node to given configuration.
 * Note that returned node must be somehow obtained from {rrt}.
 */
typedef const struct _fer_rrt_node_t *(*fer_rrt_nearest)(const struct _fer_rrt_t *rrt,
                                                         const fer_vec_t *conf,
                                                         void *);

/**
 * Expands node {n} towards configuration {conf}.
 */
typedef const fer_vec_t *(*fer_rrt_expand)(const struct _fer_rrt_t *rrt,
                                           const struct _fer_rrt_node_t *n,
                                           const fer_vec_t *conf,
                                           void *);

/**
 * Expands node {n} towards configuration {conf} - fills list {list_out}
 * with all possible configurations.
 * Use ferRRTExpandAdd() function for adding configurations into
 * {list_out}.
 */
typedef void (*fer_rrt_expand_all)(const struct _fer_rrt_t *rrt,
                                   const struct _fer_rrt_node_t *n,
                                   const fer_vec_t *conf,
                                   void * data,
                                   fer_list_t *list_out);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*fer_rrt_terminate)(const struct _fer_rrt_t *rrt, void *);

/**
 * Return true if expansion chain should be terminated. Start node (where
 * starts expansion), last created node and random configuration are
 * provided.
 * See *RunConnect() for more info.
 */
typedef int (*fer_rrt_terminate_expand)(const struct _fer_rrt_t *rrt,
                                        const struct _fer_rrt_node_t *start,
                                        const struct _fer_rrt_node_t *last,
                                        const fer_vec_t *rand_conf,
                                        void *);

/**
 * Return true if {candidate} should be used for expansion.
 * {src} is node frome which were expansion performed, {nearest} is node
 * nearest to {candidate}.
 */
typedef int (*fer_rrt_filter_blossom)(const struct _fer_rrt_t *rrt,
                                      const fer_vec_t *candidate,
                                      const struct _fer_rrt_node_t *src,
                                      const struct _fer_rrt_node_t *nearest,
                                      void *);
/**
 * Callback that is periodically called from RRT.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*fer_rrt_callback)(const struct _fer_rrt_t *rrt, void *);

/** ^^^^ */

struct _fer_rrt_ops_t {
    fer_rrt_random random;
    fer_rrt_nearest nearest;
    fer_rrt_expand expand;
    fer_rrt_expand_all expand_all;
    fer_rrt_terminate terminate;
    fer_rrt_terminate_expand terminate_expand;
    fer_rrt_filter_blossom filter_blossom;

    fer_rrt_callback callback;
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
typedef struct _fer_rrt_ops_t fer_rrt_ops_t;

/**
 * Initializes ops struct to NULL values.
 */
void ferRRTOpsInit(fer_rrt_ops_t *ops);

/**
 * Adds given configuration into list
 */
void ferRRTExpandAdd(int dim, const fer_vec_t *conf, fer_list_t *list);

/**
 * RRT Parameters
 * ---------------
 */
struct _fer_rrt_params_t {
    int dim; /*!< Dimension of problem */

    fer_nn_params_t nn;
};
typedef struct _fer_rrt_params_t fer_rrt_params_t;

/**
 * Initializes params struct to default values.
 */
void ferRRTParamsInit(fer_rrt_params_t *params);


/**
 * RRT Algorithm
 * --------------
 *
 * See fer_rrt_t.
 */

struct _fer_rrt_node_t {
    fer_vec_t *conf;
    fer_net_node_t node;
    fer_nn_el_t nn;

    fer_dij_node_t dij;
    fer_list_t path;

    int _id;
};
typedef struct _fer_rrt_node_t fer_rrt_node_t;

struct _fer_rrt_t {
    fer_rrt_ops_t ops;
    fer_rrt_params_t params;

    fer_net_t *net;
    fer_nn_t *nn;

    fer_rrt_node_t *node_init; /*!< Initial node */
    fer_rrt_node_t *node_last; /*!< Last generated node */
};
typedef struct _fer_rrt_t fer_rrt_t;

/**
 * Creates new instance of algorithm
 */
fer_rrt_t *ferRRTNew(const fer_rrt_ops_t *ops,
                     const fer_rrt_params_t *params);

/**
 * Deletes RRT instance.
 */
void ferRRTDel(fer_rrt_t *rrt);

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
void ferRRTRunBasic(fer_rrt_t *rrt, const fer_vec_t *init);

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
void ferRRTRunConnect(fer_rrt_t *rrt, const fer_vec_t *init);

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
void ferRRTRunBlossom(fer_rrt_t *rrt, const fer_vec_t *init);

/**
 * Returns number of nodes in roadmap.
 */
_fer_inline size_t ferRRTNodesLen(const fer_rrt_t *rrt);

/**
 * Returns initial node.
 *
 * The one with configuration passed to *Run*() function.
 */
_fer_inline const fer_rrt_node_t *ferRRTNodeInitial(const fer_rrt_t *rrt);

/**
 * Returns last newly created node.
 */
_fer_inline const fer_rrt_node_t *ferRRTNodeLast(const fer_rrt_t *rrt);

/**
 * Creates new node in tree with configuration {conf} and this node will be
 * connected with node {n}. Node {n} must be already in RRT's net.
 */
const fer_rrt_node_t *ferRRTNodeNew(fer_rrt_t *rrt, const fer_vec_t *conf,
                                    const fer_rrt_node_t *n);

/**
 * Returns nearest node to given configuration {c}. Nearest node is
 * meassured in euclidean distance metric.
 * This function is used if ops.nearest is set to NULL.
 */
const fer_rrt_node_t *ferRRTNearest(const fer_rrt_t *rrt, const fer_vec_t *c);

/**
 * Tries to find path in net from init to goal.
 * If path was found 0 is returned and argument list is filled by nodes
 * representing path. Nodes are connected into this list by member .path.
 * If path wasn't found -1 is returned.
 */
int ferRRTFindPath(fer_rrt_t *rrt,
                   const fer_rrt_node_t *init, const fer_rrt_node_t *goal,
                   fer_list_t *list);

/**
 * Dumps net in SVT format.
 */
void ferRRTDumpSVT(fer_rrt_t *rrt, FILE *out, const char *name);

/**
 * Node Functions
 * ---------------
 *
 * See fer_rrt_node_t.
 */

/**
 * Returns configuration (state) of node.
 */
_fer_inline const fer_vec_t *ferRRTNodeConf(const fer_rrt_node_t *n);


/**** INLINES ****/
_fer_inline size_t ferRRTNodesLen(const fer_rrt_t *rrt)
{
    return ferNetNodesLen(rrt->net);
}

_fer_inline const fer_rrt_node_t *ferRRTNodeInitial(const fer_rrt_t *rrt)
{
    return rrt->node_init;
}

_fer_inline const fer_rrt_node_t *ferRRTNodeLast(const fer_rrt_t *rrt)
{
    return rrt->node_last;
}


_fer_inline const fer_vec_t *ferRRTNodeConf(const fer_rrt_node_t *n)
{
    return n->conf;
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __FER_RRT_H__ */
