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

#ifndef __SVO_PRM_H__
#define __SVO_PRM_H__

#include <boruvka/vec.h>
#include <boruvka/net.h>
#include <boruvka/gug.h>
#include <boruvka/dij.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define SVO_PRM_FREE 1
#define SVO_PRM_OBST 2


/**
 * PRM - Probabilistic Roadmaps for Path Planning
 * ===============================================
 */

/**
 * PRM Operations
 * ---------------
 *
 * See svo_prm_ops_t.
 */

/** vvvv */

/**
 * Returns random configuration.
 */
typedef const bor_vec_t *(*svo_prm_conf)(void *);

/**
 * Returns true if algorithm should terminate.
 */
typedef int (*svo_prm_terminate)(void *);

/**
 * Evaluate configuration.
 *
 * Returns SVO_PRM_FREE if configuration is in free space and SVO_PRM_OBST
 * if it is obstacle configuration.
 */
typedef int (*svo_prm_eval)(const bor_vec_t *c, void *);

/**
 * Returns true if there exists path from start configuration to goal
 * configuration.
 */
typedef int (*svo_prm_find_path)(const bor_vec_t *start,
                                 const bor_vec_t *goal, void *);

/**
 * Callback that is peridically called from PRM.
 *
 * It is called every .callback_period'th added node.
 */
typedef void (*svo_prm_callback)(void *);

/** ^^^^ */

struct _svo_prm_ops_t {
    svo_prm_conf      conf;
    svo_prm_terminate terminate;
    svo_prm_eval      eval;
    svo_prm_find_path find_path;

    svo_prm_callback callback;
    unsigned long callback_period;

    void *data; /*!< Data pointer that will be provided to all callbacks if
                     not specified otherwise. */

    void *conf_data;
    void *terminate_data;
    void *eval_data;
    void *find_path_data;
    void *callback_data;
};
typedef struct _svo_prm_ops_t svo_prm_ops_t;

/**
 * Initializes ops struct to NULL values.
 */
void svoPRMOpsInit(svo_prm_ops_t *ops);



/**
 * PRM Parameters
 * ---------------
 */
struct _svo_prm_params_t {
    int d; /*!< Dimension of problem */

    bor_real_t max_dist;      /*!< Maximal distance between nodes that the
                                   local planner may try to connect */
    bor_real_t max_neighbors; /*!< Maximum number of calls of the local
                                   planner per node */

    bor_gug_params_t gug;
};
typedef struct _svo_prm_params_t svo_prm_params_t;

/**
 * Initializes params struct to default values.
 */
void svoPRMParamsInit(svo_prm_params_t *params);



/**
 * PRM Algorithm
 * --------------
 */
struct _svo_prm_t {
    bor_net_t *net; /*!< Holds roadmap */
    bor_gug_t *gug; /*!< NN search */

    svo_prm_ops_t ops;
    svo_prm_params_t params;

    bor_list_t components;
};
typedef struct _svo_prm_t svo_prm_t;

struct _svo_prm_component_t {
    struct _svo_prm_component_t *parent;
    bor_list_t list;
};
typedef struct _svo_prm_component_t svo_prm_component_t;

struct _svo_prm_node_t {
    bor_vec_t *conf;
    svo_prm_component_t *comp;
    bor_net_node_t node;
    bor_gug_el_t gug;

    bor_dij_node_t dij; /*!< Connection for dijkstra algorithm */
    bor_list_t path;

    int _id;
};
typedef struct _svo_prm_node_t svo_prm_node_t;


/**
 * Creates new instance of PRM algorithm.
 */
svo_prm_t *svoPRMNew(const svo_prm_ops_t *ops,
                     const svo_prm_params_t *params);

/**
 * Deletes PRM instance.
 */
void svoPRMDel(svo_prm_t *prm);

/**
 * Runs algorithm
 */
void svoPRMRun(svo_prm_t *prm);

/**
 * Tries to find path in net from start to goal.
 * If path was found 0 is returned and argument list is filled by nodes
 * representing path. Nodes are connected into this list by member .path.
 * If path wasn't found -1 is returned.
 */
int svoPRMFindPath(svo_prm_t *prm,
                   const bor_vec_t *start, const bor_vec_t *goal,
                   bor_list_t *list);

/**
 * Returns number of nodes in roadmap.
 */
_bor_inline size_t svoPRMNodesLen(const svo_prm_t *prm);

/**
 * Dumps net in SVT format.
 */
void svoPRMDumpSVT(svo_prm_t *prm, FILE *out, const char *name);

/**** INLINES ****/
_bor_inline size_t svoPRMNodesLen(const svo_prm_t *prm)
{
    return borNetNodesLen(prm->net);
}


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_PRM_H__ */

