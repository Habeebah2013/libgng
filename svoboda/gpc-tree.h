/***
 * Svoboda
 * --------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
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

#ifndef __SVO_GPC_TREE_H__
#define __SVO_GPC_TREE_H__

#include <stdio.h>
#include <svoboda/gpc.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Base struct for tree's node
 */
struct _svo_gpc_node_t {
    int idx;   /*!< Index of predicate/class */
    int ndesc; /*!< Number of descendants.
                    If 0, this is a terminal (class) node */
    struct _svo_gpc_node_t **desc; /*!< Array of descendants */
    void *mem; /*!< User defined memory */
} bor_packed;
typedef struct _svo_gpc_node_t svo_gpc_node_t;


/**
 * Contructs a new node
 */
svo_gpc_node_t *svoGPCNodeNew(int idx, int ndesc, size_t memsize);

/**
 * Recursively deletes a node and its subtree
 */
void svoGPCNodeDel(svo_gpc_node_t *node);

/**
 * Recursively clones a node tree
 */
svo_gpc_node_t *svoGPCNodeClone(svo_gpc_t *gpc, svo_gpc_node_t *node);




/**
 * Struct representing a whole tree (an individual)
 */
struct _svo_gpc_tree_t {
    bor_real_t fitness;   /*!< Fitness of individual represented by the tree */
    svo_gpc_node_t *root; /*!< Root node of the tree */
    int num_nodes;        /*!< Number of nodes in the tree */
    int depth;            /*!< Depth of the tree */
};
typedef struct _svo_gpc_tree_t svo_gpc_tree_t;


/**
 * Creates a new tree structure
 */
svo_gpc_tree_t *svoGPCTreeNew(void);

/**
 * Deletes a tree
 */
void svoGPCTreeDel(svo_gpc_tree_t *tree);

/**
 * Clones a whole tree.
 */
svo_gpc_tree_t *svoGPCTreeClone(svo_gpc_t *gpc, svo_gpc_tree_t *tree);

/**
 * Fixes info about a tree.
 * If the tree was changed, this function should be called to fix an info
 * about it stored in the structre.
 */
void svoGPCTreeFix(svo_gpc_tree_t *tree);

/**
 * Returns a idx'th node from a tree along with of its storage in desc
 * array and its depth.
 */
svo_gpc_node_t *svoGPCTreeNodeById(svo_gpc_tree_t *tree, int idx,
                                   svo_gpc_node_t ***desc, int *depth);

/**
 * Pretty print of a tree. For debug purposes.
 */
void __svoGPCTreePrint(const svo_gpc_tree_t *tree, FILE *fout);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SVO_GPC_TREE_H__ */
