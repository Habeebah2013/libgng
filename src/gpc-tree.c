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

#include <string.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>
#include <svoboda/gpc-tree.h>


svo_gpc_node_t *svoGPCNodeNew(int idx, int ndesc, size_t memsize)
{
    svo_gpc_node_t *node;

    node = BOR_ALLOC(svo_gpc_node_t);
    node->idx   = idx;
    node->ndesc = ndesc;

    node->desc = NULL;
    if (ndesc > 0){
        node->desc = BOR_ALLOC_ARR(svo_gpc_node_t *, ndesc);
    }

    node->mem = NULL;
    if (memsize > 0)
        node->mem = borRealloc(NULL, memsize);

    return node;
}

void svoGPCNodeDel(svo_gpc_node_t *node)
{
    int i;

    for (i = 0; i < node->ndesc; i++){
        if (node->desc[i] != NULL)
            svoGPCNodeDel(node->desc[i]);
    }

    if (node->desc != NULL)
        BOR_FREE(node->desc);
    if (node->mem != NULL)
        BOR_FREE(node->mem);
    BOR_FREE(node);
}

svo_gpc_node_t *svoGPCNodeClone(svo_gpc_t *gpc, svo_gpc_node_t *node)
{
    svo_gpc_node_t *n;
    size_t memsize;
    int i;

    memsize = 0;
    if (node->ndesc > 0)
        memsize = __svoGPCPredMemsize(gpc, node->idx);

    n = svoGPCNodeNew(node->idx, node->ndesc, memsize);

    memcpy(n->mem, node->mem, memsize);

    for (i = 0; i < n->ndesc; i++){
        n->desc[i] = svoGPCNodeClone(gpc, node->desc[i]);
    }

    return n;
}



svo_gpc_tree_t *svoGPCTreeNew(void)
{
    svo_gpc_tree_t *tree;

    tree = BOR_ALLOC(svo_gpc_tree_t);
    tree->fitness = BOR_ZERO;
    tree->root    = NULL;
    tree->num_nodes = 0;
    tree->depth     = 0;

    return tree;
}

void svoGPCTreeDel(svo_gpc_tree_t *tree)
{
    // TODO: del root and its subtree
    if (tree->root)
        svoGPCNodeDel(tree->root);
    BOR_FREE(tree);
}

svo_gpc_tree_t *svoGPCTreeClone(svo_gpc_t *gpc, svo_gpc_tree_t *tree)
{
    svo_gpc_tree_t *ntree;

    ntree            = svoGPCTreeNew();
    ntree->fitness   = tree->fitness;
    ntree->root      = svoGPCNodeClone(gpc, tree->root);
    ntree->num_nodes = tree->num_nodes;

    return ntree;
}

static int fixNumNodes(svo_gpc_node_t *node, int depth, int *rdepth)
{
    int i, num = 1;

    if (depth > *rdepth)
        *rdepth = depth;

    if (node->ndesc > 0){
        for (i = 0; i < node->ndesc; i++){
            num += fixNumNodes(node->desc[i], depth + 1, rdepth);
        }
    }

    return num;
}

void svoGPCTreeFix(svo_gpc_tree_t *tree)
{
    int depth = 0;

    if (tree->root){
        tree->num_nodes = fixNumNodes(tree->root, 0, &depth);
        tree->depth = depth;
    }
}


static int nodeById(svo_gpc_node_t *node, int idx, int cur, int depth,
                    svo_gpc_node_t **rnode, svo_gpc_node_t ***rdesc,
                    int *rdepth)
{
    int i, ret;

    if (cur == idx){
        // we reached the correct node, record the node and its depth
        if (*rnode == NULL){
            *rnode = node;
            *rdepth = depth;
        }
        return -1;
    }

    if (node->ndesc == 0)
        return cur;

    for (i = 0; i < node->ndesc; i++){
        ret = nodeById(node->desc[i], idx, cur + 1, depth + 1,
                       rnode, rdesc, rdepth);
        if (ret == -1){
            // a correct node was reached, record its storage in desc array
            if (*rdesc == NULL)
                *rdesc = &node->desc[i];
            return -1;
        }else{
            cur = ret;
        }
    }

    return cur;
}

svo_gpc_node_t *svoGPCTreeNodeById(svo_gpc_tree_t *tree, int idx,
                                   svo_gpc_node_t ***desc, int *depth)
{
    svo_gpc_node_t *node;

    *depth = 0;

    if (idx == 0){
        *desc = &tree->root;
        return tree->root;
    }

    node = NULL;
    *desc = NULL;
    if (nodeById(tree->root, idx, 0, 0, &node, desc, depth) != -1)
        return NULL;

    return node;
}

static void svoGPCNodePrint(const svo_gpc_node_t *node, FILE *fout, int depth)
{
    int i;

    for (i = 0; i < depth; i++)
        fprintf(fout, "  ");

    fprintf(fout, "idx: %d, ndesc: %d [%lx]",
            (int)node->idx, (int)node->ndesc, (long)node);

    for (i = 0; i < node->ndesc; i++){
        fprintf(fout, " %lx", (long)&node->desc[i]);
    }
    fprintf(fout, "\n");

    for (i = 0; i < node->ndesc; i++){
        svoGPCNodePrint(node->desc[i], fout, depth + 1);
    }
}

void __svoGPCTreePrint(const svo_gpc_tree_t *tree, FILE *fout)
{
    fprintf(fout, "fitness: %f, num_nodes: %d [%lx]\n",
            tree->fitness, (int)tree->num_nodes, (long)tree);
    if (tree->root)
        svoGPCNodePrint(tree->root, fout, 0);
    fprintf(fout, "--------\n");
}
