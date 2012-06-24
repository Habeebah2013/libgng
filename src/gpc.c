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

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>
#include <boruvka/sort.h>
#include <svoboda/gpc.h>
#include <svoboda/gpc-tree.h>


/**
 * Definition of predicate
 */
struct _svo_gpc_pred_t {
    svo_gpc_pred pred;      /*!< Predicate callback */
    svo_gpc_pred_init init; /*!< Initialization callback */
    svo_gpc_pred_format format; /*!< Format callback */
    size_t memsize;         /*!< Size of allocated memory per predicate instance */
    int ndesc;              /*!< Number of descendants */
    void *data;             /*!< User data */
};
typedef struct _svo_gpc_pred_t svo_gpc_pred_t;

/**
 * Definition of class
 */
struct _svo_gpc_class_t {
    int class; /*!< Class identifier */
};
typedef struct _svo_gpc_class_t svo_gpc_class_t;


/** Randomly generate a node with its subtree */
static svo_gpc_node_t *svoGPCGenTree(svo_gpc_t *gpc, int depth, int max_depth);
static svo_gpc_node_t *svoGPCGenClass(svo_gpc_t *gpc);
/** Create randomly generated initial population in gpc->pop[pop] */
static void svoGPCCreateInitPop(svo_gpc_t *gpc, int pop);
/** Evals a tree with one data row. Returns resulting class as defined by
 *  svoGPCAddClass() function. */
static int svoGPCEvalTreeClass(svo_gpc_t *gpc, svo_gpc_tree_t *tree, void *data);
/** Evaluate a tree on all data rows and records a new fitness that is also
 *  returned */
static bor_real_t svoGPCEvalTree(svo_gpc_t *gpc, svo_gpc_tree_t *tree,
                                 int res_arr);
/** Evaluate whole population of decision trees */
static void svoGPCEvalPop(svo_gpc_t *gpc, int pop);
/** Sort whole population in descending order */
static void svoGPCSortPop(svo_gpc_t *gpc, int pop);
/** Performs elitism */
static void svoGPCKeepBest(svo_gpc_t *gpc, int from_pop, int to_pop);
/** Throw away the worst individuals from given population */
static void svoGPCThrowWorst(svo_gpc_t *gpc, int pop);
/** Create a new population */
static void svoGPCCreateNewPop(svo_gpc_t *gpc, int from_pop, int to_pop);
/** Reset speficied population, i.e., delete remaining individuals and
 *  truncate it to zero length. */
static void svoGPCResetPop(svo_gpc_t *gpc, int pop);
/** Tournament selection */
static int svoGPCSelectionTournament(svo_gpc_t *gpc, int tour_size,
                                     svo_gpc_tree_t **pop, int pop_size);
/** Genetic operators: */
static void svoGPCReproduction(svo_gpc_t *gpc, int from_pop, int to_pop);
static void svoGPCCrossover(svo_gpc_t *gpc, int from_pop, int to_pop);
static void svoGPCMutation(svo_gpc_t *gpc, int from_pop, int to_pop);
/** Reproduce a particular tree */
static void svoGPCReproduction2(svo_gpc_t *gpc, int to_pop, svo_gpc_tree_t *tree);

/** Simplifies all trees in speficied population */
static void svoGPCSimplify(svo_gpc_t *gpc, int pop);
static void svoGPCPruneDeep(svo_gpc_t *gpc, int pop);
/** Removes all duplicates from the population */
static void svoGPCRemoveDuplicates(svo_gpc_t *gpc, int pop);

#define OPS_DATA(name) \
    if (!gpc->ops.name ## _data) \
        gpc->ops.name ## _data = gpc->ops.data;

#define OPS_CHECK(name) \
    if (!gpc->ops.name){ \
        fprintf(stderr, "Fermat :: GPC :: No ." #name " callback set.\n"); \
        exit(-1); \
    }

#define OPS_CHECK_DATA(name) \
    OPS_DATA(name) \
    OPS_CHECK(name)


void svoGPCOpsInit(svo_gpc_ops_t *ops)
{
    memset(ops, 0, sizeof(*ops));
}

void svoGPCParamsInit(svo_gpc_params_t *params)
{
    params->pop_size    = 1;
    params->max_depth   = 5;
    params->keep_best   = 1;
    params->throw_worst = 1;
    params->max_steps   = 10UL;
    params->data_rows   = 0;

    params->tournament_size = 5;

    params->pr        = 14;
    params->pc        = 85;
    params->pm        = 1;

    params->simplify      = 0UL;
    params->prune_deep    = 0UL;
    params->rm_duplicates = 0UL;
    params->inc_max_depth = 0UL;
    params->inc_max_depth_step = 1;

    params->parallel = 0;
}



svo_gpc_t *svoGPCNew(const svo_gpc_ops_t *ops, const svo_gpc_params_t *params)
{
    svo_gpc_t *gpc;
    bor_real_t prob;
    int i;

    gpc = BOR_ALLOC(svo_gpc_t);

    gpc->params = *params;
    prob = gpc->params.pr + gpc->params.pc + gpc->params.pm;
    gpc->params.pr = gpc->params.pr / prob;
    gpc->params.pc = gpc->params.pc / prob;
    gpc->params.pm = gpc->params.pm / prob;

    gpc->ops = *ops;
    OPS_CHECK_DATA(fitness);
    OPS_CHECK_DATA(data_row);
    OPS_DATA(callback);

    gpc->rand = borRandMTNewAuto();
    //gpc->rand = borRandMTNew(9999);

    gpc->pop[0] = BOR_ALLOC_ARR(svo_gpc_tree_t *, gpc->params.pop_size);
    gpc->pop[1] = BOR_ALLOC_ARR(svo_gpc_tree_t *, gpc->params.pop_size);
    gpc->pop[2] = BOR_ALLOC_ARR(svo_gpc_tree_t *, gpc->params.pop_size);
    memset(gpc->pop[0], 0, sizeof(svo_gpc_tree_t *) * gpc->params.pop_size);
    memset(gpc->pop[1], 0, sizeof(svo_gpc_tree_t *) * gpc->params.pop_size);
    gpc->pop_size[0] = gpc->pop_size[1] = 0;

    gpc->pred_size = SVO_GPC_PRED_INIT_SIZE;
    gpc->pred_len  = 0;
    gpc->pred      = BOR_ALLOC_ARR(svo_gpc_pred_t, gpc->pred_size);

    gpc->class_size = SVO_GPC_CLASS_INIT_SIZE;
    gpc->class_len  = 0;
    gpc->class      = BOR_ALLOC_ARR(svo_gpc_class_t, gpc->class_size);


    gpc->threads = (params->parallel > 1 ? params->parallel : 1);

    gpc->eval_results = BOR_ALLOC_ARR(int *, gpc->threads);
    for (i = 0; i < gpc->threads; i++){
        gpc->eval_results[i] = BOR_ALLOC_ARR(int, gpc->params.data_rows);
    }

    gpc->tasks = 0;
    if (gpc->threads > 1){
        gpc->tasks = borTasksNew(gpc->threads);
        borTasksRun(gpc->tasks);
    }

    return gpc;
}

void svoGPCDel(svo_gpc_t *gpc)
{
    int i;

    borRandMTDel(gpc->rand);

    for (i = 0; i < gpc->params.pop_size; i++){
        if (gpc->pop[0][i] != NULL)
            svoGPCTreeDel(gpc->pop[0][i]);
        if (gpc->pop[1][i] != NULL)
            svoGPCTreeDel(gpc->pop[1][i]);
    }
    BOR_FREE(gpc->pop[0]);
    BOR_FREE(gpc->pop[1]);
    BOR_FREE(gpc->pop[2]);

    BOR_FREE(gpc->pred);
    BOR_FREE(gpc->class);

    for (i = 0; i < gpc->threads; i++){
        BOR_FREE(gpc->eval_results[i]);
    }
    BOR_FREE(gpc->eval_results);

    if (gpc->tasks)
        borTasksDel(gpc->tasks);

    BOR_FREE(gpc);
}

int svoGPCMaxDepth(const svo_gpc_t *gpc)
{
    return gpc->params.max_depth;
}

int svoGPCAddPred(svo_gpc_t *gpc,
                  svo_gpc_pred pred,
                  svo_gpc_pred_init init,
                  svo_gpc_pred_format format,
                  int num_descendants, size_t memsize,
                  void *data)
{
    if (gpc->pred_len >= gpc->pred_size){
        gpc->pred_size *= 2;
        gpc->pred = BOR_REALLOC_ARR(gpc->pred, svo_gpc_pred_t, gpc->pred_size);
    }

    gpc->pred[gpc->pred_len].pred    = pred;
    gpc->pred[gpc->pred_len].init    = init;
    gpc->pred[gpc->pred_len].format  = format;
    gpc->pred[gpc->pred_len].memsize = memsize;
    gpc->pred[gpc->pred_len].ndesc   = num_descendants;
    gpc->pred[gpc->pred_len].data    = data;
    gpc->pred_len++;

    return 0;
}

int svoGPCAddClass(svo_gpc_t *gpc, int class_id)
{
    if (gpc->class_len >= gpc->class_size){
        gpc->class_size *= 2;
        gpc->class = BOR_REALLOC_ARR(gpc->class, svo_gpc_class_t, gpc->class_size);
    }

    gpc->class[gpc->class_len].class = class_id;
    gpc->class_len++;

    return 0;
}

int __svoGPCPredMemsize(const svo_gpc_t *gpc, int idx)
{
    return gpc->pred[idx].memsize;
}



#define PERIODIC_OP(varname, call) \
    varname += 1UL; \
    if (varname == gpc->params.varname){ \
        call; \
        varname = 0UL; \
    }

int svoGPCRun(svo_gpc_t *gpc)
{
    unsigned long step, cb, simplify, prune_deep, rm_duplicates;
    unsigned long inc_max_depth;
    int pop_cur, pop_other, pop_tmp;

    // early exit if we don't have any classes
    if (gpc->class_len == 0)
        return -1;

    // initialize stats
    memset(&gpc->stats, 0, sizeof(gpc->stats));


    gpc->pop_cur = pop_cur = 0;
    pop_other = 1;


    // create initial population
    svoGPCCreateInitPop(gpc, pop_cur);

    // evaluate initial population
    svoGPCEvalPop(gpc, pop_cur);

    if (gpc->ops.callback){
        gpc->ops.callback(gpc, gpc->ops.callback_data);
    }

    cb = 0UL;
    simplify      = 0UL;
    prune_deep    = 0UL;
    rm_duplicates = 0UL;
    inc_max_depth = 0UL;
    for (step = 0UL; step < gpc->params.max_steps; step += 1UL){
        // perform elitism and the opposite
        svoGPCKeepBest(gpc, pop_cur, pop_other);
        svoGPCThrowWorst(gpc, pop_cur);

        // create a new population
        svoGPCCreateNewPop(gpc, pop_cur, pop_other);

        // reset the old population
        svoGPCResetPop(gpc, pop_cur);

        // prune deep trees
        PERIODIC_OP(prune_deep, svoGPCPruneDeep(gpc, pop_other));

        // simplify a new population
        PERIODIC_OP(simplify, svoGPCSimplify(gpc, pop_other));

        // evaluate a new population
        svoGPCEvalPop(gpc, pop_other);

        // remove duplicates
        PERIODIC_OP(rm_duplicates, svoGPCRemoveDuplicates(gpc, pop_other));

        // increase max depth
        PERIODIC_OP(inc_max_depth,
                    gpc->params.max_depth += gpc->params.inc_max_depth_step);


        // switch old and new population
        BOR_SWAP(pop_cur, pop_other, pop_tmp);
        gpc->pop_cur = pop_cur;


        // update stats
        gpc->stats.elapsed = step + 1;

        cb += 1UL;
        if (cb == gpc->ops.callback_period && gpc->ops.callback){
            gpc->ops.callback(gpc, gpc->ops.callback_data);
            cb = 0UL;
        }
    }

    // remove duplicates
    svoGPCRemoveDuplicates(gpc, pop_other);

    // simplify resulting population
    svoGPCSimplify(gpc, gpc->pop_cur);

    return 0;
}

bor_real_t svoGPCBestFitness(const svo_gpc_t *gpc)
{
    if (gpc->pop[gpc->pop_cur][0])
        return gpc->pop[gpc->pop_cur][0]->fitness;
    return -BOR_REAL_MAX;
}

void *svoGPCTree(const svo_gpc_t *gpc, int i)
{
    i = BOR_MIN(i, gpc->pop_size[gpc->pop_cur] - 1);
    return (void *)gpc->pop[gpc->pop_cur][i];
}

int svoGPCTreeEval(svo_gpc_t *gpc, void *tree, void *data)
{
    return svoGPCEvalTreeClass(gpc, (svo_gpc_tree_t *)tree, data);
}

int svoGPCTreeDepth(svo_gpc_t *gpc, void *tree)
{
    return ((svo_gpc_tree_t *)tree)->depth;
}


static void printBest(svo_gpc_t *gpc, svo_gpc_node_t *node, FILE *fout,
                      char *str, size_t str_maxlen, int depth)
{
    int i, j;

    str[0] = 0x0;

    for (i = 0; i < depth; i++){
        fprintf(fout, "    ");
    }

    if (node->ndesc == 0){
        fprintf(fout, "return %d;\n", gpc->class[node->idx].class);
    }else{
        fprintf(fout, "if (");
        if (gpc->pred[node->idx].format){
            gpc->pred[node->idx].format(gpc, node->mem,
                                        gpc->pred[node->idx].data,
                                        str, str_maxlen);
            fprintf(fout, "%s", str);
        }else{
            fprintf(fout, "UNKOWN PREDICATE");
        }
        fprintf(fout, "){\n");

        for (i = 0; i < node->ndesc; i++){
            printBest(gpc, node->desc[i], fout, str, str_maxlen, depth + 1);
            if (i < node->ndesc - 1){
                for (j = 0; j < depth; j++)
                    fprintf(fout, "    ");
                fprintf(fout, "}else{\n");
            }
        }

        for (i = 0; i < depth; i++)
            fprintf(fout, "    ");
        fprintf(fout, "}\n");
    }

}

void svoGPCTreePrintC(svo_gpc_t *gpc, void *_tree, const char *func_name, FILE *fout)
{
    svo_gpc_tree_t *tree = (svo_gpc_tree_t *)_tree;
    char str[1024];

    if (!tree)
        return;

    fprintf(fout, "int %s(bor_real_t *data)\n{\n", func_name);
    printBest(gpc, tree->root, fout, str, 1024, 1);
    fprintf(fout, "}\n");
}

void svoGPCStats(const svo_gpc_t *gpc, svo_gpc_stats_t *stats)
{
    int i, pop;
    svo_gpc_tree_t *tree;

    pop = gpc->pop_cur;

    stats->min_fitness = BOR_REAL_MAX;
    stats->max_fitness = -BOR_REAL_MAX;
    stats->avg_fitness = BOR_ZERO;
    stats->min_nodes = INT_MAX;
    stats->max_nodes = 0;
    stats->avg_nodes = 0;
    stats->min_depth = INT_MAX;
    stats->max_depth = 0;
    stats->avg_depth = 0;
    for (i = 0; i < gpc->pop_size[pop]; i++){
        tree = gpc->pop[pop][i];

        if (tree->fitness > stats->max_fitness)
            stats->max_fitness = tree->fitness;
        if (tree->fitness < stats->min_fitness)
            stats->min_fitness = tree->fitness;
        stats->avg_fitness += tree->fitness;

        if (tree->num_nodes < stats->min_nodes)
            stats->min_nodes = tree->num_nodes;
        if (tree->num_nodes > stats->max_nodes)
            stats->max_nodes = tree->num_nodes;
        stats->avg_nodes += tree->num_nodes;

        if (tree->depth < stats->min_depth)
            stats->min_depth = tree->depth;
        if (tree->depth > stats->max_depth)
            stats->max_depth = tree->depth;
        stats->avg_depth += tree->depth;

    }
    stats->avg_fitness /= gpc->pop_size[pop];
    stats->avg_nodes /= gpc->pop_size[pop];
    stats->avg_depth /= gpc->pop_size[pop];

    stats->med_fitness = gpc->pop[pop][gpc->pop_size[pop] / 2 + 1]->fitness;
    if (gpc->pop_size[pop] % 2 == 0){
        stats->med_fitness += gpc->pop[pop][gpc->pop_size[pop] / 2]->fitness;
        stats->med_fitness /= BOR_REAL(2.);
    }

    stats->elapsed = gpc->stats.elapsed;
}


static svo_gpc_node_t *svoGPCGenTree(svo_gpc_t *gpc, int depth, int max_depth)
{
    svo_gpc_node_t *node;
    int idx, i;

    // Randomly choose a predicate or a class.
    // If max_depth is reached, generate only a class.
    if (depth == max_depth){
        idx = svoGPCRandInt(gpc, 0, gpc->class_len);
        idx += gpc->pred_len;
    }else{
        idx = svoGPCRandInt(gpc, 0, gpc->pred_len + gpc->class_len);
    }

    if (idx >= gpc->pred_len){
        // create a new class node
        idx = idx - gpc->pred_len;
        node = svoGPCNodeNew(idx, 0, 0);

    }else{
        // create a new predicate node
        node = svoGPCNodeNew(idx, gpc->pred[idx].ndesc,
                             gpc->pred[idx].memsize);
        // initialize it
        if (gpc->pred[idx].init){
            gpc->pred[idx].init(gpc, node->mem,
                                gpc->pred[idx].data);
        }

        // and fill descendants
        for (i = 0; i < node->ndesc; i++){
            node->desc[i] = svoGPCGenTree(gpc, depth + 1, max_depth);
        }
    }

    return node;
}
static svo_gpc_node_t *svoGPCGenClass(svo_gpc_t *gpc)
{
    int idx;
    idx = svoGPCRandInt(gpc, 0, gpc->class_len);
    return svoGPCNodeNew(idx, 0, 0);
}


static void svoGPCCreateInitPop(svo_gpc_t *gpc, int pop)
{
    int i, len;
    int pop_other;

    pop_other = (pop + 1) % 2;

    gpc->pop_size[pop] = 0;
    gpc->pop_size[pop_other] = 0;

    len = gpc->params.pop_size;
    for (i = 0; i < len; i++){
        gpc->pop[pop][i] = svoGPCTreeNew();
        gpc->pop[pop][i]->root = svoGPCGenTree(gpc, 0, gpc->params.max_depth);
        svoGPCTreeFix(gpc->pop[pop][i]);
        gpc->pop_size[pop]++;

        gpc->pop[pop_other][i] = NULL;

        //svoGPCTreePrint(gpc->pop[0][i], stdout);
    }
}

static int svoGPCEvalTreeClass(svo_gpc_t *gpc, svo_gpc_tree_t *tree, void *data)
{
    svo_gpc_node_t *node;
    svo_gpc_pred pred;
    int dispatch;

    // traverse the decision tree
    node = tree->root;
    while (node->ndesc != 0){
        pred = gpc->pred[node->idx].pred;

        // find which descendant is used
        dispatch = pred(gpc, node->mem, data, gpc->pred[node->idx].data);
        dispatch = BOR_MIN(dispatch, node->ndesc - 1);

        // descent to next node
        node = node->desc[dispatch];
    }

    return gpc->class[node->idx].class;
}

static bor_real_t svoGPCEvalTree(svo_gpc_t *gpc, svo_gpc_tree_t *tree,
                                 int res_arr)
{
    int i, class;
    void *data;

    for (i = 0; i < gpc->params.data_rows; i++){
        // obtain a data row using a callback
        data = gpc->ops.data_row(gpc, i, gpc->ops.data_row_data);

        // eval the tree on the data row
        class = svoGPCEvalTreeClass(gpc, tree, data);
        gpc->eval_results[res_arr][i] = class;
    }

    // compute a fitness value
    tree->fitness = gpc->ops.fitness(gpc, gpc->eval_results[res_arr],
                                     gpc->ops.fitness_data);

    //svoGPCTreePrint(tree, stdout);

    return tree->fitness;
}

struct eval_pop_t {
    svo_gpc_t *gpc;
    int pop;
};
static void svoGPCEvalPopTask(int id, void *data, const bor_tasks_thinfo_t *thinfo)
{
    struct eval_pop_t *ev = (struct eval_pop_t *)data;
    svo_gpc_t *gpc = ev->gpc;
    int pop = ev->pop;
    int len, from, to, i;

    len = gpc->pop_size[pop] / gpc->threads;
    from = id * len;
    to   = from + len;
    if (id == gpc->threads - 1)
        to = gpc->pop_size[pop];
    len = to - from;

    for (i = from; i < to; i++){
        svoGPCEvalTree(gpc, gpc->pop[pop][i], id);
    }
}

static void svoGPCEvalPop(svo_gpc_t *gpc, int pop)
{
    int i;
    struct eval_pop_t ev;

    if (gpc->threads > 1){
        ev.gpc = gpc;
        ev.pop = pop;

        for (i = 0; i < gpc->threads; i++){
            borTasksAdd(gpc->tasks, svoGPCEvalPopTask, i, (void *)&ev);
        }
        borTasksBarrier(gpc->tasks);
    }else{
        for (i = 0; i < gpc->pop_size[pop]; i++){
            svoGPCEvalTree(gpc, gpc->pop[pop][i], 0);
        }
    }

    svoGPCSortPop(gpc, pop);
}

static void svoGPCSortPop(svo_gpc_t *gpc, int pop)
{
    borRadixSortPtr((void **)gpc->pop[pop], (void **)gpc->pop[2],
                    gpc->pop_size[pop],
                    bor_offsetof(svo_gpc_tree_t, fitness), 1);
}

static void svoGPCKeepBest(svo_gpc_t *gpc, int from_pop, int to_pop)
{
    int i;

    // copy the best individuals
    for (i = 0; i < gpc->params.keep_best; i++){
        if (gpc->pop[from_pop][i] != NULL)
            svoGPCReproduction2(gpc, to_pop, gpc->pop[from_pop][i]);
    }
}

static void svoGPCThrowWorst(svo_gpc_t *gpc, int pop)
{
    svo_gpc_tree_t *tree;
    int i;

    // throw the worst
    for (i = 0; i < gpc->params.throw_worst; i++){
        tree = gpc->pop[pop][gpc->pop_size[pop] - i - 1];
        if (tree != NULL)
            svoGPCTreeDel(tree);
        gpc->pop[pop][gpc->pop_size[pop] - i - 1] = NULL;
    }
    gpc->pop_size[pop] -= gpc->params.throw_worst;
}


static void svoGPCCreateNewPop(svo_gpc_t *gpc, int from_pop, int to_pop)
{
    bor_real_t action;

    while (gpc->pop_size[to_pop] < gpc->params.pop_size
            && gpc->pop_size[from_pop] > 0){
        // choose action
        action = svoGPCRand01(gpc);

        if (action < gpc->params.pr){
            // reproduction
            svoGPCReproduction(gpc, from_pop, to_pop);
        }else if (action < gpc->params.pr + gpc->params.pc){
            // crossover
            svoGPCCrossover(gpc, from_pop, to_pop);
        }else{
            // mutation
            svoGPCMutation(gpc, from_pop, to_pop);
        }
    }
}

static void svoGPCResetPop(svo_gpc_t *gpc, int pop)
{
    int i;

    for (i = 0; i < gpc->pop_size[pop]; i++){
        if (gpc->pop[pop][i])
            svoGPCTreeDel(gpc->pop[pop][i]);
        gpc->pop[pop][i] = NULL;
    }
    gpc->pop_size[pop] = 0;
}

static int svoGPCSelectionTournament(svo_gpc_t *gpc, int tour_size,
                                     svo_gpc_tree_t **pop, int pop_size)
{
    bor_real_t best_fitness;
    int i, sel, best = 0;

    best_fitness = -BOR_REAL_MAX;
    for (i = 0; i < tour_size; i++){
        sel = svoGPCRandInt(gpc, 0, pop_size);
        if (pop[sel]->fitness > best_fitness){
            best = sel;
        }
    }

    return best;
}

static void svoGPCReproduction(svo_gpc_t *gpc, int from_pop, int to_pop)
{
    int idx;

    // select an individual from population
    idx = svoGPCSelectionTournament(gpc, gpc->params.tournament_size,
                                    gpc->pop[from_pop], gpc->pop_size[from_pop]);

    // and copy the individual to other population
    gpc->pop[to_pop][gpc->pop_size[to_pop]] = svoGPCTreeClone(gpc, gpc->pop[from_pop][idx]);
    gpc->pop_size[to_pop]++;
}

static void svoGPCReproduction2(svo_gpc_t *gpc, int to_pop, svo_gpc_tree_t *tree)
{
    gpc->pop[to_pop][gpc->pop_size[to_pop]] = svoGPCTreeClone(gpc, tree);
    gpc->pop_size[to_pop]++;
}

static void svoGPCCrossover(svo_gpc_t *gpc, int from_pop, int to_pop)
{
    int idx[2], node_idx[2];
    int depth[2];
    int i;
    svo_gpc_node_t *node[2];
    svo_gpc_node_t **desc[2];
    svo_gpc_tree_t *tree[2];

    if (gpc->pop_size[from_pop] < 2)
        return;

    // select two individuals from population
    idx[0] = svoGPCSelectionTournament(gpc, gpc->params.tournament_size,
                                       gpc->pop[from_pop], gpc->pop_size[from_pop]);
    do {
        idx[1] = svoGPCSelectionTournament(gpc, gpc->params.tournament_size,
                                           gpc->pop[from_pop], gpc->pop_size[from_pop]);
    } while (idx[0] == idx[1]);

    for (i = 0; i < 2; i++){
        tree[i] = gpc->pop[from_pop][idx[i]];

        // choose crossover nodes
        node_idx[i] = svoGPCRandInt(gpc, 0, tree[i]->num_nodes);

        // obtain those nodes
        node[i] = svoGPCTreeNodeById(tree[i], node_idx[i], &desc[i], &depth[i]);
    }

    // switch the subtrees
    *desc[0] = node[1];
    *desc[1] = node[0];
    svoGPCTreeFix(tree[0]);
    svoGPCTreeFix(tree[1]);


    if (idx[0] < idx[1]){
        BOR_SWAP(idx[0], idx[1], i);
    }


    // move the trees to destination population
    for (i = 0; i < 2; i++){
        if (gpc->pop_size[to_pop] >= gpc->params.pop_size){
            // delete a tree if the population is full
            svoGPCTreeDel(tree[i]);
        }else{
            //svoGPCTreePrint(tree[i], stderr);
            gpc->pop[to_pop][gpc->pop_size[to_pop]] = tree[i];
            gpc->pop_size[to_pop]++;
        }

        gpc->pop[from_pop][idx[i]] = gpc->pop[from_pop][gpc->pop_size[from_pop] - 1];
        gpc->pop[from_pop][gpc->pop_size[from_pop] - 1] = NULL;
        gpc->pop_size[from_pop]--;
    }
        
}


static void svoGPCMutation(svo_gpc_t *gpc, int from_pop, int to_pop)
{
    int idx, node_idx;
    int depth;
    svo_gpc_tree_t *tree;
    svo_gpc_node_t *node, **desc;

    // select an individual from population
    idx = svoGPCSelectionTournament(gpc, gpc->params.tournament_size,
                                    gpc->pop[from_pop], gpc->pop_size[from_pop]);
    tree = gpc->pop[from_pop][idx];

    do {
        // choose a node that will undergo a mutation
        node_idx = svoGPCRandInt(gpc, 0, tree->num_nodes);

        // get a node from the tree
        node = svoGPCTreeNodeById(tree, node_idx, &desc, &depth);

    } while (gpc->params.max_depth < depth);

    // delete an old subtree and generate a new one
    svoGPCNodeDel(node);
    *desc = svoGPCGenTree(gpc, 0, gpc->params.max_depth - depth);
    svoGPCTreeFix(tree);

    // copy the tree to destination array
    gpc->pop[to_pop][gpc->pop_size[to_pop]] = tree;
    gpc->pop_size[to_pop]++;

    // remove the tree from source array
    gpc->pop[from_pop][idx] = gpc->pop[from_pop][gpc->pop_size[from_pop] - 1];
    gpc->pop[from_pop][gpc->pop_size[from_pop] - 1] = NULL;
    gpc->pop_size[from_pop]--;
}


static void svoGPCSimplifyDuplicatePred(svo_gpc_t *gpc, svo_gpc_node_t *node,
                                        svo_gpc_node_t **desc, int desc_i)
{
    void *b1, *b2;
    svo_gpc_node_t *rm;
    size_t memsize;

    b1 = node->mem;
    b2 = desc[desc_i]->mem;

    memsize = gpc->pred[node->idx].memsize;
    if (memsize == 0 || memcmp(b1, b2, memsize) == 0){
        rm      = desc[desc_i];

        desc[desc_i]    = rm->desc[desc_i];
        rm->desc[desc_i] = NULL;
        svoGPCNodeDel(rm);
    }
}

static svo_gpc_node_t *svoGPCSimplifySubtree(svo_gpc_t *gpc, svo_gpc_node_t *node)
{
    int i, idx;
    svo_gpc_node_t *rnode;

    if (node->ndesc == 0)
        return node;


    // first dive down in tree
    for (i = 0; i < node->ndesc; i++){
        if (node->desc[i]->ndesc > 0)
            node->desc[i] = svoGPCSimplifySubtree(gpc, node->desc[i]);
    }

    // check if all descendants are terminals and compare all their idx
    // also check if there isn't a duplicate predicate
    idx = node->desc[0]->idx;
    for (i = 0; i < node->ndesc; i++){
        if (node->desc[i]->idx == node->idx
                && node->desc[i]->ndesc == node->ndesc){
            svoGPCSimplifyDuplicatePred(gpc, node, node->desc, i);
        }

        if (node->desc[i]->ndesc != 0 || idx != node->desc[i]->idx)
            break;
    }

    if (i == node->ndesc){
        rnode = node->desc[0];
        node->desc[0] = NULL;
        svoGPCNodeDel(node);
        return rnode;
    }

    return node;
}

static void svoGPCSimplify(svo_gpc_t *gpc, int pop)
{
    int i;

    for (i = 0; i < gpc->pop_size[pop]; i++){
        gpc->pop[pop][i]->root = svoGPCSimplifySubtree(gpc, gpc->pop[pop][i]->root);
        svoGPCTreeFix(gpc->pop[pop][i]);
    }
}


static void svoGPCPruneDeepSubtree(svo_gpc_t *gpc, svo_gpc_node_t *node,
                                   int depth)
{
    int i;

    if (node->ndesc == 0)
        return;

    if (depth == gpc->params.max_depth - 1){
        for (i = 0; i < node->ndesc; i++){
            if (node->desc[i]->ndesc != 0){
                svoGPCNodeDel(node->desc[i]);
                node->desc[i] = svoGPCGenClass(gpc);
            }
        }
    }else{
        for (i = 0; i < node->ndesc; i++){
            if (node->desc[i]->ndesc != 0){
                svoGPCPruneDeepSubtree(gpc, node->desc[i], depth + 1);
            }
        }
    }
}

static void svoGPCPruneDeep(svo_gpc_t *gpc, int pop)
{
    int i;

    for (i = 0; i < gpc->pop_size[pop]; i++){
        svoGPCPruneDeepSubtree(gpc, gpc->pop[pop][i]->root, 0);
        svoGPCTreeFix(gpc->pop[pop][i]);
    }
}

static int eqTrees(svo_gpc_t *gpc, svo_gpc_node_t *n1, svo_gpc_node_t *n2)
{
    int i;

    if (n1->idx != n2->idx)
        return 0;

    if (n1->ndesc > 0){
        if (memcmp(n1->mem, n2->mem, gpc->pred[n1->idx].memsize) != 0)
            return 0;

        for (i = 0; i < n1->ndesc; i++){
            if (!eqTrees(gpc, n1->desc[i], n2->desc[i]))
                return 0;
        }
    }

    return 1;
}

static void svoGPCRemoveDuplicates(svo_gpc_t *gpc, int pop)
{
    svo_gpc_tree_t *last, *tree;
    int i;

    last = gpc->pop[pop][0];
    for (i = 1; i < gpc->pop_size[pop]; i++){
        tree = gpc->pop[pop][i];
        if (last->num_nodes == tree->num_nodes
                && last->depth == tree->depth
                && borEq(last->fitness, tree->fitness)
                && !eqTrees(gpc, last->root, tree->root)){
            svoGPCTreeDel(tree);
            gpc->pop[pop][i] = NULL;
        }else{
            last = tree;
        }
    }

    for (i = 1; i < gpc->pop_size[pop]; i++){
        if (gpc->pop[pop][i] == NULL){
            while (gpc->pop[pop][--gpc->pop_size[pop]] == NULL);
            gpc->pop[pop][i] = gpc->pop[pop][gpc->pop_size[pop]];
            gpc->pop[pop][gpc->pop_size[pop]] = NULL;
        }
    }
    svoGPCSortPop(gpc, pop);
}
