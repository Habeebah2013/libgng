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

#include <svoboda/ga.h>
#include <boruvka/tasks.h>
#include <boruvka/sort.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>
#include <strings.h>
#include <string.h>
#include <limits.h>

/** Performs one breeding step on population in specified range */
static void _svoGAStep(svo_ga_t *ga, size_t from, size_t to);
/** Initialize population in specified range */
static void _svoGAInit(svo_ga_t *ga, size_t from, size_t to);

/** Runs GA in threads */
static void __svoGAStepTask(int id, void *data, const bor_tasks_thinfo_t *thinfo);
static void _svoGARunThreads(svo_ga_t *ga);
/** Runs single-threaded GA */
static void _svoGARun1(svo_ga_t *ga);


/** Invididual is array organized as follows:
 *  (bor_real_t * fitness_size) | (gene_size * genotype_size)
 */
static void *svoGAIndivNew(svo_ga_t *ga);
static void svoGAIndivDel(svo_ga_t *ga, void *indiv);

#define OPS_DATA(name) \
    if (!ga->ops.name ## _data) \
        ga->ops.name ## _data = ga->ops.data;

#define OPS_CHECK(name) \
    if (!ga->ops.name){ \
        fprintf(stderr, "Fermat :: GA :: No " #name "callback set.\n"); \
        exit(-1); \
    }

#define OPS_CHECK_DATA(name) \
    OPS_DATA(name) \
    OPS_CHECK(name)



void svoGAOpsInit(svo_ga_ops_t *ops)
{
    bzero(ops, sizeof(svo_ga_ops_t));
    ops->sel       = svoGASelTournament2;
    ops->crossover = svoGACrossover2;
    ops->mutate    = svoGAMutateNone;
}

void svoGAParamsInit(svo_ga_params_t *p)
{
    p->pc             = 0.7;
    p->pm             = 0.3;
    p->gene_size      = 1;
    p->genotype_size  = 1;
    p->pop_size       = 1;
    p->fitness_size   = 1;
    p->crossover_size = 2;
    p->presel_max     = 10;
    p->threads        = 1;
}

svo_ga_t *svoGANew(const svo_ga_ops_t *ops, const svo_ga_params_t *params)
{
    svo_ga_t *ga;
    size_t i;

    ga = BOR_ALLOC(svo_ga_t);
    ga->tid = -1;
    ga->params = *params;

    ga->ops = *ops;
    OPS_CHECK_DATA(eval)
    OPS_CHECK_DATA(terminate)
    OPS_CHECK_DATA(init)
    OPS_CHECK_DATA(sel)
    OPS_CHECK_DATA(crossover)
    OPS_CHECK_DATA(mutate)
    OPS_DATA(callback)


    // allocate populations
    ga->pop[0] = BOR_ALLOC_ARR(void *, ga->params.pop_size);
    ga->pop[1] = BOR_ALLOC_ARR(void *, ga->params.pop_size);
    for (i = 0; i < ga->params.pop_size; i++){
        ga->pop[0][i] = svoGAIndivNew(ga);
        ga->pop[1][i] = svoGAIndivNew(ga);
    }
    ga->pop_cur = 0;

    ga->gt[0] = ga->gt[1] = NULL;
    ga->ft[0] = ga->ft[1] = NULL;

    ga->presel = BOR_ALLOC_ARR(size_t, ga->params.presel_max);

    ga->rand = borRandMTNewAuto();

    return ga;
}

void svoGADel(svo_ga_t *ga)
{
    size_t i;

    for (i = 0; i < ga->params.pop_size; i++){
        svoGAIndivDel(ga, ga->pop[0][i]);
        svoGAIndivDel(ga, ga->pop[1][i]);
    }
    BOR_FREE(ga->pop[0]);
    BOR_FREE(ga->pop[1]);

    BOR_FREE(ga->presel);

    borRandMTDel(ga->rand);

    BOR_FREE(ga);
}

void svoGARun(svo_ga_t *ga)
{
    if (ga->params.threads > 1){
        _svoGARunThreads(ga);
    }else{
        _svoGARun1(ga);
    }
}


size_t svoGASelTournament2(svo_ga_t *ga, void *data)
{
    size_t t[2];
    bor_real_t f[2];

    t[0] = svoGARandInt(ga, 0, ga->params.pop_size);
    t[1] = svoGARandInt(ga, 0, ga->params.pop_size);
    f[0] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][t[0]])[0];
    f[1] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][t[1]])[0];

    if (f[0] > f[1])
        return t[0];
    return t[1];
}

size_t svoGASelTournament3(svo_ga_t *ga, void *data)
{
    size_t t[3];
    bor_real_t f[3];

    t[0] = svoGARandInt(ga, 0, ga->params.pop_size);
    t[1] = svoGARandInt(ga, 0, ga->params.pop_size);
    t[2] = svoGARandInt(ga, 0, ga->params.pop_size);
    f[0] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][t[0]])[0];
    f[1] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][t[1]])[0];
    f[2] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][t[2]])[0];

    if (f[0] > f[1]){
        if (f[0] > f[2])
            return t[0];
        return t[2];
    }else{
        if (f[1] > f[2])
            return t[1];
        return t[2];
    }
}

void svoGACrossover2(svo_ga_t *ga, void **ing, void **outg, void *data)
{
    int cross = svoGARandInt(ga, 0, ga->params.genotype_size - 1);
    size_t size1, size2;

    // size of first and second half
    size1 = ga->params.gene_size * (cross + 1);
    size2 = ga->params.gene_size * (ga->params.genotype_size - cross - 1);

    memcpy(outg[0], ing[0], size1);
    memcpy((char *)outg[0] + size1, (char *)ing[1] + size1, size2);

    if (outg[1]){
        memcpy(outg[1], ing[1], size1);
        memcpy((char *)outg[1] + size1, (char *)ing[0] + size1, size2);
    }
}

void svoGAMutateNone(svo_ga_t *ga, void *gt, void *data)
{
}

size_t svoGAPreselElite(svo_ga_t *ga, size_t *sel, void *data)
{
    bor_radix_sort_t *rs, *rstmp;
    void *indiv;
    size_t i;

    if (ga->params.presel_max == 0)
        return 0;

    rs = BOR_ALLOC_ARR(bor_radix_sort_t, ga->params.pop_size);
    rstmp = BOR_ALLOC_ARR(bor_radix_sort_t, ga->params.pop_size);
    for (i = 0; i < ga->params.pop_size; i++){
        indiv = svoGAIndiv(ga, i);
        rs[i].key = svoGAIndivFitness(ga, indiv)[0];
        rs[i].val = i;
    }
    borRadixSort(rs, rstmp, ga->params.pop_size);

    for (i = 0; i < ga->params.presel_max; i++){
        sel[i] = rs[ga->params.pop_size - 1 - i].val;
    }

    BOR_FREE(rs);
    BOR_FREE(rstmp);
    return ga->params.presel_max;
}



void __svoGATRandRefill(svo_ga_t *ga)
{
    size_t i;

    pthread_mutex_lock(ga->tlock);
    for (i = 0; i < ga->trand_max; i++){
        ga->trand[i] = borRandMT01(ga->rand);
    }
    pthread_mutex_unlock(ga->tlock);

    ga->trand_next = 0;
}






static void _svoGAStep(svo_ga_t *ga, size_t from, size_t to)
{
    size_t ind, i, sel;


    ind = from;
    while (ind < to){
        // selection
        for (i = 0; i < ga->params.crossover_size; i++){
            sel = ga->ops.sel(ga, ga->ops.sel_data);
            ga->gt[0][i] = svoGAIndivGenotype(ga, ga->pop[ga->pop_cur][sel]);
            ga->ft[0][i] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][sel]);

            if (ind < to){
                ga->gt[1][i] = svoGAIndivGenotype(ga, ga->pop[ga->pop_cur ^ 1][ind]);
                ga->ft[1][i] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur ^ 1][ind]);
                ++ind;
            }
            //DBG("[%d] %d", (int)i, (int)sel);
            //DBG("Sel: %d %d", (int)sel[0], (int)sel[1]);
        }

        // crossover
        if (svoGARand01(ga) < ga->params.pc){
            ga->ops.crossover(ga, ga->gt[0], ga->gt[1], ga->ops.crossover_data);
        }else{
            for (i = 0; i < ga->params.crossover_size && ga->gt[1][i]; i++){
                memcpy(ga->gt[1][i], ga->gt[0][i],
                       ga->params.gene_size * ga->params.genotype_size);
            }
        }

        // mutation
        for (i = 0; i < ga->params.crossover_size && ga->gt[1][i]; i++){
            if (svoGARand01(ga) < ga->params.pm){
                ga->ops.mutate(ga, ga->gt[1][i], ga->ops.mutate_data);
            }
        }

        // eval
        for (i = 0; i < ga->params.crossover_size && ga->gt[1][i]; i++){
            ga->ops.eval(ga, ga->gt[1][i], ga->ft[1][i], ga->ops.eval_data);
        }
    }
}

static void _svoGAInit(svo_ga_t *ga, size_t from, size_t to)
{
    size_t i;

    for (i = from; i < to; ++i){
        ga->gt[0][0] = svoGAIndivGenotype(ga, ga->pop[ga->pop_cur][i]);
        ga->ft[0][0] = svoGAIndivFitness(ga, ga->pop[ga->pop_cur][i]);

        ga->ops.init(ga, ga->gt[0][0], ga->ops.init_data);
        ga->ops.eval(ga, ga->gt[0][0], ga->ft[0][0], ga->ops.eval_data);
    }
}

static size_t _svoGAPreselect(svo_ga_t *ga)
{
    size_t i, len;
    void *f, *t;
    bor_real_t *ft;

    if (!ga->ops.presel)
        return 0;

    len = ga->ops.presel(ga, ga->presel, ga->ops.presel_data);
    for (i = 0; i < len; i++){
        t  = svoGAIndivGenotype(ga, ga->pop[ga->pop_cur ^ 1][i]);
        ft = svoGAIndivFitness(ga, ga->pop[ga->pop_cur ^ 1][i]);
        f = svoGAIndivGenotype(ga, ga->pop[ga->pop_cur][ga->presel[i]]);

        memcpy(t, f, ga->params.gene_size * ga->params.genotype_size);
        ga->ops.eval(ga, t, ft, ga->ops.eval_data);
    }

    return len;
}

static void __svoGAStepTask(int id, void *data, const bor_tasks_thinfo_t *thinfo)
{
    svo_ga_t *ga = (svo_ga_t *)data;
    _svoGAStep(ga, ga->tfrom, ga->tto);
}

static void _svoGARunThreads(svo_ga_t *ga)
{
    unsigned long cb = 0UL;

    bor_tasks_t *tasks;
    svo_ga_t *gas;
    int i, poplen, popfrom;


    // alloc temporary memory
    ga->gt[0] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
    ga->gt[1] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
    ga->ft[0] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);
    ga->ft[1] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);
    ga->tlock = BOR_ALLOC(pthread_mutex_t);
    pthread_mutex_init(ga->tlock, NULL);

    gas = BOR_ALLOC_ARR(svo_ga_t, ga->params.threads);
    for (i = 0; i < ga->params.threads; i++){
        gas[i] = *ga;

        // set up thread specific data
        gas[i].tid   = i;

        // alloc temporary memory
        gas[i].gt[0] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
        gas[i].gt[1] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
        gas[i].ft[0] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);
        gas[i].ft[1] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);

        // prepare array for random numbers
        gas[i].trand_max = ga->params.pop_size * 5;
        gas[i].trand = BOR_ALLOC_ARR(bor_real_t, gas[i].trand_max);
        gas[i].trand_next = 0;
        __svoGATRandRefill(&gas[i]);
    }

    // create thread pool
    tasks = borTasksNew(ga->params.threads);
    borTasksRun(tasks);


    // initialize individuals
    _svoGAInit(ga, 0, ga->params.pop_size);

    while (!ga->ops.terminate(ga, ga->ops.terminate_data)){
        cb += 1UL;
        if (cb == ga->ops.callback_period && ga->ops.callback){
            ga->ops.callback(ga, ga->ops.callback_data);
            cb = 0UL;
        }

        // preselect
        popfrom = _svoGAPreselect(ga);

        // one step of breeding
        poplen  = (ga->params.pop_size - popfrom) / ga->params.crossover_size;
        poplen /= ga->params.threads;
        poplen *= ga->params.crossover_size;

        for (i = 0; i < ga->params.threads; i++){
            // set up range for specific thread
            gas[i].tfrom = (i * poplen) + popfrom;
            gas[i].tto   = gas[i].tfrom + poplen;
            if (i == ga->params.threads - 1)
                gas[i].tto = ga->params.pop_size;

            borTasksAdd(tasks, __svoGAStepTask, i, (void *)&gas[i]);
        }
        borTasksBarrier(tasks);

        ga->pop_cur ^= 1;
        for (i = 0; i < ga->params.threads; i++){
            gas[i].pop_cur ^= 1;
        }
    }


    // delete thread pool
    borTasksDel(tasks);

    for (i = 0; i < ga->params.threads; i++){
        // free tmp memory
        BOR_FREE(gas[i].gt[0]);
        BOR_FREE(gas[i].gt[1]);
        BOR_FREE(gas[i].ft[0]);
        BOR_FREE(gas[i].ft[1]);

        gas[i].gt[0] = gas[i].gt[1] = NULL;
        gas[i].ft[0] = gas[i].ft[1] = NULL;

        BOR_FREE(gas[i].trand);
    }

    BOR_FREE(gas);

    pthread_mutex_destroy(ga->tlock);
    BOR_FREE(ga->tlock);
    // free tmp memory
    BOR_FREE(ga->gt[0]);
    BOR_FREE(ga->gt[1]);
    BOR_FREE(ga->ft[0]);
    BOR_FREE(ga->ft[1]);
}

static void _svoGARun1(svo_ga_t *ga)
{
    unsigned long cb = 0UL;
    size_t popfrom;

    // alloc temporary memory
    ga->gt[0] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
    ga->gt[1] = BOR_ALLOC_ARR(void *, ga->params.crossover_size);
    ga->ft[0] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);
    ga->ft[1] = BOR_ALLOC_ARR(bor_real_t *, ga->params.crossover_size);

    // initialize individuals
    _svoGAInit(ga, 0, ga->params.pop_size);

    do {
        cb += 1UL;
        if (cb == ga->ops.callback_period && ga->ops.callback){
            ga->ops.callback(ga, ga->ops.callback_data);
            cb = 0UL;
        }

        // preselect
        popfrom = _svoGAPreselect(ga);

        // one step of breeding
        _svoGAStep(ga, popfrom, ga->params.pop_size);

        ga->pop_cur ^= 1;
    } while (!ga->ops.terminate(ga, ga->ops.terminate_data));

    // free tmp memory
    BOR_FREE(ga->gt[0]);
    BOR_FREE(ga->gt[1]);
    BOR_FREE(ga->ft[0]);
    BOR_FREE(ga->ft[1]);

    ga->gt[0] = ga->gt[1] = NULL;
    ga->ft[0] = ga->ft[1] = NULL;
}




static void *svoGAIndivNew(svo_ga_t *ga)
{
    size_t size;
    void *indiv;

    size = sizeof(bor_real_t) * ga->params.fitness_size;
    size += ga->params.gene_size * ga->params.genotype_size;

    indiv = borRealloc(NULL, size);

    return indiv;
}

static void svoGAIndivDel(svo_ga_t *ga, void *indiv)
{
    BOR_FREE(indiv);
}
