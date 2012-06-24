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

#include <svoboda/prm.h>
#include <boruvka/timer.h>
#include <boruvka/rand-mt.h>
#include <boruvka/dbg.h>

#define FREE SVO_PRM_FREE
#define OBST SVO_PRM_OBST

bor_real_t aabb[12] = { -54, 85, -45, 93, -48, 90,
                        -M_PI, M_PI, -M_PI, M_PI, -M_PI, M_PI };
svo_prm_ops_t ops;
svo_prm_params_t params;
svo_prm_t *prm;

bor_rand_mt_t *rand_mt;

bor_timer_t timer;

bor_vec_t *conf, *start, *goal;
bor_real_t step = 0.12;
size_t max_nodes;
size_t find_path = 5000;
unsigned long evals = 0;


static int terminate(void *data);
static void callback(void *data);
static const bor_vec_t *confcb(void *data);
static int eval(const bor_vec_t *conf, void *data);
static int findPath(const bor_vec_t *c1, const bor_vec_t *c2, void *data);
static void printPath(bor_list_t *path, FILE *out);


int main(int argc, char *argv[])
{
    if (argc != 3){
        fprintf(stderr, "Usage: %s max_nodes scene\n", argv[0]);
        return -1;
    }

    svoPRMOpsInit(&ops);
    svoPRMParamsInit(&params);

    params.d = 6;
    params.max_neighbors = 10;
    params.max_dist      = 5.;
    params.gug.num_cells = 0;
    params.gug.max_dens  = 1;
    params.gug.expand_rate = 1.2;
    params.gug.aabb = aabb;

    ops.data      = NULL;
    ops.conf      = confcb;
    ops.eval      = eval;
    ops.terminate = terminate;
    ops.find_path = findPath;
    ops.callback  = callback;
    ops.callback_period = 500;


    rand_mt = borRandMTNewAuto();

    prm = svoPRMNew(&ops, &params);

    max_nodes = atoi(argv[1]);

    conf  = borVecNew(6);
    start = borVecNew(6);
    goal  = borVecNew(6);

    borVecSetZero(6, start);
    borVecSet(start, 0, -2.6);
    borVecSet(start, 1, 2.34);
    borVecSet(start, 2, 0.71);
    borVecSetZero(6, goal);
    borVecSet(goal, 0, -8);
    borVecSet(goal, 1, 57);
    borVecSet(goal, 2, -1.4);

    borTimerStart(&timer);
    svoPRMRun(prm);
    callback(NULL);
    fprintf(stderr, "\n");


    //svoPRMDumpSVT(prm, stdout, "Result");

    svoPRMDel(prm);
    borRandMTDel(rand_mt);
    
    fprintf(stderr, "Evals: %ld\n", evals);
    fprintf(stdout, "# Evals: %ld\n", evals);

    return 0;
}

static int terminate(void *data)
{
    size_t nodes;
    int res;
    bor_list_t path;

    nodes = svoPRMNodesLen(prm);

    if (nodes > find_path && nodes % find_path == 0){
        borListInit(&path);
        res = svoPRMFindPath(prm, start, goal, &path);
        if (res == 0){
            fprintf(stderr, "\n");
            fprintf(stderr, "Path found. Nodes: %d\n",
                    (int)svoPRMNodesLen(prm));
            printPath(&path, stdout);

            return 1;
        }
    }

    return nodes >= max_nodes;
}

static void callback(void *data)
{
    borTimerStopAndPrintElapsed(&timer, stderr, " n: %d / %d, evals: %lu\r",
                                (int)svoPRMNodesLen(prm),
                                (int)max_nodes,
                                evals);
    fflush(stderr);
}

static const bor_vec_t *confcb(void *data)
{
    int i;

    for (i = 0; i < 6; i++){
        borVecSet(conf, i, borRandMT(rand_mt, aabb[2 * i], aabb[2 * i + 1]));
    }

    return conf;
}

static int eval(const bor_vec_t *conf, void *data)
{
    evals += 1L;
    // TODO
    //return __eval(conf, NULL);
    return FREE;
}

static int findPath(const bor_vec_t *c1, const bor_vec_t *c2, void *data)
{
    BOR_VEC(move, 6);
    BOR_VEC(c, 6);
    bor_real_t dist;
    int ev;

    borVecCopy(6, c, c1);
    borVecSub2(6, move, c2, c1);
    dist = borVecLen(6, move);
    borVecScale(6, move, step * borRecp(dist));

    while (dist > step){
        borVecAdd(6, c, move);

        ev = eval(c, data);
        if (ev != SVO_PRM_FREE)
            return 0;

        dist -= step;
    }

    return 1;
}

static void printPath(bor_list_t *path, FILE *out)
{
    bor_list_t *item;
    svo_prm_node_t *n;
    size_t id;

    fprintf(out, "------\n");
    fprintf(out, "Name: path\n");
    fprintf(out, "Edge width: 3\n");
    fprintf(out, "Edge color: 0 0 0\n");

    fprintf(out, "Points:\n");
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_prm_node_t, path);
        borVec2Print((bor_vec2_t *)n->conf, out);
        fprintf(out, "\n");
    }

    fprintf(out, "Edges:\n");
    id = 0;
    BOR_LIST_FOR_EACH(path, item){
        if (borListNext(item) == path)
            break;

        fprintf(out, "%d %d\n", (int)id, (int)id + 1);
        id++;
    }

    fprintf(out, "------\n");
}



