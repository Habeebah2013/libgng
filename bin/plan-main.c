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

#include <svoboda/gnnp.h>
#include <svoboda/rrt.h>
#include <boruvka/timer.h>
#include <boruvka/cfg.h>
#include <boruvka/opts.h>
#include <boruvka/dbg.h>
#include <boruvka/rand-mt.h>
#include <boruvka/quat.h>
#include "cfg-map.h"


bor_vec_t *is;
bor_vec_t *init, *goal;
unsigned long evals = 0UL;


int list_robots;
const char *robot_name = NULL;
const char *method_name = NULL;
long callback_period = 0;
bor_real_t max_time = 3600.;
bor_real_t elapsed_time = 0.;
int rmax = 0;
bor_real_t h = 0.25;
int use_rot = 0;
int dbg_dump = 0;
int alg_num = 0;
int rrt_goal_conf = 1000;
bor_timer_t timer;
bor_rand_mt_t *rnd;


static void setUpNN(bor_nn_params_t *nn);
_bor_inline void updateTimer(void);


static svo_gnnp_t *gnnp = NULL;
static int gnnpTerminate(svo_gnnp_t *nn, void *data);
static const bor_vec_t *gnnpInputSignal(svo_gnnp_t *nn, void *data);
static int gnnpEval(svo_gnnp_t *nn, const bor_vec_t *conf, void *data);
static void gnnpCallback(svo_gnnp_t *nn, void *data);
static void gnnpInit(void);
static void gnnpDestroy(void);
static int gnnpRun(bor_list_t *path);
static void gnnpDump(int ret, bor_list_t *path);
static long gnnpNodesLen(void);


static svo_rrt_t *rrt = NULL;
static int rrtTerminate(const svo_rrt_t *rrt, void *data);
static const bor_vec_t *rrtExpand(const svo_rrt_t *rrt,
                                  const svo_rrt_node_t *n,
                                  const bor_vec_t *rand, void *data);
static const bor_vec_t *rrtConf(const svo_rrt_t *rrt, void *data);
static void rrtCallback(const svo_rrt_t *rrt, void *data);
static void rrtInit(void);
static void rrtDestroy(void);
static int rrtRun(bor_list_t *path);
static void rrtDump(int ret, bor_list_t *path);
static long rrtNodesLen(void);

static int rrtConnectTerminateExpand(const svo_rrt_t *rrt,
                                     const svo_rrt_node_t *start,
                                     const svo_rrt_node_t *last,
                                     const bor_vec_t *rand,
                                     void *data);
static void rrtConnectInit(void);
static int rrtConnectRun(bor_list_t *path);


static void rrtBlossomExpandAll(const svo_rrt_t *rrt,
                                const svo_rrt_node_t *n,
                                const bor_vec_t *conf,
                                void *data,
                                bor_list_t *list_out);
static void rrtBlossomInit(void);
static int rrtBlossomRun(bor_list_t *path);

static int rrtBlossomFilter(const svo_rrt_t *rrt,
                            const bor_vec_t *candidate,
                            const svo_rrt_node_t *src,
                            const svo_rrt_node_t *nearest,
                            void *data);
static void rrtBlossomFilterInit(void);

struct alg_t {
    void (*init)(void);
    void (*destroy)(void);
    int (*run)(bor_list_t *path);
    void (*dump)(int ret, bor_list_t *path);
    long (*nodes_len)(void);
};
#define ALG_GNNP        0
#define ALG_RRT         1
#define ALG_RRT_CONNECT 2
#define ALG_RRT_BLOSSOM 3
#define ALG_RRT_BLOSSOM_FILTER 4
#define ALG_LEN         5

#define ALG_DEF(name) \
    { .init      = name ## Init, \
      .destroy   = name ## Destroy, \
      .run       = name ## Run, \
      .dump      = name ## Dump, \
      .nodes_len = name ## NodesLen }
const char *methods[ALG_LEN] = { "gnnp", "rrt", "rrt-connect", "rrt-blossom", "rrt-blossom-filter" };
struct alg_t algs[ALG_LEN] = {
    ALG_DEF(gnnp),
    ALG_DEF(rrt),

    { .init      = rrtConnectInit, \
      .destroy   = rrtDestroy, \
      .run       = rrtConnectRun, \
      .dump      = rrtDump, \
      .nodes_len = rrtNodesLen },

    { .init      = rrtBlossomInit, \
      .destroy   = rrtDestroy, \
      .run       = rrtBlossomRun, \
      .dump      = rrtDump, \
      .nodes_len = rrtNodesLen },

    { .init      = rrtBlossomFilterInit, \
      .destroy   = rrtDestroy, \
      .run       = rrtBlossomRun, \
      .dump      = rrtDump, \
      .nodes_len = rrtNodesLen }
};

int opts(int *argc, char *argv[])
{
    int i, help;
    int ok = 1;

    borOptsAddDesc("help", 0x0, BOR_OPTS_NONE, (void *)&help, NULL,
                   "Print this help");
    borOptsAddDesc("robot", 'r', BOR_OPTS_STR, (void *)&robot_name, NULL,
                   "The robot with the specified name will be used (default: none)");
    borOptsAddDesc("method", 'm', BOR_OPTS_STR, (void *)&method_name, NULL,
                   "Choose the planning method: gnnp, rrt, rrt-connect (default: gnnp)");
    borOptsAddDesc("max-time", 0x0, BOR_OPTS_REAL, (void *)&max_time, NULL,
                   "Maximal time in seconds (default: 3600)");
    borOptsAddDesc("rmax", 0x0, BOR_OPTS_INT, (void *)&rmax, NULL,
                   "Rmax parameter (default: 2^(dim + 1))");
    borOptsAddDesc("rot", 0x0, BOR_OPTS_NONE, (void *)&use_rot, NULL,
                   "Also rotation is considered");
    borOptsAddDesc("robots", 0x0, BOR_OPTS_NONE, (void *)&list_robots, NULL,
                   "Print list of available robots");
    borOptsAddDesc("cb-period", 0x0, BOR_OPTS_LONG, (void *)&callback_period, NULL,
                   "Callback period");
    borOptsAddDesc("dbg-dump", 0x0, BOR_OPTS_NONE, (void *)&dbg_dump, NULL,
                   "Enables debug dumps into dbg/ directory in each callback");
    borOptsAddDesc("rrt-goal-conf", 0x0, BOR_OPTS_INT, (void *)&rrt_goal_conf, NULL,
                   "How often should be goal configuration presented to the algorithm (default 1000)");


    if (borOpts(argc, argv) != 0)
        ok = 0;

    if (method_name){
        for (i = 0; i < ALG_LEN; i++){
            if (strcmp(method_name, methods[i]) == 0){
                alg_num = i;
                break;
            }
        }
    }

    if (callback_period == 0){
        callback_period = 10000;
        if (alg_num == ALG_GNNP){
            callback_period = 100000;
        }
    }


    if (help || !ok || *argc != 2 || (!list_robots && (robot_name == NULL || method_name == NULL))){
        fprintf(stderr, "Usage: %s [ OPTIONS ] [-r robot | --robots] -m method cfg_file\n", argv[0]);
        fprintf(stderr, "  OPTIONS:\n");
        borOptsPrint(stderr, "    ");
        return -1;
    }

    if (ok)
        return 0;
    return -1;
}

int main(int argc, char *argv[])
{
    bor_list_t path;
    int ret = 0;

    if (opts(&argc, argv) != 0)
        return -1;

    if (borCfgMapInit(argv[1]) != 0)
        return -1;

    if (use_rot || borCfgMapDim() == 3){
        borCfgMapUseRot();
    }

    if (rmax == 0){
        rmax = powl(2, borCfgMapConfDim() + 1);
    }

    if (list_robots){
        borCfgMapListRobots(stdout);
        borCfgMapDestroy();
        return 0;
    }

    is   = borVecNew(6);
    init = borVecNew(6);
    goal = borVecNew(6);
    borVecSetZero(6, is);
    borVecSetZero(6, init);
    borVecSetZero(6, goal);

    // load robot with parameters
    if (borCfgMapRobot(robot_name, &h, init, goal) != 0){
        borVecDel(is);
        borVecDel(init);
        borVecDel(goal);
        borCfgMapDestroy();
        return -1;
    }

    if (borCfgMapCollide(init)
            || borCfgMapCollide(goal)){
        if (borCfgMapCollide(init))
            fprintf(stderr, "Error: init configuration is OBST.\n");
        if (borCfgMapCollide(goal))
            fprintf(stderr, "Error: goal configuration is OBST.\n");

        if (dbg_dump){
            borCfgMapDumpSVT(stdout, NULL);
            borCfgMapRobotDumpSVT(init, stdout, "Init");
            borCfgMapRobotDumpSVT(goal, stdout, "Goal");
        }

        borVecDel(is);
        borVecDel(init);
        borVecDel(goal);
        borCfgMapDestroy();
        return -1;
    }

    rnd = borRandMTNewAuto();


    algs[alg_num].init();

    borListInit(&path);
    borTimerStart(&timer);
    ret = algs[alg_num].run(&path);
    borTimerStop(&timer);
    elapsed_time += borTimerElapsedInSF(&timer);

    fprintf(stderr, "ret: %d\n", ret);
    fprintf(stderr, "nodes: %ld\n", algs[alg_num].nodes_len());
    fprintf(stderr, "evals: %ld\n", (long)evals);
    fprintf(stderr, "Time: %f s\n", (float)elapsed_time);
    fflush(stderr);

    algs[alg_num].dump(ret, &path);
    algs[alg_num].destroy();


    borVecDel(is);
    borVecDel(init);
    borVecDel(goal);

    borRandMTDel(rnd);

    borCfgMapDestroy();

    return 0;
}

static void setUpNN(bor_nn_params_t *nn)
{
    nn->type = BOR_NN_GUG;

    nn->gug.dim = borCfgMapConfDim();
    nn->gug.max_dens = 1.;
    nn->gug.expand_rate = 1.3;
    nn->gug.aabb = (bor_real_t *)borCfgMapAABB();

    if (borCfgMapConfDim() == 6)
        nn->type = BOR_NN_VPTREE;
}

_bor_inline void updateTimer(void)
{
    borTimerStop(&timer);
    elapsed_time += borTimerElapsedInSF(&timer);
    borTimerStart(&timer);
}

/*** GNNP ***/
static int gnnpTerminate(svo_gnnp_t *nn, void *data)
{
    //updateTimer();
    if (elapsed_time > max_time)
        return 1;
    return 0;
}

static const bor_vec_t *gnnpInputSignal(svo_gnnp_t *nn, void *data)
{
    return borCfgMapConf();
}

static int gnnpEval(svo_gnnp_t *nn, const bor_vec_t *conf, void *data)
{
    evals += 1UL;
    return !borCfgMapCollide(conf);
}

static void gnnpDumpDBG(svo_gnnp_t *nn, int c)
{
    char fn[100];
    FILE *fout;

    snprintf(fn, 100, "dbg/map-%06d.svt", c);
    fout = fopen(fn, "w");
    if (fout){
        svoGNNPDumpSVT(nn, fout, NULL);
        borCfgMapDumpSVT(fout, NULL);
        borCfgMapRobotDumpSVT(init, fout, "Init");
        borCfgMapRobotDumpSVT(goal, fout, "Goal");
        fclose(fout);
    }
}

static void gnnpCallback(svo_gnnp_t *nn, void *data)
{
    static int c = 0;

    if (dbg_dump)
        gnnpDumpDBG(nn, c);

    updateTimer();
    fprintf(stderr, "step %d, nodes: %d, evals: %ld  [%f s]\n",
            c, (int)svoGNNPNodesLen(nn), (long)evals,
            (float)elapsed_time);

    c++;
}

static void gnnpPrintPath(svo_gnnp_t *nn, bor_list_t *path)
{
    bor_list_t *item;
    svo_gnnp_node_t *n;
    int i, len;

    if (borCfgMapConfDim() > 3){
        BOR_LIST_FOR_EACH(path, item){
            n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);
            printf("#P: ");
            borVecPrint(borCfgMapConfDim(), n->w, stdout);
            printf("\n");
        }
        return;
    }

    printf("----\n");
    printf("Name: PATH\n");

    printf("Points off: 1\n");
    printf("Edge color: 1 0 0\n");
    printf("Edge width: 2\n");
    printf("Points:\n");
    len = 0;
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);
        borVecPrint(borCfgMapConfDim(), n->w, stdout);
        printf("\n");
        len++;
    }

    printf("Edges:\n");
    for (i = 0; i < len - 1; i++){
        printf("%d %d\n", i, i + 1);
    }

    printf("----\n");
    printf("----\n");
}

static void gnnpPrintSolutionVideoPath(bor_list_t *path, bor_list_t *end, FILE *out)
{
    bor_list_t *item;
    svo_gnnp_node_t *n;
    int i, len;

    if (borListNext(path) == end)
        return;

    fprintf(out, "----\n");
    fprintf(out, "Name: PATH\n");
    fprintf(out, "Points off: 1\n");
    fprintf(out, "Edge color: 1 0 0\n");
    fprintf(out, "Edge width: 1\n");

    fprintf(out, "Points:\n");
    len = 0;
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);

        borVecPrint(2, n->w, out);
        fprintf(out, "\n");

        len++;
        if (item == end)
            break;
    }

    fprintf(out, "Edges:\n");
    for (i = 0; i < len - 1; i++){
        fprintf(out, "%d %d\n", i, i + 1);
    }

    fprintf(out, "----\n");
}

static void gnnpPrintSolutionVideo(svo_gnnp_t *nn, bor_list_t *path)
{
    char fn[300];
    bor_list_t *item;
    svo_gnnp_node_t *n;
    int i;
    FILE *out;


    i = 0;
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_gnnp_node_t, path);
        snprintf(fn, 300, "gen-video/map-%06d.svt", i);
        out = fopen(fn, "w");
        if (out){
            borCfgMapDumpSVT(out, "Map");
            borCfgMapRobotDumpSVT(n->w, out, "Robot");
            gnnpPrintSolutionVideoPath(path, item, out);
            fclose(out);
        }
        i++;
    }
}

static void gnnpInit(void)
{
    svo_gnnp_ops_t ops;
    svo_gnnp_params_t params;

    svoGNNPOpsInit(&ops);
    ops.terminate    = gnnpTerminate;
    ops.input_signal = gnnpInputSignal;
    ops.eval         = gnnpEval;
    ops.callback        = gnnpCallback;
    ops.callback_period = callback_period;

    svoGNNPParamsInit(&params);
    params.dim  = borCfgMapConfDim();
    params.rmax = rmax;
    params.h    = h;
    setUpNN(&params.nn);

    gnnp = svoGNNPNew(&ops, &params);
}

static void gnnpDestroy(void)
{
    svoGNNPDel(gnnp);
}

static int gnnpRun(bor_list_t *path)
{
    return svoGNNPFindPath(gnnp, init, goal, path);
}

static void gnnpDump(int ret, bor_list_t *path)
{
    if (ret == 0){
        gnnpPrintPath(gnnp, path);
        gnnpPrintSolutionVideo(gnnp, path);
    }
    svoGNNPDumpSVT(gnnp, stdout, NULL);
    borCfgMapDumpSVT(stdout, NULL);
    borCfgMapRobotDumpSVT(init, stdout, "Init");
    borCfgMapRobotDumpSVT(goal, stdout, "Goal");
}

static long gnnpNodesLen(void)
{
    return svoGNNPNodesLen(gnnp);
}
/*** GNNP END ***/


/*** RRT ***/
static const svo_rrt_node_t *rrt_last = NULL;
static bor_real_t h2 = 0.;
static BOR_VEC(rrt_move, 6);
static BOR_VEC(rrt_new_conf, 6);
static bor_real_t near_dist = BOR_REAL_MAX;
static int rrt_found = 0;
static int rrtTerminate(const svo_rrt_t *rrt, void *data)
{
    const bor_vec_t *lconf;
    bor_real_t dist;

    //updateTimer();
    if (elapsed_time > max_time)
        return 1;

    rrt_last = svoRRTNodeLast(rrt);
    lconf = svoRRTNodeConf(rrt_last);
    dist = borVecDist2(borCfgMapConfDim(), lconf, goal);
    if (dist < near_dist)
        near_dist = dist;
    if (dist < h2){
        rrt_found = 1;
        return 1;
    }

    return 0;
}

static const bor_vec_t *rrtExpand(const svo_rrt_t *rrt,
                                  const svo_rrt_node_t *n,
                                  const bor_vec_t *rand, void *data)
{
    const bor_vec_t *near;
    bor_real_t len;

    near = svoRRTNodeConf(n);

    borVecSub2(rrt->params.dim, rrt_move, rand, near);
    len = borVecLen(rrt->params.dim, rrt_move);
    borVecScale(rrt->params.dim, rrt_move, h * borRecp(len));

    borVecAdd2(rrt->params.dim, rrt_new_conf, near, rrt_move);

    evals += 1UL;
    if (borCfgMapCollide(rrt_new_conf))
        return NULL;
    return rrt_new_conf;
}

static const bor_vec_t *rrtConf(const svo_rrt_t *rrt, void *data)
{
    static int counter = 0;
    if (counter++ == rrt_goal_conf){
        counter = 0;
        return goal;
    }
    return borCfgMapConf();
}

static void rrtDumpDBG(int c)
{
    char fn[100];
    FILE *fout;

    snprintf(fn, 100, "dbg/map-%06d.svt", c);
    fout = fopen(fn, "w");
    if (fout){
        svoRRTDumpSVT(rrt, fout, NULL);
        borCfgMapDumpSVT(fout, NULL);
        borCfgMapRobotDumpSVT(init, fout, "Init");
        borCfgMapRobotDumpSVT(goal, fout, "Goal");
        fclose(fout);
    }
}

static void rrtCallback(const svo_rrt_t *r, void *data)
{
    static unsigned long c = 0UL;

    if (dbg_dump){
        rrtDumpDBG(c);
    }

    updateTimer();
    fprintf(stderr, "step %ld, nodes: %d, evals: %ld, nearest: %f  [%f s]\n",
            c, (int)svoRRTNodesLen(rrt), (long)evals, BOR_SQRT(near_dist),
            (float)elapsed_time);
    c++;
}

static void rrtInit(void)
{
    svo_rrt_ops_t ops;
    svo_rrt_params_t params;

    svoRRTOpsInit(&ops);
    ops.terminate = rrtTerminate;
    ops.expand    = rrtExpand;
    ops.random    = rrtConf;
    ops.callback  = rrtCallback;
    ops.callback_period = callback_period;

    svoRRTParamsInit(&params);
    params.dim = borCfgMapConfDim();
    setUpNN(&params.nn);

    rrt = svoRRTNew(&ops, &params);

    h2 = h * h;
}

static void rrtDestroy(void)
{
    svoRRTDel(rrt);
}

static int rrtRun(bor_list_t *path)
{
    int ret;
    const svo_rrt_node_t *init_node;

    svoRRTRunBasic(rrt, init);
    init_node = svoRRTNodeInitial(rrt);
    if (rrt_found){
        borListInit(path);
        ret = svoRRTFindPath(rrt, init_node, rrt_last, path);
        return ret;
    }

    return -1;
}

static void rrtPrintPath(bor_list_t *path, FILE *out)
{
    bor_list_t *item;
    const svo_rrt_node_t *last_node;
    svo_rrt_node_t *n;
    size_t id;

    if (borCfgMapConfDim() > 3){
        BOR_LIST_FOR_EACH(path, item){
            n = BOR_LIST_ENTRY(item, svo_rrt_node_t, path);
            fprintf(out, "#P: ");
            borVecPrint(rrt->params.dim, n->conf, out);
            fprintf(out, "\n");
        }
        return;
    }

    last_node = svoRRTNodeLast(rrt);
    svoRRTNodeNew(rrt, goal, last_node);

    fprintf(out, "------\n");
    fprintf(out, "Name: PATH\n");
    fprintf(out, "Edge width: 3\n");
    fprintf(out, "Edge color: 0.8 0 0\n");

    fprintf(out, "Points:\n");
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_rrt_node_t, path);
        borVecPrint(rrt->params.dim, n->conf, out);
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

static void rrtPrintSolutionVideo(bor_list_t *path)
{
    char fn[300];
    bor_list_t *item;
    svo_rrt_node_t *n;
    int i;
    FILE *out;

    i = 0;
    BOR_LIST_FOR_EACH(path, item){
        n = BOR_LIST_ENTRY(item, svo_rrt_node_t, path);
        snprintf(fn, 300, "rrt-gen-video/map-%06d.svt", i);
        out = fopen(fn, "w");
        if (out){
            borCfgMapDumpSVT(out, "Map");
            borCfgMapRobotDumpSVT(n->conf, out, "Robot");
            fclose(out);
        }
        i++;
    }
}
static void rrtDump(int ret, bor_list_t *path)
{
    if (ret == 0){
        rrtPrintPath(path, stdout);
        rrtPrintSolutionVideo(path);
    }
    svoRRTDumpSVT(rrt, stdout, NULL);
    borCfgMapDumpSVT(stdout, NULL);
    borCfgMapRobotDumpSVT(init, stdout, "Init");
    borCfgMapRobotDumpSVT(goal, stdout, "Goal");
}

static long rrtNodesLen(void)
{
    return svoRRTNodesLen(rrt);
}

/*** RRT END ***/

/*** RRT CONNECT ***/
static int rrtConnectTerminateExpand(const svo_rrt_t *rrt,
                                     const svo_rrt_node_t *start,
                                     const svo_rrt_node_t *last,
                                     const bor_vec_t *rand,
                                     void *data)
{
    const bor_vec_t *n;
    bor_real_t dist;

    n = svoRRTNodeConf(last);
    dist = borVecDist2(rrt->params.dim, n, rand);

    return dist <= h2;
}

static void rrtConnectInit(void)
{
    svo_rrt_ops_t ops;
    svo_rrt_params_t params;

    svoRRTOpsInit(&ops);
    ops.terminate = rrtTerminate;
    ops.terminate_expand = rrtConnectTerminateExpand;
    ops.expand    = rrtExpand;
    ops.random    = rrtConf;
    ops.callback  = rrtCallback;
    ops.callback_period = callback_period;

    svoRRTParamsInit(&params);
    params.dim = borCfgMapConfDim();
    setUpNN(&params.nn);

    rrt = svoRRTNew(&ops, &params);

    h2 = h * h;
}

static int rrtConnectRun(bor_list_t *path)
{
    int ret;
    const svo_rrt_node_t *init_node;

    svoRRTRunConnect(rrt, init);
    init_node = svoRRTNodeInitial(rrt);
    borListInit(path);
    ret = svoRRTFindPath(rrt, init_node, rrt_last, path);
    return ret;
}
/*** RRT CONNECT END ***/


/*** RRT BLOSSOM ***/
BOR_VEC(rrt_move2, 6);
static void rrtBlossomExpandAll(const svo_rrt_t *rrt,
                                const svo_rrt_node_t *n,
                                const bor_vec_t *c,
                                void *data,
                                bor_list_t *list_out)
{
    const bor_vec_t *near;
    bor_real_t len, angle[3];
    bor_quat_t rot3d;
    int i;

    near = svoRRTNodeConf(n);

    borVecSub2(rrt->params.dim, rrt_move, c, near);
    len = borVecLen(rrt->params.dim, rrt_move);
    borVecScale(rrt->params.dim, rrt_move, h * borRecp(len));

    /*
    borVecAdd2(rrt->params.dim, rrt_new_conf, near, rrt_move);
    evals += 1UL;
    if (!borCfgMapCollide(rrt_new_conf))
        svoRRTExpandAdd(rrt->params.dim, rrt_new_conf, list_out);
    */

    if (borCfgMapConfDim() == 2){
        for (i = 0; i < 3; i++){
            //angle = 0.05;
            angle[0] = borRandMT(rnd, -M_PI_2, M_PI_2);
            borVec2Rot2((bor_vec2_t *)rrt_move2, (const bor_vec2_t *)rrt_move, angle[0]);
            borVecAdd2(rrt->params.dim, rrt_new_conf, near, rrt_move2);
            evals += 1UL;
            if (!borCfgMapCollide(rrt_new_conf))
                svoRRTExpandAdd(rrt->params.dim, rrt_new_conf, list_out);
        }
    }else if (borCfgMapConfDim() == 3){
        for (i = 0; i < 5; i++){
            //angle = 0.05;
            angle[0] = borRandMT(rnd, -M_PI_2, M_PI_2);
            angle[1] = borRandMT(rnd, -M_PI_2, M_PI_2);
            angle[2] = borRandMT(rnd, -M_PI_2, M_PI_2);
            borQuatSetEuler(&rot3d, angle[0], angle[1], angle[2]);
            borVecCopy(rrt->params.dim, rrt_move2, rrt_move);
            borQuatRotVec((bor_vec3_t *)rrt_move2, &rot3d);
            borVecAdd2(rrt->params.dim, rrt_new_conf, near, rrt_move2);
            evals += 1UL;
            if (!borCfgMapCollide(rrt_new_conf))
                svoRRTExpandAdd(rrt->params.dim, rrt_new_conf, list_out);
        }
    }
}


static void rrtBlossomInit(void)
{
    svo_rrt_ops_t ops;
    svo_rrt_params_t params;

    svoRRTOpsInit(&ops);
    ops.terminate  = rrtTerminate;
    ops.expand_all = rrtBlossomExpandAll;
    ops.random     = rrtConf;
    ops.filter_blossom = NULL;
    ops.callback   = rrtCallback;
    ops.callback_period = callback_period;

    svoRRTParamsInit(&params);
    params.dim = borCfgMapConfDim();
    setUpNN(&params.nn);

    rrt = svoRRTNew(&ops, &params);

    h2 = h * h;
}

static int rrtBlossomRun(bor_list_t *path)
{
    int ret;
    const svo_rrt_node_t *init_node;

    svoRRTRunBlossom(rrt, init);
    init_node = svoRRTNodeInitial(rrt);
    borListInit(path);
    ret = svoRRTFindPath(rrt, init_node, rrt_last, path);
    return ret;
}

/*** RRT BLOSSOM END ***/

/*** RRT BLOSSOM WITH FILTER ***/
static int rrtBlossomFilter(const svo_rrt_t *rrt,
                            const bor_vec_t *c,
                            const svo_rrt_node_t *src,
                            const svo_rrt_node_t *near,
                            void *data)
{
    const bor_vec_t *s, *n;

    s = svoRRTNodeConf(src);
    n = svoRRTNodeConf(near);

    return s == n || borVecDist(rrt->params.dim, c, n) > borVecDist(rrt->params.dim, c, s);
}

static void rrtBlossomFilterInit(void)
{
    svo_rrt_ops_t ops;
    svo_rrt_params_t params;

    svoRRTOpsInit(&ops);
    ops.terminate  = rrtTerminate;
    ops.expand_all = rrtBlossomExpandAll;
    ops.random     = rrtConf;
    ops.filter_blossom = rrtBlossomFilter;
    ops.callback   = rrtCallback;
    ops.callback_period = callback_period;

    svoRRTParamsInit(&params);
    params.dim = borCfgMapConfDim();
    setUpNN(&params.nn);

    rrt = svoRRTNew(&ops, &params);

    h2 = h * h;
}
/*** RRT BLOSSOM WITH FILTER END ***/
