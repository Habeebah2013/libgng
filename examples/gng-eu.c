#include <stdio.h>
#include <boruvka/dbg.h>
#include <boruvka/timer.h>
#include <svoboda/gng-eu.h>

size_t max_nodes;
svo_gng_eu_t *gng;
bor_timer_t timer;
bor_pc_t *pc;
bor_pc_it_t pcit;

static int terminate(void *data);
static void callback(void *data);
static const bor_vec_t *input_signal(void *data);

int main(int argc, char *argv[])
{
    svo_gng_eu_params_t params;
    svo_gng_eu_ops_t ops;
    size_t size;
    bor_real_t aabb[30];

    if (argc < 4){
        fprintf(stderr, "Usage: %s dim file.pts max_nodes\n", argv[0]);
        return -1;
    }

    max_nodes = atoi(argv[3]);


    svoGNGEuParamsInit(&params);
    params.dim = atoi(argv[1]);
    params.nn.type = BOR_NN_GUG;
    params.nn.type = BOR_NN_VPTREE;
    params.nn.gug.num_cells = 0;
    params.nn.gug.max_dens = 0.1;
    params.nn.gug.expand_rate = 1.5;

    svoGNGEuOpsInit(&ops);
    ops.terminate = terminate;
    ops.input_signal = input_signal;
    ops.callback  = callback;
    ops.callback_period = 300;
    ops.data = NULL;

    pc = borPCNew(params.dim);
    size = borPCAddFromFile(pc, argv[2]);
    borPCAABB(pc, aabb);
    params.nn.gug.aabb = aabb;
    fprintf(stderr, "Added %d points from %s\n", (int)size, argv[2]);

    borPCPermutate(pc);
    borPCItInit(&pcit, pc);

    gng = svoGNGEuNew(&ops, &params);

    borTimerStart(&timer);
    svoGNGEuRun(gng);
    callback(NULL);
    fprintf(stderr, "\n");

    svoGNGEuDumpSVT(gng, stdout, NULL);

    svoGNGEuDel(gng);

    borPCDel(pc);

    return 0;
}


static int terminate(void *data)
{
    return svoGNGEuNodesLen(gng) >= max_nodes;
}

static void callback(void *data)
{
    size_t nodes_len;

    nodes_len = svoGNGEuNodesLen(gng);

    borTimerStopAndPrintElapsed(&timer, stderr, " n: %d / %d\r", nodes_len, max_nodes);
}

static const bor_vec_t *input_signal(void *data)
{
    const bor_vec_t *v;

    if (borPCItEnd(&pcit)){
        borPCPermutate(pc);
        borPCItInit(&pcit, pc);
    }
    v = borPCItGet(&pcit);
    borPCItNext(&pcit);
    return v;
}
