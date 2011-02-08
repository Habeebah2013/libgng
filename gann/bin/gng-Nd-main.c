#include <stdio.h>
#include <fermat/dbg.h>
#include <fermat/timer.h>
#include <gann/gng`N`.h>

struct _params_t {
    size_t max_nodes;
    gann_gng`N`_t *gng;
    fer_timer_t timer;
};
typedef struct _params_t params_t;


static int terminate(void *data);
static void callback(void *data);

int main(int argc, char *argv[])
{
    gann_gng`N`_params_t params;
    gann_gng`N`_ops_t ops;
    gann_gng`N`_t *gng;
    params_t p;
    size_t size;

    if (argc < 2){
        fprintf(stderr, "Usage: %s file.pts [ max_nodes ]\n", argv[0]);
        return -1;
    }

    gannGNG`N`ParamsInit(&params);

    gannGNG`N`OpsInit(&ops);
    ops.terminate = terminate;
    ops.callback  = callback;
    ops.callback_period = 100;
    ops.data = &p;

    gng = gannGNG`N`New(&ops, &params);
    size = gannGNG`N`AddInputSignalsFromFile(gng, argv[1]);
    fprintf(stderr, "Added %d points from %s\n", size, argv[1]);

    p.max_nodes = 1000;
    p.gng = gng;

    if (argc >= 3){
        p.max_nodes = atoi(argv[2]);
    }

    ferTimerStart(&p.timer);
    gannGNG`N`Run(gng);
    callback(&p);
    fprintf(stderr, "\n");

    gannGNG`N`DumpSVT(gng, stdout, NULL);

    gannGNG`N`Del(gng);

    return 0;
}


static int terminate(void *data)
{
    params_t *p = (params_t *)data;
    return gannGNG`N`NodesLen(p->gng) >= p->max_nodes;
}

static void callback(void *data)
{
    params_t *p = data;
    size_t nodes_len;

    nodes_len = gannGNG`N`NodesLen(p->gng);

    ferTimerStopAndPrintElapsed(&p->timer, stderr, " n: %d / %d\r", nodes_len, p->max_nodes);
}