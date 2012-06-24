#include <stdio.h>
#include <svoboda/gpc.h>
#include <boruvka/dbg.h>
#include <boruvka/alloc.h>
#include <boruvka/cfg.h>
#include <boruvka/opts.h>

bor_cfg_t *cfg = NULL;

int classes, cols;

int train_rows;
const bor_real_t *train_x;
const int *train_y;
size_t train_len;

int test_rows;
const bor_real_t *test_x;
const int *test_y;
size_t test_len;

svo_gpc_ops_t ops;
svo_gpc_params_t params;
int help = 0;
int verbose = 0;
int output_results = 1;
const char *output_prefix = "result";
const char *log_fn = NULL;
FILE *log_fout = NULL;


static int readCfg(const char *fn);

static void *dataRow(svo_gpc_t *gpc, int i, void *_);
static bor_real_t fitness(svo_gpc_t *gpc, int *class, void *_);
static void callback(svo_gpc_t *gpc, void *_);

struct cmp_t {
    int idx;
    bor_real_t val;
    int op; // 0 - '<', 1 - '>'
};
static void cmpInit(svo_gpc_t *gpc, void *mem, void *ud);
static int cmpPred(svo_gpc_t *gpc, void *mem, void *data, void *ud);
static void cmpFormat(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max);

struct cmp2_t {
    int idx1, idx2;
    int op; // 0 - '<', 1 - '>'
};
static void cmp2Init(svo_gpc_t *gpc, void *mem, void *ud);
static int cmp2Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud);
static void cmp2Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max);

struct cmp3_t {
    int idx1, idx2;
    bor_real_t val;
    int op; // X0 - '<', X1 - '>' | 00X - '+', 01X - '-', 10X - '*', 11X - '/'
};
static void cmp3Init(svo_gpc_t *gpc, void *mem, void *ud);
static int cmp3Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud);
static void cmp3Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max);

struct cmp4_t {
    int idx1, idx2, idx3;
    int op; // X0 - '<', X1 - '>' | 00X - '+', 01X - '-', 10X - '*', 11X - '/'
};
static void cmp4Init(svo_gpc_t *gpc, void *mem, void *ud);
static int cmp4Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud);
static void cmp4Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max);

static void outputResults(svo_gpc_t *gpc);


static int opts(int *argc, char *argv[])
{
    int ok = 1;

    svoGPCOpsInit(&ops);
    ops.fitness  = fitness;
    ops.data_row = dataRow;
    ops.callback = callback;
    ops.callback_period = 1;

    svoGPCParamsInit(&params);
    params.pop_size    = 200;
    params.max_depth   = 3;
    params.keep_best   = 1;//params.pop_size * 0.02;
    params.throw_worst = 1;//params.pop_size * 0.005;
    params.max_steps   = 2000;
    params.tournament_size = 3;

    params.pr = 10;
    params.pc = 30;
    params.pm = 10;

    params.simplify      = 100UL;
    params.prune_deep    = 100UL;
    params.rm_duplicates = 100UL;
    params.inc_max_depth = 0UL;
    params.inc_max_depth_step = 1;

    params.parallel = 0;


    borOptsAddDesc("help", 0x0, BOR_OPTS_NONE, (void *)&help, NULL,
                   "Print this help");
    borOptsAddDesc("verbose", 'v', BOR_OPTS_NONE, &verbose, NULL,
                   "Turn on verbose mode.");
    borOptsAddDesc("progress-period", 0x0, BOR_OPTS_LONG, (long *)&ops.callback_period, NULL,
                   "Set up period of progress refreshing.");

    borOptsAddDesc("output-results", 'o', BOR_OPTS_INT, &output_results, NULL,
                   "Print specified number of best individuals. Default: 1");
    borOptsAddDesc("output-prefix", 'p', BOR_OPTS_STR, &output_prefix, NULL,
                   "Set up prefix of output file. Default: `result'");
    borOptsAddDesc("log", 'l', BOR_OPTS_STR, &log_fn, NULL,
                   "Set up name of the log file. Default: none");

    borOptsAddDesc("pop-size", 0x0, BOR_OPTS_INT, &params.pop_size, NULL,
                   "Population size. Default: 200");
    borOptsAddDesc("max-depth", 0x0, BOR_OPTS_INT, &params.max_depth, NULL,
                   "Maximal depth of a tree individual. Default: 3");
    borOptsAddDesc("keep-best", 0x0, BOR_OPTS_INT, &params.keep_best, NULL,
                   "Keep specified number of the best individuals. Default: 1");
    borOptsAddDesc("throw-worst", 0x0, BOR_OPTS_INT, &params.throw_worst, NULL,
                   "Discard specified number of the worst individuals.  Default: 1");
    borOptsAddDesc("max-steps", 0x0, BOR_OPTS_LONG, (long *)&params.max_steps, NULL,
                   "Maximal number of steps. Default: 2000");
    borOptsAddDesc("tour-size", 0x0, BOR_OPTS_INT, &params.tournament_size, NULL,
                   "Size of tournament selection. Default: 3");

    borOptsAddDesc("pr", 0x0, BOR_OPTS_REAL, &params.pr, NULL,
                   "Probability of reproduction. The number is considered "
                   "in comparison with --pc and --pm. Default: 10");
    borOptsAddDesc("pc", 0x0, BOR_OPTS_REAL, &params.pc, NULL,
                   "Probability of crossover. Default: 30");
    borOptsAddDesc("pm", 0x0, BOR_OPTS_REAL, &params.pm, NULL,
                   "Probability of mutation. Default: 10");

    borOptsAddDesc("simplify", 0x0, BOR_OPTS_LONG, (long *)&params.simplify, NULL,
                   "All individuals are simplified every specified step.  Default: 100");
    borOptsAddDesc("prune-deep", 0x0, BOR_OPTS_LONG, (long *)&params.prune_deep, NULL,
                   "Prune all deep trees every specified step. Default: 100");
    borOptsAddDesc("rm-dupl", 0x0, BOR_OPTS_LONG, (long *)&params.rm_duplicates, NULL,
                   "Remove duplicates every specified step. Default: 100");
    borOptsAddDesc("inc-max-depth", 0x0, BOR_OPTS_LONG, (long *)&params.inc_max_depth, NULL,
                   "Increase a max. depth by --inc-max-depth-step value"
                   " every specified step. Default: 0");
    borOptsAddDesc("inc-max-depth-step", 0x0, BOR_OPTS_INT, (int *)&params.inc_max_depth_step, NULL,
                   "See --inc-max-depth. Default: 1");

    borOptsAddDesc("parallel", 0x0, BOR_OPTS_INT, &params.parallel, NULL,
                   "Set up number of parallel threads. Default: 0");


    if (borOpts(argc, argv) != 0)
        ok = 0;

    if (*argc != 2)
        ok = 0;



    if (!help && readCfg(argv[1]) != 0){
        fprintf(stderr, "Invalid input file `%s'\n", argv[1]);
        return -1;
    }
    params.data_rows   = train_rows;


    if (help || !ok){
        fprintf(stderr, "Usage: %s [ OPTIONS ] file.data\n", argv[0]);
        fprintf(stderr, "  OPTIONS:\n");
        borOptsPrint(stderr, "    ");
        return -1;
    }

    if (log_fn != NULL){
        log_fout = fopen(log_fn, "w");
        if (log_fout){
            fprintf(log_fout, "# avg_fitness med_fitness max_fitness avg_depth max_depth avg_node\n");
        }else{
            fprintf(stderr, "Can't open a file `%s' for logging.\n", log_fn);
        }
    }


    if (verbose){
        fprintf(stderr, "Parameters:\n");
        fprintf(stderr, "    Progress period: %ld\n", ops.callback_period);
        fprintf(stderr, "\n");
        fprintf(stderr, "    Population size: %d\n", params.pop_size);
        fprintf(stderr, "    Max. depth:      %d\n", params.max_depth);
        fprintf(stderr, "    Keep no. best:   %d\n", params.keep_best);
        fprintf(stderr, "    Throw no. worst: %d\n", params.throw_worst);
        fprintf(stderr, "    Max. no. steps:  %ld\n", params.max_steps); 
        fprintf(stderr, "    Tournament size: %d\n", params.tournament_size);
        fprintf(stderr, "\n");
        fprintf(stderr, "    Reproduction: %f\n", params.pr);
        fprintf(stderr, "    Crossover:    %f\n", params.pc);
        fprintf(stderr, "    Mutation:     %f\n", params.pm);
        fprintf(stderr, "\n");
        fprintf(stderr, "    Simplify every:    %ld\n", params.simplify);
        fprintf(stderr, "    Prune every:       %ld\n", params.prune_deep);
        fprintf(stderr, "    Remove duplicates: %ld\n", params.rm_duplicates);
        fprintf(stderr, "    Inc. max. depth:   %ld by %d\n",
                params.inc_max_depth, params.inc_max_depth_step);
        fprintf(stderr, "\n");
        fprintf(stderr, "    Classes:    %d\n", classes);
        fprintf(stderr, "    Predictors: %d\n", cols);
        fprintf(stderr, "    Train rows: %d\n", train_rows);
        fprintf(stderr, "    Test rows:  %d\n", test_rows);
        fprintf(stderr, "\n");
        fprintf(stderr, "    Parallel: %d\n", params.parallel);
        fprintf(stderr, "\n");
    }

    return 0;
}


int main(int argc, char *argv[])
{
    int res, i;
    svo_gpc_t *gpc;


    if (opts(&argc, argv) != 0){
        return -1;
    }

    gpc = svoGPCNew(&ops, &params);

    for (i = 0; i < classes; i++){
        svoGPCAddClass(gpc, i);
    }

    svoGPCAddPred(gpc, cmpPred, cmpInit, cmpFormat, 2, sizeof(struct cmp_t), NULL);
    svoGPCAddPred(gpc, cmp2Pred, cmp2Init, cmp2Format, 2, sizeof(struct cmp2_t), NULL);
    svoGPCAddPred(gpc, cmp3Pred, cmp3Init, cmp3Format, 2, sizeof(struct cmp3_t), NULL);
    svoGPCAddPred(gpc, cmp4Pred, cmp4Init, cmp4Format, 2, sizeof(struct cmp4_t), NULL);

    res = svoGPCRun(gpc);
    callback(gpc, NULL);
    fprintf(stderr, "\n");
    fprintf(stdout, "Best fitness: %f\n", svoGPCBestFitness(gpc));

    outputResults(gpc);

    svoGPCDel(gpc);

    if (cfg)
        borCfgDel(cfg);

    borOptsClear();

    if (log_fout)
        fclose(log_fout);

    return res;
}


static int readCfg(const char *fn)
{
    size_t len;

    if ((cfg = borCfgRead(fn)) == NULL)
        return -1;

    if (!borCfgParamIsInt(cfg, "classes")
            || !borCfgParamIsInt(cfg, "cols")
            || !borCfgParamIsInt(cfg, "train_rows")
            || !borCfgParamIsInt(cfg, "test_rows")
            || !borCfgParamIsFlt(cfg, "train_x")
            || !borCfgParamIsArr(cfg, "train_x")
            || !borCfgParamIsInt(cfg, "train_y")
            || !borCfgParamIsArr(cfg, "train_y")
            || !borCfgParamIsFlt(cfg, "test_x")
            || !borCfgParamIsArr(cfg, "test_x")
            || !borCfgParamIsInt(cfg, "test_y")
            || !borCfgParamIsArr(cfg, "test_y")
       ){
        borCfgDel(cfg);
        return -1;
    }

    borCfgParamInt(cfg, "classes", &classes);
    borCfgParamInt(cfg, "cols", &cols);

    borCfgParamInt(cfg, "train_rows", &train_rows);
    borCfgParamFltArr(cfg, "train_x", &train_x, &len);
    borCfgParamIntArr(cfg, "train_y", &train_y, &train_len);
    if (len / cols != train_len){
        fprintf(stderr, "Error: len(train_x) != len(train_y) [%d != %d]\n",
                (int)train_len, (int)(len / cols));
        borCfgDel(cfg);
        return -1;
    }

    borCfgParamInt(cfg, "test_rows", &test_rows);
    borCfgParamFltArr(cfg, "test_x", &test_x, &len);
    borCfgParamIntArr(cfg, "test_y", &test_y, &test_len);
    if (len / cols != test_len){
        fprintf(stderr, "Error: len(test_x) != len(test_y) [%d != %d]\n",
                (int)test_len, (int)(len / cols));
        borCfgDel(cfg);
        return -1;
    }

    return 0;
}

static void *dataRow(svo_gpc_t *gpc, int i, void *_)
{
    return (void *)(train_x + (i * cols));
}

static bor_real_t fitness(svo_gpc_t *gpc, int *class, void *_)
{
    int i, ft;

    ft = 0;
    for (i = 0; i < train_rows; i++){
        ft += (class[i] == train_y[i]);
    }

    return ((bor_real_t)ft) / train_rows;
}

static void callback(svo_gpc_t *gpc, void *_)
{
    svo_gpc_stats_t stats;

    svoGPCStats(gpc, &stats);
    fprintf(stderr, "[%06ld] min: %f, max: %f, avg: %f, med: %f "
                    "| depth: %.2f (%3d) "
                    "| nodes: %.2f (%4d) "
                    "| max. depth: %d "
                    "          \r",
            stats.elapsed, stats.min_fitness, stats.max_fitness,
            stats.avg_fitness, stats.med_fitness,
            stats.avg_depth, stats.max_depth,
            stats.avg_nodes, stats.max_nodes,
            svoGPCMaxDepth(gpc));
    fflush(stderr);

    if (log_fout){
        fprintf(log_fout, "%f %f %f %f %f %f\n",
                stats.avg_fitness, stats.med_fitness,
                stats.max_fitness,
                stats.avg_depth, (float)svoGPCMaxDepth(gpc),
                stats.avg_nodes);
        fflush(log_fout);
    }
}


static void cmpInit(svo_gpc_t *gpc, void *mem, void *ud)
{
    struct cmp_t *m = (struct cmp_t *)mem;
    m->idx = svoGPCRandInt(gpc, 0, cols);
    m->val = svoGPCRand(gpc, -1., 1.);
    m->op  = svoGPCRandInt(gpc, 0, 2);
}

static int cmpPred(svo_gpc_t *gpc, void *mem, void *data, void *ud)
{
    struct cmp_t *m = (struct cmp_t *)mem;
    bor_real_t *d = (bor_real_t *)data;
    if (m->op == 0){
        return (d[m->idx] < m->val ? 0 : 1);
    }else{
        return (d[m->idx] > m->val ? 0 : 1);
    }
}

static void cmpFormat(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max)
{
    struct cmp_t *m = (struct cmp_t *)mem;
    if (m->op == 0){
        snprintf(str, max, "data[%d] < %f", (int)m->idx, m->val);
    }else{
        snprintf(str, max, "data[%d] > %f", (int)m->idx, m->val);
    }
}

static void cmp2Init(svo_gpc_t *gpc, void *mem, void *ud)
{
    struct cmp2_t *m = (struct cmp2_t *)mem;
    m->idx1 = svoGPCRandInt(gpc, 0, cols);
    do {
        m->idx2 = svoGPCRandInt(gpc, 0, cols);
    } while (m->idx2 == m->idx1);
    m->op  = svoGPCRandInt(gpc, 0, 2);
}

static int cmp2Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud)
{
    struct cmp2_t *m = (struct cmp2_t *)mem;
    bor_real_t *d = (bor_real_t *)data;
    if (m->op == 0){
        return (d[m->idx1] < d[m->idx2] ? 0 : 1);
    }else{
        return (d[m->idx1] > d[m->idx2] ? 0 : 1);
    }
}

static void cmp2Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max)
{
    struct cmp2_t *m = (struct cmp2_t *)mem;
    if (m->op == 0){
        snprintf(str, max, "data[%d] < data[%d]", m->idx1, m->idx2);
    }else{
        snprintf(str, max, "data[%d] > data[%d]", m->idx1, m->idx2);
    }
}


static void cmp3Init(svo_gpc_t *gpc, void *mem, void *ud)
{
    struct cmp3_t *m = (struct cmp3_t *)mem;
    int op2;
    m->idx1 = svoGPCRandInt(gpc, 0, cols);
    do {
        m->idx2 = svoGPCRandInt(gpc, 0, cols);
    } while (m->idx2 == m->idx1);
    m->val = svoGPCRand(gpc, -1., 1.);

    m->op  = svoGPCRandInt(gpc, 0, 2);
    op2    = svoGPCRandInt(gpc, 0, 4);
    m->op |= (op2 << 1);
}

static int cmp3Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud)
{
    struct cmp3_t *m = (struct cmp3_t *)mem;
    bor_real_t *d = (bor_real_t *)data;
    bor_real_t v1;
    int op2;

    op2 = (m->op >> 1);
    if (op2 == 0){
        v1 = d[m->idx1] + d[m->idx2];
    }else if (op2 == 1){
        v1 = d[m->idx1] - d[m->idx2];
    }else if (op2 == 2){
        v1 = d[m->idx1] * d[m->idx2];
    }else{
        v1 = d[m->idx1] / d[m->idx2];
    }

    if ((m->op & 0x1) == 0){
        return (v1 < m->val ? 0 : 1);
    }else{
        return (v1 > m->val ? 0 : 1);
    }
}
static void cmp3Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max)
{
    struct cmp3_t *m = (struct cmp3_t *)mem;
    char cop, cop2;
    int op2;

    if ((m->op & 0x1) == 0){
        cop = '<';
    }else{
        cop = '>';
    }

    op2 = (m->op >> 1);
    if (op2 == 0){
        cop2 = '+';
    }else if (op2 == 1){
        cop2 = '-';
    }else if (op2 == 2){
        cop2 = '*';
    }else{
        cop2 = '/';
    }

    snprintf(str, max, "data[%d] %c data[%d] %c %f", m->idx1, cop2, m->idx2, cop, m->val);
}

static void cmp4Init(svo_gpc_t *gpc, void *mem, void *ud)
{
    struct cmp4_t *m = (struct cmp4_t *)mem;
    int op2;
    m->idx1 = svoGPCRandInt(gpc, 0, cols);
    do {
        m->idx2 = svoGPCRandInt(gpc, 0, cols);
    } while (m->idx2 == m->idx1);
    do {
        m->idx3 = svoGPCRandInt(gpc, 0, cols);
    } while (m->idx3 == m->idx1 || m->idx3 == m->idx2);

    m->op  = svoGPCRandInt(gpc, 0, 2);
    op2    = svoGPCRandInt(gpc, 0, 4);
    m->op |= (op2 << 1);
}

static int cmp4Pred(svo_gpc_t *gpc, void *mem, void *data, void *ud)
{
    struct cmp4_t *m = (struct cmp4_t *)mem;
    bor_real_t *d = (bor_real_t *)data;
    bor_real_t v1;
    int op2;

    op2 = (m->op >> 1);
    if (op2 == 0){
        v1 = d[m->idx1] + d[m->idx2];
    }else if (op2 == 1){
        v1 = d[m->idx1] - d[m->idx2];
    }else if (op2 == 2){
        v1 = d[m->idx1] * d[m->idx2];
    }else{
        v1 = d[m->idx1] / d[m->idx2];
    }

    if ((m->op & 0x1) == 0){
        return (v1 < d[m->idx3] ? 0 : 1);
    }else{
        return (v1 > d[m->idx3] ? 0 : 1);
    }
}
static void cmp4Format(svo_gpc_t *gpc, void *mem, void *ud, char *str, size_t max)
{
    struct cmp4_t *m = (struct cmp4_t *)mem;
    char cop, cop2;
    int op2;

    if ((m->op & 0x1) == 0){
        cop = '<';
    }else{
        cop = '>';
    }

    op2 = (m->op >> 1);
    if (op2 == 0){
        cop2 = '+';
    }else if (op2 == 1){
        cop2 = '-';
    }else if (op2 == 2){
        cop2 = '*';
    }else{
        cop2 = '/';
    }

    snprintf(str, max, "data[%d] %c data[%d] %c data[%d]", m->idx1, cop2, m->idx2, cop, m->idx3);
}


static float correct(svo_gpc_t *gpc, int id,
                     const bor_real_t *x, const int *y, int len)
{
    int i, class;
    void *tree, *data;
    int correct;

    tree = svoGPCTree(gpc, id);

    correct = 0;
    for (i = 0; i < len; i++){
        data = (void *)(x + (i * cols));
        class = svoGPCTreeEval(gpc, tree, data);
        if (class == y[i])
            correct++;
    }

    return (float)correct / (float)len;
}

static void outputResults(svo_gpc_t *gpc)
{
    int i;
    void *tree;
    char fn[1024];
    FILE *fout;

    for (i = 0; i < output_results; i++){
        tree = svoGPCTree(gpc, i);
        if (!tree)
            break;

        snprintf(fn, 1024, "%s%04d", output_prefix, i);
        fout = fopen(fn, "w");
        if (!fout){
            fprintf(stderr, "Error: Can't open the file `%s'\n", fn);
            continue;
        }

        fprintf(fout, "// Train accuracy: %f\n", correct(gpc, i, train_x, train_y, train_rows));
        fprintf(fout, "// Test accuracy: %f\n", correct(gpc, i, test_x, test_y, test_rows));
        fprintf(fout, "// Depth: %d\n", svoGPCTreeDepth(gpc, tree));
        svoGPCTreePrintC(gpc, tree, "predict", fout);
        fclose(fout);
    }
    //evalTestData(gpc);
}


