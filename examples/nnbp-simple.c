#include <string.h>
#include <svoboda/nnbp.h>

void xor(void)
{
    svo_nnbp_params_t params;
    svo_nnbp_t *nn;
    size_t layer_size[] = { 3, 4, 1 };
    bor_real_t err;
    const bor_vec_t *out;
    bor_vec_t *in;
    int i, j, max_iter;
    BOR_VEC_ARR(data, 4, 8) = {
        {BOR_REAL(0.), BOR_REAL(0.), BOR_REAL(0.), BOR_REAL(0.)},
        {BOR_REAL(0.), BOR_REAL(0.), BOR_REAL(1.), BOR_REAL(1.)},
        {BOR_REAL(0.), BOR_REAL(1.), BOR_REAL(0.), BOR_REAL(1.)},
        {BOR_REAL(0.), BOR_REAL(1.), BOR_REAL(1.), BOR_REAL(0.)},
        {BOR_REAL(1.), BOR_REAL(0.), BOR_REAL(0.), BOR_REAL(1.)},
        {BOR_REAL(1.), BOR_REAL(0.), BOR_REAL(1.), BOR_REAL(0.)},
        {BOR_REAL(1.), BOR_REAL(1.), BOR_REAL(0.), BOR_REAL(0.)},
        {BOR_REAL(1.), BOR_REAL(1.), BOR_REAL(1.), BOR_REAL(1.)}
    };
    BOR_VEC_ARR(testdata, 3, 8) = {
        {0, 0, 0},
        {0, 0, 1},
        {0, 1, 0},
        {0, 1, 1},
        {1, 0, 0},
        {1, 0, 1},
        {1, 1, 0},
        {1, 1, 1}
    };

    svoNNBPParamsInit(&params);
    params.layers_num = sizeof(layer_size) / sizeof(size_t);
    params.layer_size = layer_size;

    params.eta    = 0.2;
    params.alpha  = 0.1;
    params.lambda = 1.;
    max_iter = 500000;

    nn = svoNNBPNew(&params);

    for (i = 0; i < max_iter; i++){
        for (j = 0; j < 5; j++){
            in = data[i % 8];
            svoNNBPLearn(nn, in, borVecOff(in, 3));
        }

        err = svoNNBPErr(nn, borVecOff(in, 3));
        printf("[%08d]: %f\n", (int)i, (float)err);
        if (err < 0.00001)
            break;
    }

    printf("train data:\n");
    for (i = 0; i < 8; i++){
        in = data[i];
        out = svoNNBPFeed(nn, in);
        err = svoNNBPErr(nn, borVecOff(in, 3));
        printf("[%d] train: %f, out: %f, err: %f\n", i, borVecGet(in, 3), borVecGet(out, 0), err);
    }

    printf("test data:\n");
    for (i = 0; i < 8; i++){
        in = testdata[i];
        out = svoNNBPFeed(nn, in);
        err = svoNNBPErr(nn, borVecOff(in, 3));
        printf("[%d] %f %f %f: %f\n", i,
               borVecGet(in, 0), borVecGet(in, 1), borVecGet(in, 2),
               borVecGet(out, 0));
    }

    svoNNBPDel(nn);
}

int usage(int argc, char *argv[])
{
    fprintf(stderr, "Usage: %s xor\n", argv[0]);
    return -1;
}

int main(int argc, char *argv[])
{
    if (argc != 2){
        return usage(argc, argv);
    }

    if (strcmp(argv[1], "xor") == 0){
        xor();
    }else{
        return usage(argc, argv);
    }

    return 0;
}
