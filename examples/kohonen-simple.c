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

#include <svoboda/kohonen.h>
#include <boruvka/alloc.h>
#include <boruvka/rand-mt.h>

bor_rand_mt_t *rnd;
bor_real_t aabb[6] = { -BOR_REAL(5.), BOR_REAL(5.),
                       -BOR_REAL(5.), BOR_REAL(5.),
                       -BOR_REAL(5.), BOR_REAL(5.) };

BOR_VEC(is, 2);

static const bor_vec_t *inputSignal(svo_kohonen_t *k, void *_)
{
    borVecSet(is, 0, borRandMT(rnd, -5, 5));
    borVecSet(is, 1, borRandMT(rnd, -5, 5));
    return is;
}

static int neighborhood(svo_kohonen_t *k,
                        const svo_kohonen_node_t *center,
                        const svo_kohonen_node_t *cur,
                        int depth,
                        bor_real_t *val, void *_)
{
    if (depth > 1)
        return 0;

    *val = BOR_EXP(-BOR_SQ(depth) / (2. * BOR_SQ(0.1)));
    return 1;
}

static int terminate(svo_kohonen_t *k, void *_)
{
    static int c = 0;

    if (c++ == 1000000)
        return 1;
    return 0;
}

static void callback(svo_kohonen_t *k, void *_)
{
    svoKohonenDumpSVT(k, stdout, NULL);
}

static void createGrid(svo_kohonen_t *k, int x, int y)
{
    svo_kohonen_node_t **n;
    int i, j;

    n = BOR_ALLOC_ARR(svo_kohonen_node_t *, x * y);

    for (i = 0; i < x * y; i++){
        n[i] = svoKohonenNodeNew(k, inputSignal(k, NULL));
    }

    for (i = 0; i < y; i++){
        for (j = 0; j < x; j++){
            if (j < x - 1)
                svoKohonenNodeConnect(k, n[i * x + j], n[i * x + j + 1]);
            if (i < y - 1)
                svoKohonenNodeConnect(k, n[i * x + j], n[(i + 1) * x + j]);
        }
    }

    BOR_FREE(n);
}

int main(int argc, char *argv[])
{
    svo_kohonen_ops_t ops;
    svo_kohonen_params_t params;
    svo_kohonen_t *k;

    //rnd = borRandMTNewAuto();
    rnd = borRandMTNew(1111);

    svoKohonenOpsInit(&ops);
    ops.input_signal = inputSignal;
    ops.neighborhood = neighborhood;
    ops.terminate    = terminate;
    ops.callback     = callback;
    ops.callback_period = 100;

    svoKohonenParamsInit(&params);
    params.dim = 2;
    params.learn_rate = 0.01;
    params.nn.type     = BOR_NN_GUG;
    params.nn.gug.aabb = aabb;

    k = svoKohonenNew(&ops, &params);
    createGrid(k, 5, 5);
    svoKohonenRun(k);
    svoKohonenDel(k);

    borRandMTDel(rnd);

    return 0;
}
