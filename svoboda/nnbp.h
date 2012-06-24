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

#ifndef __SVO_NNBP_H__
#define __SVO_NNBP_H__

#include <boruvka/core.h>
#include <boruvka/vec.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Backpropagation Neural Networks
 * =================================
 */


struct _svo_nnbp_layer_t {
    size_t size;   /*!< Size of layer */
    bor_vec_t *x;  /*!< Layer output.
                        x[i] = output of (i - 1)'th neuron
                        x[0] = -1 */
    bor_vec_t **w; /*!< Layer weights.
                        w[i] = weight of i'th neuron (~x[i + 1]) */
    bor_vec_t **prevw;
};
typedef struct _svo_nnbp_layer_t svo_nnbp_layer_t;

/**
 * Parameters
 * -----------
 */
struct _svo_nnbp_params_t {
    size_t layers_num;  /*!< Number of layers */
    size_t *layer_size; /*!< Number of neurons in layer (from input to
                             output). Size of array must be {.layers} */
    bor_real_t alpha;   /*!< */
    bor_real_t eta;     /*!< */

    bor_real_t lambda;  /*!< Sigmoid slope */
    int func;           /*!< Transition function of single neuron.
                             Default: SVO_NNBP_SIGMOID */
};
typedef struct _svo_nnbp_params_t svo_nnbp_params_t;

/**
 * Initialize parameters
 */
void svoNNBPParamsInit(svo_nnbp_params_t *params);

/** Transition Functions */
#define SVO_NNBP_SIGMOID   0  /*!< sigmoid <-1, 1> */
#define SVO_NNBP_SIGMOID01 1  /*!< sigmoid <0, 1> */

/**
 * Algorithm
 * ----------
 */
struct _svo_nnbp_t {
    size_t layers_num;  /*!< Number of layers */
    bor_real_t alpha, eta, lambda;

    svo_nnbp_layer_t *layers; /*!< Array of layers */
    bor_vec_t *delta[2];
    bor_vec_t *tmp;

    int func;
};
typedef struct _svo_nnbp_t svo_nnbp_t;

/**
 * Create new network
 */
svo_nnbp_t *svoNNBPNew(const svo_nnbp_params_t *params);

/**
 * Delete network
 */
void svoNNBPDel(svo_nnbp_t *nn);

/**
 * Feeds network with an input.
 * An output of the network is returned.
 */
const bor_vec_t *svoNNBPFeed(svo_nnbp_t *nn, const bor_vec_t *in);

/**
 * Returns error of feeded network (sum of sqares)
 */
bor_real_t svoNNBPErr(const svo_nnbp_t *nn, const bor_vec_t *out);

/**
 * Show one pattern for learning.
 * Returns an error.
 * A length of {in} must equal to {params.layer_size[0]} and
 * length of {out} must equal to number of neurons in output layer.
 */
void svoNNBPLearn(svo_nnbp_t *nn, const bor_vec_t *in, const bor_vec_t *out);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SVO_NNBP_H__ */

