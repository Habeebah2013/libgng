/***
 * fermat
 * -------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of fermat.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

struct _fer_cd_sap_1_radix_sort_t {
    fer_cd_sap_minmax_t *minmax;
    size_t minmax_len, minmax_alloc;
    uint32_t *counter;  /*!< Array for count-sort */
    uint32_t negative;  /*!< Number of negative values */
};
typedef struct _fer_cd_sap_1_radix_sort_t fer_cd_sap_1_radix_sort_t;

struct _fer_cd_sap_1_t {
    fer_cd_sap_t sap;
    fer_cd_sap_1_radix_sort_t rs; /*!< Cached radix sort struct */
    fer_hmap_t *pairs_reg; /*!< Register (hash map) of collide pairs */
} fer_packed fer_aligned(16);
typedef struct _fer_cd_sap_1_t fer_cd_sap_1_t;

static void sap1RadixSort(fer_cd_sap_t *_sap, int axis);
static void sap1FindPairs(fer_cd_sap_t *_sap);

static fer_cd_sap_t *ferCDSAP1New(fer_cd_t *cd, uint64_t flags)
{
    fer_cd_sap_1_t *sap;
    size_t regsize;

    sap = FER_ALLOC(fer_cd_sap_1_t);

    ferCDSAPInit(cd, &sap->sap, 1);
    sap->sap.type = FER_CD_SAP_TYPE_1;
    sap->sap.radix_sort = sap1RadixSort;
    sap->sap.find_pairs = sap1FindPairs;

    regsize = __FER_CD_SAP_HASH_TABLE_SIZE(flags);
    sap->pairs_reg = ferHMapNew(regsize, pairHash, pairEq, (void *)sap);

    sap->rs.minmax = NULL;
    sap->rs.minmax_len = 0;
    sap->rs.minmax_alloc = 0;
    sap->rs.counter = FER_ALLOC_ARR(uint32_t, 256);

    return (fer_cd_sap_t *)sap;
}

static void ferCDSAP1Del(fer_cd_sap_t *_sap)
{
    fer_cd_sap_1_t *sap = (fer_cd_sap_1_t *)_sap;

    ferHMapDel(sap->pairs_reg);

    if (sap->rs.minmax)
        free(sap->rs.minmax);
    free(sap->rs.counter);

    ferCDSAPDestroy(&sap->sap);

    free(sap);
}

_fer_inline uint32_t radixSortVal(const fer_cd_sap_minmax_t *m,
                                  uint32_t shift);
_fer_inline void radixSortZeroizeCounter(uint32_t *counter);
static void radixSortCount(const fer_cd_sap_minmax_t *minmax,
                           uint32_t len,
                           uint32_t *counter, uint32_t shift);
static void radixSortCountFinal(const fer_cd_sap_minmax_t *m,
                                uint32_t len,
                                uint32_t *counter,
                                uint32_t *negative,
                                uint32_t shift);
static void radixSortFixCounter(uint32_t *counter);
static void radixSortSort(fer_cd_sap_minmax_t *src,
                          fer_cd_sap_minmax_t *dst,
                          uint32_t len,
                          uint32_t *counter,
                          uint32_t shift);
static void radixSortSortFinal(fer_cd_sap_minmax_t *src,
                               uint32_t srclen,
                               fer_cd_sap_minmax_t *dst,
                               uint32_t dstlen,
                               uint32_t *counter,
                               uint32_t negative,
                               uint32_t shift,
                               fer_cd_sap_geom_t *geoms, int axis);

static void sap1RadixSort(fer_cd_sap_t *_sap, int axis)
{
    fer_cd_sap_1_t *sap = (fer_cd_sap_1_t *)_sap;
    fer_cd_sap_1_radix_sort_t *rs;
    fer_cd_sap_minmax_t *src, *dst, *tmp;
    uint32_t shift, i, len;

    rs = &sap->rs;

    // allocate temporary array
    if (rs->minmax_alloc < 2 * sap->sap.geoms_len){
        if (rs->minmax)
            free(rs->minmax);
        rs->minmax_alloc = 2 * sap->sap.geoms_alloc;
        rs->minmax_len = 2 * sap->sap.geoms_len;
        rs->minmax = FER_ALLOC_ARR(fer_cd_sap_minmax_t, rs->minmax_alloc);
    }
    rs->minmax_len = 2 * sap->sap.geoms_len;

    len = (fer_uint_t)sizeof(fer_real_t) - 1;
    shift = 0;
    src = sap->sap.minmax[axis];
    dst = rs->minmax;
    for (i = 0; i < len; i++){
        radixSortCount(src, rs->minmax_len, rs->counter, shift);
        radixSortFixCounter(rs->counter);
        radixSortSort(src, dst, rs->minmax_len, rs->counter, shift);

        shift += 8;
        FER_SWAP(src, dst, tmp);
    }

    radixSortCountFinal(src, rs->minmax_len, rs->counter, &rs->negative, shift);
    radixSortFixCounter(rs->counter);
    radixSortSortFinal(src, rs->minmax_len,
                       dst, rs->minmax_len,
                       rs->counter, rs->negative, shift,
                       sap->sap.geoms, axis);
}

_fer_inline uint32_t radixSortVal(const fer_cd_sap_minmax_t *m,
                                  uint32_t shift)
{
    fer_uint_t val;

    val = ferRealAsUInt(m->val);
    val = (val >> shift) & RADIX_SORT_MASK;

    return val;
}

_fer_inline void radixSortZeroizeCounter(uint32_t *counter)
{
    int i;
    for (i = 0; i < 256; i++)
        counter[i] = 0;
}

static void radixSortCount(const fer_cd_sap_minmax_t *m,
                           uint32_t len,
                           uint32_t *counter, uint32_t shift)
{
    uint32_t i;
    fer_uint_t val;

    radixSortZeroizeCounter(counter);
    for (i = 0; i < len; i++){
        val = radixSortVal(&m[i], shift);
        ++counter[val];
    }
}

static void radixSortCountFinal(const fer_cd_sap_minmax_t *m,
                                uint32_t len,
                                uint32_t *counter,
                                uint32_t *negative,
                                uint32_t shift)
{
    uint32_t i;
    fer_uint_t val;

    radixSortZeroizeCounter(counter);
    *negative = 0;
    for (i = 0; i < len; i++){
        val = radixSortVal(&m[i], shift);
        ++counter[val];

        if (m[i].val < FER_ZERO)
            ++(*negative);
    }
}

static void radixSortFixCounter(uint32_t *counter)
{
    uint32_t i, val, t;

    val = 0;
    for (i = 0; i < 256; i++){
        t = counter[i];
        counter[i] = val;
        val += t;
    }
}

static void radixSortSort(fer_cd_sap_minmax_t *src,
                          fer_cd_sap_minmax_t *dst,
                          uint32_t len,
                          uint32_t *counter,
                          uint32_t shift)
{
    uint32_t i;
    fer_uint_t val;

    for (i = 0; i < len; i++){
        val = radixSortVal(&src[i], shift);
        dst[counter[val]] = src[i];
        ++counter[val];
    }
}

static void radixSortSortFinal(fer_cd_sap_minmax_t *src,
                               uint32_t srclen,
                               fer_cd_sap_minmax_t *dst,
                               uint32_t dstlen,
                               uint32_t *counter,
                               uint32_t negative,
                               uint32_t shift,
                               fer_cd_sap_geom_t *geoms, int axis)
{
    uint32_t i;
    fer_uint_t val, pos;

    for (i = 0; i < srclen; i++){
        val = radixSortVal(&src[i], shift);
        pos = counter[val];
        if (pos >= dstlen - negative){
            pos = dstlen - pos - 1;
        }else{
            pos = negative + pos;
        }

        dst[pos] = src[i];

        if (MINMAX_ISMAX(&dst[pos])){
            geoms[MINMAX_GEOM(&dst[pos])].max[axis] = pos;
        }else{
            geoms[MINMAX_GEOM(&dst[pos])].min[axis] = pos;
        }

        ++counter[val];
    }
}



static void sap1FindPairsGeom(fer_cd_sap_1_t *sap, fer_cd_sap_geom_t *geom);

static void sap1FindPairs(fer_cd_sap_t *_sap)
{
    fer_cd_sap_1_t *sap = (fer_cd_sap_1_t *)_sap;
    size_t i;

    for (i = 0; i < sap->sap.geoms_len; i++){
        sap1FindPairsGeom(sap, &sap->sap.geoms[i]);
    }
}

static void sap1FindPairsGeom(fer_cd_sap_1_t *sap, fer_cd_sap_geom_t *geom)
{
    fer_cd_sap_minmax_t *minmax;
    fer_cd_geom_t *g2;
    fer_cd_sap_pair_t *pair, pair_test;
    int diff[3], d, i;

    diff[0] = geom->max[0] - geom->min[0];
    diff[1] = geom->max[1] - geom->min[1];
    diff[2] = geom->max[2] - geom->min[2];

    if (diff[0] < diff[1]){
        if (diff[0] < diff[2]){
            d = 0;
        }else{
            d = 2;
        }
    }else if (diff[1] < diff[2]){
        d = 1;
    }else{
        d = 2;
    }

    minmax = sap->sap.minmax[d];
    for (i = geom->min[d] + 1; i < geom->max[d]; i++){
        g2 = sap->sap.geoms[MINMAX_GEOM(&minmax[i])].g;
        if (ferCDGeomOBBOverlap(geom->g, g2)){
            pair_test.g[0] = geom->g;
            pair_test.g[1] = g2;

            if (ferHMapGet(sap->pairs_reg, &pair_test.hmap) == NULL){
                // create new pair
                pair = pairNew(geom->g, g2);

                // add this pair to list of pairs (into specified bucket)
                ferListAppend(&sap->sap.pairs[0], &pair->list);
                sap->sap.pairs_len++;

                // add pair into register
                ferHMapPut(sap->pairs_reg, &pair->hmap);
            }
        }
    }
}
