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

#ifndef __FER_VEC_H__
#define __FER_VEC_H__

#include <fermat/core.h>
#include <fermat/vec2.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Vec - Genral Vectors
 * =====================
 *
 */
typedef fer_real_t fer_vec_t;

/**
 * Static declaration of vector.
 */
#define FER_VEC(name, size) \
    fer_vec_t name[size]


/** TODO: ToVec2/3/4 functions */

/**
 * Cast from Vec2 to Vec.
 */
_fer_inline fer_vec_t *ferVecFromVec2(fer_vec2_t *v);

/**
 * Cast from Vec2 to Vec.
 */
_fer_inline const fer_vec_t *ferVecFromVec2Const(const fer_vec2_t *v);


/**
 * Functions
 * ----------
 */

/**
 * Allocate and initialize new vector.
 */
fer_vec_t *ferVecNew(size_t size);

/**
 * Delete vector.
 */
void ferVecDel(fer_vec_t *);

/**
 * Clone given fer_vec_t that must be size length. This does deep copy.
 */
_fer_inline fer_vec_t *ferVecClone(size_t size, const fer_vec_t *v);

/**
 * v = w
 */
_fer_inline void ferVecCopy(size_t size, fer_vec_t *v, const fer_vec_t *w);


_fer_inline fer_real_t ferVecGet(const fer_vec_t *v, size_t d);

_fer_inline void ferVecSet(fer_vec_t *v, size_t d, fer_real_t val);
_fer_inline void ferVecSetAll(size_t size, fer_vec_t *v, fer_real_t val);
_fer_inline void ferVecSetZero(size_t size, fer_vec_t *v);


/**
 * Returns squared length of vector.
 */
_fer_inline fer_real_t ferVecLen2(size_t size, const fer_vec_t *v);
_fer_inline fer_real_t ferVecLen(size_t size, const fer_vec_t *v);

/**
 * Returns squared distance between a and b.
 */
_fer_inline fer_real_t ferVecDist2(size_t size, const fer_vec_t *a, const fer_vec_t *b);
_fer_inline fer_real_t ferVecDist(size_t size, const fer_vec_t *a, const fer_vec_t *b);


/**
 * Adds coordinates of vector w to vector v. v = v + w
 */
_fer_inline void ferVecAdd(size_t size, fer_vec_t *v, const fer_vec_t *w);

/**
 * d = v + w
 */
_fer_inline void ferVecAdd2(size_t size, fer_vec_t *d, const fer_vec_t *v, const fer_vec_t *w);


/**
 * Substracts coordinates of vector w from vector v. v = v - w
 */
_fer_inline void ferVecSub(size_t size, fer_vec_t *v, const fer_vec_t *w);

/**
 * d = v - w
 */
_fer_inline void ferVecSub2(size_t size, fer_vec_t *d, const fer_vec_t *v, const fer_vec_t *w);

_fer_inline void ferVecAddConst(size_t size, fer_vec_t *v, fer_real_t f);
_fer_inline void ferVecAddConst2(size_t size, fer_vec_t *d, const fer_vec_t *v, fer_real_t f);
_fer_inline void ferVecSubConst(size_t size, fer_vec_t *v, fer_real_t f);
_fer_inline void ferVecSubConst2(size_t size, fer_vec_t *d, const fer_vec_t *v, fer_real_t f);


/**
 * d = d * k;
 */
_fer_inline void ferVecScale(size_t size, fer_vec_t *d, fer_real_t k);

/**
 * Dot product of two vectors.
 */
_fer_inline fer_real_t ferVecDot(size_t size, const fer_vec_t *a, const fer_vec_t *b);

/**
 * Multiplies vectors component wise:
 * a.x = a.x * b.x
 * a.y = a.y * b.y
 * a.z = a.z * b.z
 */
_fer_inline void ferVecMulComp(size_t size, fer_vec_t *a, const fer_vec_t *b);

/**
 * a.x = b.x * c.x
 * a.y = b.y * c.y
 * a.z = b.z * c.z
 */
_fer_inline void ferVecMulComp2(size_t size, fer_vec_t *a, const fer_vec_t *b, const fer_vec_t *c);




/**** INLINES ****/
_fer_inline fer_vec_t *ferVecFromVec2(fer_vec2_t *v)
{
    return (fer_vec_t *)v;
}

_fer_inline const fer_vec_t *ferVecFromVec2Const(const fer_vec2_t *v)
{
    return (const fer_vec_t *)v;
}


_fer_inline fer_vec_t *ferVecClone(size_t size, const fer_vec_t *v)
{
    fer_vec_t *w;
    w = ferVecNew(size);
    ferVecCopy(size, w, v);
    return w;
}

_fer_inline fer_real_t ferVecGet(const fer_vec_t *v, size_t d)
{
    return v[d];
}


_fer_inline fer_real_t ferVecLen2(size_t size, const fer_vec_t *v)
{
    return ferVecDot(size, v, v);
}
_fer_inline fer_real_t ferVecLen(size_t size, const fer_vec_t *v)
{
    return FER_SQRT(ferVecLen2(size, v));
}

_fer_inline fer_real_t ferVecDist2(size_t size, const fer_vec_t *a, const fer_vec_t *b)
{
    fer_real_t f, dot;
    size_t i;

    dot = FER_ZERO;
    for (i = 0; i < size; i++){
        f = a[i] - b[i];
        dot += FER_CUBE(f);
    }

    return dot;
}
_fer_inline fer_real_t ferVecDist(size_t size, const fer_vec_t *a, const fer_vec_t *b)
{
    return FER_SQRT(ferVecDist2(size, a, b));
}

_fer_inline void ferVecSet(fer_vec_t *v, size_t d, fer_real_t val)
{
    v[d] = val;
}

_fer_inline void ferVecSetAll(size_t size, fer_vec_t *v, fer_real_t val)
{
    size_t i;
    for (i = 0; i < size; i++){
        v[i] = val;
    }
}

_fer_inline void ferVecSetZero(size_t size, fer_vec_t *v)
{
    ferVecSetAll(size, v, FER_ZERO);
}


_fer_inline void ferVecCopy(size_t size, fer_vec_t *v, const fer_vec_t *w)
{
    size_t i;
    for (i = 0; i < size; i++){
        v[i] = w[i];
    }
}

_fer_inline void ferVecAdd(size_t size, fer_vec_t *v, const fer_vec_t *w)
{
    size_t i;
    for (i = 0; i < size; i++){
        v[i] += w[i];
    }
}

_fer_inline void ferVecAdd2(size_t size, fer_vec_t *d, const fer_vec_t *v, const fer_vec_t *w)
{
    size_t i;
    for (i = 0; i < size; i++){
        d[i] = v[i] + w[i];
    }
}

_fer_inline void ferVecSub(size_t size, fer_vec_t *v, const fer_vec_t *w)
{
    size_t i;
    for (i = 0; i < size; i++){
        v[i] -= w[i];
    }
}
_fer_inline void ferVecSub2(size_t size, fer_vec_t *d, const fer_vec_t *v, const fer_vec_t *w)
{
    size_t i;
    for (i = 0; i < size; i++){
        d[i] = v[i] - w[i];
    }
}

_fer_inline void ferVecAddConst(size_t size, fer_vec_t *v, fer_real_t f)
{
    size_t i;
    for (i = 0; i < size; i++){
        v[i] += f;
    }
}

_fer_inline void ferVecAddConst2(size_t size, fer_vec_t *d, const fer_vec_t *v, fer_real_t f)
{
    size_t i;
    for (i = 0; i < size; i++){
        d[i] = v[i] + f;
    }
}

_fer_inline void ferVecSubConst(size_t size, fer_vec_t *v, fer_real_t f)
{
    ferVecAddConst(size, v, -f);
}

_fer_inline void ferVecSubConst2(size_t size, fer_vec_t *d, const fer_vec_t *v, fer_real_t f)
{
    ferVecAddConst2(size, d, v, -f);
}

_fer_inline void ferVecScale(size_t size, fer_vec_t *d, fer_real_t k)
{
    size_t i;
    for (i = 0; i < size; i++){
        d[i] *= k;
    }
}


_fer_inline fer_real_t ferVecDot(size_t size, const fer_vec_t *a, const fer_vec_t *b)
{
    fer_real_t dot;
    size_t i;

    dot = FER_ZERO;
    for (i = 0; i < size; i++){
        dot += a[i] * b[i];
    }

    return dot;
}

_fer_inline void ferVecMulComp(size_t size, fer_vec_t *a, const fer_vec_t *b)
{
    size_t i;
    for (i = 0; i < size; i++){
        a[i] *= b[i];
    }
}

_fer_inline void ferVecMulComp2(size_t size, fer_vec_t *a, const fer_vec_t *b, const fer_vec_t *c)
{
    size_t i;
    for (i = 0; i < size; i++){
        a[i] = b[i] * c[i];
    }
}


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __FER_VEC_H__ */