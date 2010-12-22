/***
 * mg
 * ---
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of mg.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef __MG_CORE_H__
#define __MG_CORE_H__

#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <mg/config.h>
#include <mg/compiler.h>

#ifdef MG_SSE
# include <immintrin.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#ifndef MG_SINGLE
# ifndef MG_DOUBLE
#  error You must define MG_SINGLE or MG_DOUBLE
# endif /* MG_DOUBLE */
#endif /* MG_SINGLE */

#ifdef MG_SSE
union _mg_sse_t {
    __m128 m;
    __m128d md;
    float f[4];
    double d[2];
} mg_aligned(16) mg_packed;
typedef union _mg_sse_t mg_sse_t;
#endif /* MG_SSE */


#ifdef MG_SINGLE
# ifdef MG_DOUBLE
#  error You can define either MG_SINGLE or MG_DOUBLE, not both!
# endif /* MG_DOUBLE */

typedef float mg_real_t;

//# define MG_EPS 1E-6
# define MG_EPS FLT_EPSILON
# define MG_REAL_MAX FLT_MAX
# define MG_REAL_MIN FLT_MIN

# define MG_REAL_STR "%.30f"

# define MG_REAL(x)     (x ## f)          /*!< form a constant */

# ifdef MG_SSE
#  define MG_SQRT(x)     (__mgSqrt(x))     /*!< square root */
//#  define MG_SQRT(x)     (sqrtf(x))       /*!< square root */
# else /* MG_SSE */
#  define MG_SQRT(x)     (sqrtf(x))       /*!< square root */
# endif /* MG_SSE */

# define MG_FABS(x)     (fabsf(x))        /*!< absolute value */
# define MG_FMAX(x, y)  (fmaxf((x), (y))) /*!< maximum of two floats */
# define MG_FMIN(x, y)  (fminf((x), (y))) /*!< minimum of two floats */
# define MG_CBRT(x)     (cbrtf(x)         /*!< cube root */
# define MG_COS(x)      (cosf(x))         /*!< cosine */
# define MG_SIN(x)      (sinf(x))         /*!< sine */
# define MG_ACOS(x)     (acosf(x))        /*!< arc cosine */
# define MG_ASIN(x)     (asinf(x))        /*!< arc sine */
# define MG_ATAN2(y, x) (atan2f(ym x))    /*!< arc tangent of y/x */
# define MG_POW(x, y)   (powf((x), (y)))  /*!< power function */

#endif /* MG_SINGLE */

#ifdef MG_DOUBLE
typedef double mg_real_t;

//# define MG_EPS 1E-10
# define MG_EPS DBL_EPSILON
# define MG_REAL_MAX DBL_MAX
# define MG_REAL_MIN DBL_MIN

# define MG_REAL_STR "%.30lf"

# define MG_REAL(x)     (x ## f)         /*!< form a constant */

# ifdef MG_SSE
#  define MG_SQRT(x)     (__mgSqrt(x))     /*!< square root */
//#  define MG_SQRT(x)     (sqrt(x))       /*!< square root */
# else /* MG_SSE */
#  define MG_SQRT(x)     (sqrt(x))       /*!< square root */
# endif /* MG_SSE */

# define MG_FABS(x)     (fabs(x))        /*!< absolute value */
# define MG_FMAX(x, y)  (fmax((x), (y))) /*!< maximum of two floats */
# define MG_FMIN(x, y)  (fmin((x), (y))) /*!< minimum of two floats */
# define MG_CBRT(x)     (cbrt(x)         /*!< cube root */
# define MG_COS(x)      (cos(x))         /*!< cosine */
# define MG_SIN(x)      (sin(x))         /*!< sine */
# define MG_ACOS(x)     (acos(x))        /*!< arc cosine */
# define MG_ASIN(x)     (asin(x))        /*!< arc sine */
# define MG_ATAN2(y, x) (atan2(ym x))    /*!< arc tangent of y/x */
# define MG_POW(x, y)   (pow((x), (y)))  /*!< power function */

#endif /* MG_DOUBLE */

#define MG_ONE MG_REAL(1.)
#define MG_ZERO MG_REAL(0.)
#define MG_CUBE(x) ((x) * (x))


#ifdef MG_SSE
_mg_inline mg_real_t __mgSqrt(mg_real_t val);
#endif /* MG_SSE */

/** Returns sign of value. */
_mg_inline int mgSign(mg_real_t val);
/** Returns true if val is zero. **/
_mg_inline int mgIsZero(mg_real_t val);
/** Returns true if a and b equal. **/
_mg_inline int mgEq(mg_real_t a, mg_real_t b);
_mg_inline int mgNEq(mg_real_t a, mg_real_t b);

/**
 * Returns 1 / d.
 */
_mg_inline mg_real_t mgRecp(mg_real_t v);

/**
 * Returns 1 / sqrt(v)
 */
_mg_inline mg_real_t mgRsqrt(mg_real_t v);


/***** INLINES *****/
#ifdef MG_SSE
_mg_inline mg_real_t __mgSqrt(mg_real_t val)
{
    mg_sse_t m;

#ifdef MG_SSE_SINGLE
    m.m = _mm_set1_ps(val);
    m.m = _mm_sqrt_ps(m.m);
    return m.f[0];
#else /* MG_SSE_SINGLE */
    m.md = _mm_set1_pd(val);
    m.md = _mm_sqrt_pd(m.md);
    return m.d[0];
#endif /* MG_SSE_SINGLE */
}
#endif /* MG_SSE */

_mg_inline int mgSign(mg_real_t val)
{
    if (mgIsZero(val)){
        return 0;
    }else if (val < MG_ZERO){
        return -1;
    }
    return 1;
}

_mg_inline int mgIsZero(mg_real_t val)
{
    return MG_FABS(val) < MG_EPS;
}

_mg_inline int mgEq(mg_real_t _a, mg_real_t _b)
{
    mg_real_t ab;

    ab = MG_FABS(_a - _b);
    if (ab < MG_EPS)
        return 1;

    mg_real_t a, b;
    a = MG_FABS(_a);
    b = MG_FABS(_b);
    if (b > a){
        return ab < MG_EPS * b;
    }else{
        return ab < MG_EPS * a;
    }
}

_mg_inline int mgNEq(mg_real_t a, mg_real_t b)
{
    return !mgEq(a, b);
}

_mg_inline mg_real_t mgRecp(mg_real_t v)
{
#ifdef MG_SSE_SINGLE
    mg_sse_t m;
    m.m = _mm_set1_ps(v);
    m.m = _mm_rcp_ps(m.m);
    return m.f[0];
#else /* MG_SSE_SINGLE */
    return MG_ONE / v;
#endif /* MG_SSE_SINGLE */
}

_mg_inline mg_real_t mgRsqrt(mg_real_t v)
{
#ifdef MG_SSE_SINGLE
    mg_sse_t m;
    m.m = _mm_set1_ps(v);
    m.m = _mm_rsqrt_ps(m.m);
    return m.f[0];
#else /* MG_SSE_SINGLE */
    return MG_ONE / MG_SQRT(v);
# endif /* MG_SSE_SINGLE */
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __MG_CORE_H__ */
