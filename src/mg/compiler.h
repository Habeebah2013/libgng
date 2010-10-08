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

#ifndef __MG_COMPILER_H__
#define __MG_COMPILER_H__


#define mg_offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#define mg_container_of(ptr, type, member) \
    (type *)( (char *)ptr - mg_offsetof(type, member))


/**
 * Marks inline function.
 */
#ifdef __GNUC__
# define _mg_inline static inline __attribute__((always_inline))
#else /* __GNUC__ */
# define _mg_inline static inline
#endif /* __GNUC__ */


/**
 * __prefetch(x)  - prefetches the cacheline at "x" for read
 * __prefetchw(x) - prefetches the cacheline at "x" for write
 */
#ifdef __GNUC__
# define _mg_prefetch(x) __builtin_prefetch(x)
# define _mg_prefetchw(x) __builtin_prefetch(x,1)
#else /* __GNUC__ */
# define _mg_prefetch(x)
# define _mg_prefetchw(x)
#endif /* __GNUC__ */


#ifdef __ICC
// disable unused parameter warning
# pragma warning(disable:869)
// disable annoying "operands are evaluated in unspecified order" warning
# pragma warning(disable:981)
#endif /* __ICC */

#endif /* __MG_COMPILER_H__ */

