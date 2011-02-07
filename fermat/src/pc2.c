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

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <fermat/pc2.h>
#include <fermat/rand.h>
#include <fermat/parse.h>
#include <fermat/alloc.h>
#include <fermat/dbg.h>

/**
 * Updates given AABB using given point.
 */
_fer_inline void ferPC2AABBUpdate(fer_real_t *aabb, fer_real_t x, fer_real_t y);

fer_pc2_t *ferPC2New(void)
{
    fer_pc2_t *pc;

    pc = FER_ALLOC(fer_pc2_t);
    ferListInit(&pc->head);
    pc->len = 0;
    pc->aabb[0] = pc->aabb[2] = FER_REAL_MAX;
    pc->aabb[1] = pc->aabb[3] = FER_REAL_MIN;

    pc->min_chunk_size = FER_PC2_MIN_CHUNK_SIZE;
    

    return pc;
}

#define PC_FUNC(n) ferPC2 ## n
#define PC_T fer_pc2_t
#define VEC_FUNC(n) ferVec2 ## n
#define VEC_T fer_vec2_t
#define PC_UPDATE(pc, v) \
    ferPC2AABBUpdate((pc)->aabb, ferVec2X(v), ferVec2Y(v))
#define PC_PARSE_VEC ferParseVec2

#include "pc-common.c"

#if 0
void ferPC2Add(fer_pc2_t *pc, const fer_vec2_t *v)
{
    fer_list_t *item;
    fer_pc_mem_t *mem;

    item = ferListPrev(&pc->head);
    mem = ferListEntry(item, fer_pc_mem_t, list);
    if (ferListEmpty(&pc->head) || ferPCMemFull(mem)){
#ifdef FER_SSE
        mem = ferPCMemNew(pc->min_chunk_size, sizeof(fer_vec2_t), 16);
#else /* FER_SSE */
        mem = ferPCMemNew(pc->min_chunk_size, sizeof(fer_vec2_t), 0);
#endif /* FER_SSE */
        ferListAppend(&pc->head, &mem->list);
    }

    ferPCMemAdd(mem, v, fer_vec2_t);
    ferPC2AABBUpdate(pc->aabb, ferVec2X(v), ferVec2Y(v));
    pc->len++;
}

fer_vec2_t *ferPC2Get(fer_pc2_t *pc, size_t n)
{
    fer_list_t *item;
    fer_pc_mem_t *mem;
    size_t pos;
    fer_vec2_t *p = NULL;

    if (n >= pc->len)
        return NULL;

    pos = 0;
    ferListForEach(&pc->head, item){
        mem = ferListEntry(item, fer_pc_mem_t, list);
        if (pos + mem->len > n){
            p = ferPCMemGet(mem, n - pos, fer_vec2_t);
            break;
        }
        pos += mem->len;
    }

    return p;
}

/** Returns (via other and other_pos) memory chunk and position within it
 *  randomly chosen from point cloud from range starting at position from
 *  of mem chunk mem_from. */
static void ferPC2PermutateOther(fer_pc_mem_t *mem_from, size_t from,
                               size_t len, fer_rand_t *rand,
                               fer_pc_mem_t **other, size_t *other_pos)
{
    size_t pos;
    fer_pc_mem_t *mem;
    fer_list_t *item;

    mem = mem_from;

    // choose position
    pos = ferRand(rand, (fer_real_t)from, (fer_real_t)len);

    // find correct mem chunk
    while (pos >= mem->len){
        pos -= mem->len;
        item = ferListNext(&mem->list);
        mem = ferListEntry(item, fer_pc_mem_t, list);
    }

    *other = mem;
    *other_pos = pos;
}

void ferPC2Permutate(fer_pc2_t *pc)
{
    fer_list_t *item;
    fer_pc_mem_t *cur_mem, *other_mem;
    size_t cur_pos, pc_len, other_pos;
    fer_rand_t rand;
    fer_vec2_t v, *cur, *other;

    ferRandInit(&rand);
    pc_len = pc->len;

    // iterate over all positions in all chunks consequently from beginning
    // and choose some point from rest of point cloud (from positions _after_
    // the current one) and swap points
    ferListForEach(&pc->head, item){
        cur_mem = ferListEntry(item, fer_pc_mem_t, list);
        for (cur_pos = 0; cur_pos < cur_mem->len && pc_len - cur_pos > 1; cur_pos++){
            // choose other point for swapping
            ferPC2PermutateOther(cur_mem, cur_pos + 1, pc_len, &rand,
                               &other_mem, &other_pos);

            // swap points
            cur   = ferPCMemGet(cur_mem, cur_pos, fer_vec2_t);
            other = ferPCMemGet(other_mem, other_pos, fer_vec2_t);
            ferVec2Copy(&v, other);
            ferVec2Copy(other, cur);
            ferVec2Copy(cur, &v);
        }

        // length must be decreased because ferPC2PermutateOther takes
        // length which is relative to first mem chunk
        pc_len -= cur_mem->len;
    }
}

size_t ferPC2AddFromFile(fer_pc2_t *pc, const char *filename)
{

    int fd;
    size_t size;
    struct stat st;
    void *file;
    char *fstr, *fend, *fnext;
    fer_vec2_t v;
    size_t added = 0;

    // open file
    if ((fd = open(filename, O_RDONLY)) == -1){
        ERR("Can't open file '%s'", filename);
        return added;
    }

    // get stats (mainly size of file)
    if (fstat(fd, &st) == -1){
        close(fd);
        ERR("Can't get file info of '%s'", filename);
        return added;
    }

    // pick up size of file
    size = st.st_size;

    // mmap whole file into memory, we need only read from it and don't need
    // to share anything
    file = mmap(0, size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (file == MAP_FAILED){
        close(fd);
        ERR("Can't map file '%s' into memory: %s", filename, strerror(errno));

        // Fall to stdio method if it's not possible to map whole file into
        // memory
        // TODO gsrmInsigAddFromFileSTDIO(is, filename);
        return added;
    }

    // set up char pointers to current char (fstr) and to end of memory (fend)
    fstr = (char *)file;
    fend = (char *)file + size;
    while (ferParseVec2(fstr, fend, &v, &fnext) == 0){
        ferPC2Add(pc, &v);
        added++;
        fstr = fnext;
    }

    // unmap mapped memory
    munmap(file, size);

    // close file
    close(fd);

    return added;
}
#endif

_fer_inline void ferPC2AABBUpdate(fer_real_t *aabb, fer_real_t x, fer_real_t y)
{
    if (aabb[0] > x)
        aabb[0] = x;
    if (aabb[1] < x)
        aabb[1] = x;
    if (aabb[2] > y)
        aabb[2] = y;
    if (aabb[3] < y)
        aabb[3] = y;
}