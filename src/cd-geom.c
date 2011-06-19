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

#include <fermat/cd.h>
#include <fermat/alloc.h>
#include <fermat/dbg.h>

fer_cd_geom_t *ferCDGeomNew(fer_cd_t *cd)
{
    fer_cd_geom_t *g;

    g = FER_ALLOC_ALIGN(fer_cd_geom_t, 16);
    ferVec3Set(&g->tr, FER_ZERO, FER_ZERO, FER_ZERO);
    ferMat3SetIdentity(&g->rot);
    ferListInit(&g->obbs);

    // add to list of all gemos
    ferListAppend(&cd->geoms, &g->list);
    cd->geoms_len++;
    ferListAppend(&cd->geoms_dirty, &g->list_dirty);
    cd->geoms_dirty_len++;

    g->data = NULL;

    g->sap = -1;

    if (cd->sap)
        ferCDSAPAdd(cd->sap, g);

    return g;
}

void ferCDGeomDel(fer_cd_t *cd, fer_cd_geom_t *g)
{
    fer_list_t *item;
    fer_cd_obb_t *obb;

    while (!ferListEmpty(&g->obbs)){
        item = ferListNext(&g->obbs);
        ferListDel(item);
        obb = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
        ferCDOBBDel(obb);
    }

    ferListDel(&g->list);
    cd->geoms_len--;
    ferListDel(&g->list_dirty);
    cd->geoms_dirty_len--;

    if (cd->sap && g->sap)
        ferCDSAPRemove(cd->sap, g);

    if (g->sap >= 0)
        ferCDSAPRemove(cd->sap, g);

    free(g);
}

void ferCDGeomBuild(fer_cd_t *cd, fer_cd_geom_t *g)
{
    ferCDOBBMerge(&g->obbs, cd->build_flags);
    ferCDGeomSetDirty(cd, g);
}


static void _ferCDGeomAddShape(fer_cd_t *cd, fer_cd_geom_t *g,
                               fer_cd_shape_t *shape)
{
    fer_cd_obb_t *obb;

    obb = ferCDOBBNewShape((fer_cd_shape_t *)shape, cd->build_flags);
    ferListAppend(&g->obbs, &obb->list);

    ferCDGeomSetDirty(cd, g);
}

static void _ferCDGeomAddShape2(fer_cd_t *cd, fer_cd_geom_t *g,
                                fer_cd_shape_t *shape,
                                const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    fer_cd_shape_off_t *off;
    fer_cd_obb_t *obb;

    off = ferCDShapeOffNew((fer_cd_shape_t *)shape, rot, tr);
    obb = ferCDOBBNewShape((fer_cd_shape_t *)off, cd->build_flags);
    ferListAppend(&g->obbs, &obb->list);

    ferCDGeomSetDirty(cd, g);
}

void ferCDGeomAddSphere(fer_cd_t *cd, fer_cd_geom_t *g, fer_real_t radius)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDSphereNew(radius));
}

void ferCDGeomAddSphere2(fer_cd_t *cd, fer_cd_geom_t *g, fer_real_t radius,
                         const fer_vec3_t *tr)
{
    _ferCDGeomAddShape2(cd, g, (fer_cd_shape_t *)ferCDSphereNew(radius),
                        fer_mat3_identity, tr);
}


void ferCDGeomAddBox(fer_cd_t *cd, fer_cd_geom_t *g,
                     fer_real_t lx, fer_real_t ly, fer_real_t lz)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDBoxNew(lx, ly, lz));
}

void ferCDGeomAddBox2(fer_cd_t *cd, fer_cd_geom_t *g,
                      fer_real_t lx, fer_real_t ly, fer_real_t lz,
                      const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    _ferCDGeomAddShape2(cd, g, (fer_cd_shape_t *)ferCDBoxNew(lx, ly, lz),
                        rot, tr);
}


void ferCDGeomAddCyl(fer_cd_t *cd, fer_cd_geom_t *g,
                     fer_real_t radius, fer_real_t height)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDCylNew(radius, height));
}

void ferCDGeomAddCyl2(fer_cd_t *cd, fer_cd_geom_t *g,
                      fer_real_t radius, fer_real_t height,
                      const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    _ferCDGeomAddShape2(cd, g, (fer_cd_shape_t *)ferCDCylNew(radius, height),
                        rot, tr);
}

void ferCDGeomAddCap(fer_cd_t *cd, fer_cd_geom_t *g,
                     fer_real_t radius, fer_real_t height)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDCapNew(radius, height));
}

void ferCDGeomAddCap2(fer_cd_t *cd, fer_cd_geom_t *g,
                      fer_real_t radius, fer_real_t height,
                      const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    _ferCDGeomAddShape2(cd, g, (fer_cd_shape_t *)ferCDCapNew(radius, height),
                        rot, tr);
}

void ferCDGeomAddPlane(fer_cd_t *cd, fer_cd_geom_t *g)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDPlaneNew());
}

void ferCDGeomAddPlane2(fer_cd_t *cd, fer_cd_geom_t *g,
                        const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    _ferCDGeomAddShape2(cd, g, (fer_cd_shape_t *)ferCDPlaneNew(),
                        rot, tr);
}


void ferCDGeomAddTri(fer_cd_t *cd, fer_cd_geom_t *g,
                     const fer_vec3_t *p0, const fer_vec3_t *p1,
                     const fer_vec3_t *p2)
{
    _ferCDGeomAddShape(cd, g, (fer_cd_shape_t *)ferCDTriNew(p0, p1, p2));
}

void ferCDGeomAddTriMesh(fer_cd_t *cd, fer_cd_geom_t *g,
                         const fer_vec3_t *pts,
                         const unsigned int *ids, size_t len)
{
    ferCDGeomAddTriMesh2(cd, g, pts, ids, len, fer_mat3_identity, fer_vec3_origin);
    ferCDGeomSetDirty(cd, g);
}

void ferCDGeomAddTriMesh2(fer_cd_t *cd, fer_cd_geom_t *g,
                          const fer_vec3_t *pts,
                          const unsigned int *ids, size_t len,
                          const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    fer_cd_trimesh_t *t;
    fer_cd_obb_t *obb;

    t   = ferCDTriMeshNew(pts, ids, len, rot, tr);
    obb = ferCDOBBNewTriMesh(t, cd->build_flags);
    ferListAppend(&g->obbs, &obb->list);

    ferCDGeomSetDirty(cd, g);
}

void ferCDGeomAddTrisFromRaw(fer_cd_t *cd, fer_cd_geom_t *g,
                             const char *filename)
{
    ferCDGeomAddTrisFromRawScale(cd, g, filename, FER_ONE);
    ferCDGeomSetDirty(cd, g);
}

void ferCDGeomAddTrisFromRawScale(fer_cd_t *cd, fer_cd_geom_t *g,
                                  const char *filename, fer_real_t scale)
{
    FILE *fin;
    float ax, ay, az, bx, by, bz, cx, cy, cz;
    fer_vec3_t p0, p1, p2;
    fer_cd_tri_t *tri;
    fer_cd_obb_t *obb;
    size_t zero_tris;

    fin = fopen(filename, "r");
    if (!fin){
        fprintf(stderr, "CD Error: Can't open file `%s'\n", filename);
        return;
    }

    zero_tris = 0;
    while (fscanf(fin, "%f %f %f %f %f %f %f %f %f",
                  &ax, &ay, &az, &bx, &by, &bz, &cx, &cy, &cz) == 9){
        ferVec3Set(&p0, ax, ay, az);
        ferVec3Set(&p1, bx, by, bz);
        ferVec3Set(&p2, cx, cy, cz);
        ferVec3Scale(&p0, scale);
        ferVec3Scale(&p1, scale);
        ferVec3Scale(&p2, scale);

        if (ferIsZero(FER_REAL(0.5) * ferVec3TriArea2(&p0, &p1, &p2))){
            zero_tris++;
            continue;
        }

        tri = ferCDTriNew(&p0, &p1, &p2);
        obb = ferCDOBBNewShape((fer_cd_shape_t *)tri, cd->build_flags);
        ferListAppend(&g->obbs, &obb->list);
    }

    if (zero_tris > 0){
        fprintf(stderr, "CD Warning: ferCDGeomAddTrisFromRaw(): %d triangles"
                        " with zero area ignored.\n", (int)zero_tris);
    }

    fclose(fin);

    ferCDGeomSetDirty(cd, g);
}


struct __collide_t {
    fer_cd_t *cd;
    const fer_cd_geom_t *g1;
    const fer_cd_geom_t *g2;
    int ret;
};

static int __ferCDGeomCollideCB(const fer_cd_obb_t *obb1,
                                const fer_cd_obb_t *obb2,
                                void *data)
{
    struct __collide_t *c = (struct __collide_t *)data;

    c->ret = __ferCDShapeCollide(c->cd,
                                 obb1->shape, &c->g1->rot, &c->g1->tr,
                                 obb2->shape, &c->g2->rot, &c->g2->tr);

    if (c->ret)
        return -1;
    return 0;
}

int ferCDGeomCollide(fer_cd_t *cd,
                     const fer_cd_geom_t *g1, const fer_cd_geom_t *g2)
{
    struct __collide_t c;
    fer_list_t *item1, *item2;
    fer_cd_obb_t *obb1, *obb2;

    c.cd = cd;
    c.g1 = g1;
    c.g2 = g2;

    FER_LIST_FOR_EACH(&g1->obbs, item1){
        obb1 = FER_LIST_ENTRY(item1, fer_cd_obb_t, list);

        FER_LIST_FOR_EACH(&g2->obbs, item2){
            obb2 = FER_LIST_ENTRY(item2, fer_cd_obb_t, list);

            c.ret = 0;
            ferCDOBBOverlapPairsCB(obb1, &g1->rot, &g1->tr,
                                   obb2, &g2->rot, &g2->tr,
                                   __ferCDGeomCollideCB, (void *)&c);
            if (c.ret)
                return 1;
        }
    }
    return 0;
}

struct __separate_t {
    fer_cd_t *cd;
    const fer_cd_geom_t *g1;
    const fer_cd_geom_t *g2;
    fer_cd_contacts_t *con;
};
static int __ferCDGeomSeparateCB(const fer_cd_obb_t *obb1,
                                 const fer_cd_obb_t *obb2,
                                 void *data)
{
    struct __separate_t *sep = (struct __separate_t *)data;

    __ferCDShapeSeparate(sep->cd,
                         obb1->shape, &sep->g1->rot, &sep->g1->tr,
                         obb2->shape, &sep->g2->rot, &sep->g2->tr,
                         sep->con);

    return 0;
}

int ferCDGeomSeparate(fer_cd_t *cd,
                      const fer_cd_geom_t *g1, const fer_cd_geom_t *g2,
                      fer_cd_contacts_t *con)
{
    struct __separate_t sep;
    fer_list_t *item1, *item2;
    fer_cd_obb_t *obb1, *obb2;
    int num = con->num;

    sep.cd = cd;
    sep.g1 = g1;
    sep.g2 = g2;
    sep.con = con;

    FER_LIST_FOR_EACH(&g1->obbs, item1){
        obb1 = FER_LIST_ENTRY(item1, fer_cd_obb_t, list);

        FER_LIST_FOR_EACH(&g2->obbs, item2){
            obb2 = FER_LIST_ENTRY(item2, fer_cd_obb_t, list);

            ferCDOBBOverlapPairsCB(obb1, &g1->rot, &g1->tr,
                                   obb2, &g2->rot, &g2->tr,
                                   __ferCDGeomSeparateCB, (void *)&sep);
        }
    }

    return con->num - num;
}

int ferCDGeomOBBOverlap(const fer_cd_geom_t *g1, const fer_cd_geom_t *g2)
{
    fer_list_t *item1, *item2;
    fer_cd_obb_t *obb1, *obb2;
    int ret = 0;

    FER_LIST_FOR_EACH(&g1->obbs, item1){
        obb1 = FER_LIST_ENTRY(item1, fer_cd_obb_t, list);

        FER_LIST_FOR_EACH(&g2->obbs, item2){
            obb2 = FER_LIST_ENTRY(item2, fer_cd_obb_t, list);

            if (!ferCDOBBDisjoint(obb1, &g1->rot, &g1->tr,
                                  obb2, &g2->rot, &g2->tr)){
                ret = 1;
                break;
            }
        }
    }

    return ret;
}

void ferCDGeomSetDirty(fer_cd_t *cd, fer_cd_geom_t *g)
{
    if (!ferCDGeomDirty(cd, g)){
        ferListAppend(&cd->geoms_dirty, &g->list_dirty);
        cd->geoms_dirty_len++;
    }
}

void __ferCDGeomResetDirty(fer_cd_t *cd, fer_cd_geom_t *g)
{
    ferListDel(&g->list_dirty);
    cd->geoms_dirty_len--;
}

static void dumpSVT(const fer_cd_geom_t *g,
                    fer_cd_obb_t *obb, FILE *out, const char *name)
{
    fer_list_t *item;
    fer_cd_obb_t *o;

    if (obb->shape){
        if (obb->shape->cl->dump_svt){
            obb->shape->cl->dump_svt(obb->shape, out, name, &g->rot, &g->tr);
        }
    }else{
        FER_LIST_FOR_EACH(&obb->obbs, item){
            o = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
            dumpSVT(g, o, out, name);
        }
    }
}

void ferCDGeomDumpSVT(const fer_cd_geom_t *g, FILE *out, const char *name)
{
    fer_list_t *item;
    fer_cd_obb_t *obb;

    FER_LIST_FOR_EACH(&g->obbs, item){
        obb = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
        dumpSVT(g, obb, out, name);
    }
}

void ferCDGeomDumpOBBSVT(const fer_cd_geom_t *g, FILE *out, const char *name)
{
    fer_list_t *item;
    fer_cd_obb_t *obb;

    FER_LIST_FOR_EACH(&g->obbs, item){
        obb = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
        ferCDOBBDumpTreeSVT(obb, out, name, &g->rot, &g->tr);
    }
}

static size_t _ferCDGeomDumpTriSVT(const fer_cd_obb_t *obb, FILE *out,
                                   const fer_mat3_t *rot, const fer_vec3_t *tr)
{
    fer_list_t *item;
    fer_cd_obb_t *o;
    fer_cd_tri_t *t;
    fer_vec3_t v;
    size_t s = 0;

    if (ferListEmpty(&obb->obbs)){
        if (obb->shape && (obb->shape->cl->type == FER_CD_SHAPE_TRI
                            || obb->shape->cl->type == FER_CD_SHAPE_TRIMESH_TRI)){
            t = (fer_cd_tri_t *)obb->shape;

            ferMat3MulVec(&v, rot, t->p[0]);
            ferVec3Add(&v, tr);
            ferVec3Print(&v, out);
            fprintf(out, "\n");

            ferMat3MulVec(&v, rot, t->p[1]);
            ferVec3Add(&v, tr);
            ferVec3Print(&v, out);
            fprintf(out, "\n");

            ferMat3MulVec(&v, rot, t->p[2]);
            ferVec3Add(&v, tr);
            ferVec3Print(&v, out);
            fprintf(out, "\n");
            s += 3;
        }
    }else{
        FER_LIST_FOR_EACH(&obb->obbs, item){
            o = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
            s += _ferCDGeomDumpTriSVT(o, out, rot, tr);
        }
    }

    return s;
}

void ferCDGeomDumpTriSVT(const fer_cd_geom_t *g, FILE *out, const char *name)
{
    fer_list_t *item;
    fer_cd_obb_t *obb;
    size_t numpts, i;

    fprintf(out, "-----\n");

    if (name)
        fprintf(out, "Name: %s\n", name);

    numpts = 0;
    fprintf(out, "Points:\n");
    FER_LIST_FOR_EACH(&g->obbs, item){
        obb = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
        numpts += _ferCDGeomDumpTriSVT(obb, out, &g->rot, &g->tr);
    }

    fprintf(out, "Faces:\n");
    for (i = 0; i < numpts; i += 3){
        fprintf(out, "%d %d %d\n", (int)i, (int)i + 1, (int)i + 2);
    }

    fprintf(out, "-----\n");
}


void __ferCDGeomSetMinMax(const fer_cd_geom_t *g,
                          const fer_vec3_t *axis,
                          fer_real_t *min, fer_real_t *max)
{
    fer_list_t *item;
    fer_cd_obb_t *obb;

    *min = FER_REAL_MAX;
    *max = -FER_REAL_MAX;
    FER_LIST_FOR_EACH(&g->obbs, item){
        obb = FER_LIST_ENTRY(item, fer_cd_obb_t, list);
        __ferCDOBBUpdateMinMax(obb, axis, &g->rot, &g->tr, min, max);
    }
}
