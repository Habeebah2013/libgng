/***
 * Svoboda
 * --------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
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

#include <boruvka/cfg.h>
#include <hooke/hooke.h>
#include <boruvka/alloc.h>
#include <boruvka/dbg.h>
#include <boruvka/vec.h>
#include <boruvka/vec3.h>
#include <boruvka/mat3.h>
#include <boruvka/rand-mt.h>


static bor_cfg_t *cfg = NULL;
static bor_rand_mt_t *rnd = NULL;

static hke_t *cd = NULL;
static hke_geom_t *map = NULL;
static hke_geom_t *robot = NULL;
static const char *robot_name = NULL;
static bor_real_t aabb[12];
static int dim;
static int conf_dim;
static int use_rot = 0;
static BOR_VEC(conf, 6);


static hke_geom_t *buildTriMesh(const bor_vec3_t *pts,
                                   const int *ids, size_t ids_len);
static void loadMap(void);

#define CHECK_PARAM(name, type) \
    if (!borCfgHaveParam(cfg, name)){ \
        fprintf(stderr, "Error: Parameter `%s' is not present.\n", name); \
        ok = 0; \
    }else if (borCfgParamType(cfg, name) != (type)){ \
        fprintf(stderr, "Error: Invalid type of parameter `%s'.\n", name); \
        ok = 0; \
    }

int borCfgMapInit(const char *fn)
{
    char name[300];
    char **robots;
    size_t robots_len;
    size_t i, len;
    int ok = 1;
    hke_params_t cd_params;
    const bor_real_t *aabb2;

    borVecSetZero(6, conf);

    rnd = borRandMTNewAuto();
    if (!rnd){
        fprintf(stderr, "Can't initialize random generator.\n");
        return -1;
    }

    cfg = borCfgRead(fn);
    if (!cfg){
        fprintf(stderr, "Can't read cfg file `%s'\n", fn);
        return -1;
    }

    // check params
    CHECK_PARAM("name", BOR_CFG_PARAM_STR)
    CHECK_PARAM("dim", BOR_CFG_PARAM_INT)
    CHECK_PARAM("aabb", BOR_CFG_PARAM_FLT | BOR_CFG_PARAM_ARR)
    CHECK_PARAM("pts", BOR_CFG_PARAM_V3 | BOR_CFG_PARAM_ARR)
    CHECK_PARAM("ids", BOR_CFG_PARAM_INT | BOR_CFG_PARAM_ARR)
    CHECK_PARAM("robots", BOR_CFG_PARAM_STR | BOR_CFG_PARAM_ARR)
    if (!ok)
        return -1;

    // check robots
    borCfgParamStrArr(cfg, "robots", &robots, &robots_len);
    for (i = 0; i < robots_len; i++){
        snprintf(name, 300, "%s_pts", robots[i]);
        CHECK_PARAM(name, BOR_CFG_PARAM_V3 | BOR_CFG_PARAM_ARR)
        snprintf(name, 300, "%s_ids", robots[i]);
        CHECK_PARAM(name, BOR_CFG_PARAM_INT | BOR_CFG_PARAM_ARR)
        snprintf(name, 300, "%s_init", robots[i]);
        CHECK_PARAM(name, BOR_CFG_PARAM_V3)
        snprintf(name, 300, "%s_goal", robots[i]);
        CHECK_PARAM(name, BOR_CFG_PARAM_V3)
        snprintf(name, 300, "%s_h", robots[i]);
        CHECK_PARAM(name, BOR_CFG_PARAM_FLT)
    }
    if (!ok)
        return -1;

    hkeParamsInit(&cd_params);
    //cd_params.build_flags = HKE_TOP_DOWN
    //                            | HKE_FIT_NAIVE
    //                            | HKE_FIT_NAIVE_NUM_ROT(5);
    cd_params.use_sap = 0;
    cd_params.num_threads = 1;
    cd = hkeNew(&cd_params);

    loadMap();

    borCfgParamInt(cfg, "dim", &dim);
    conf_dim = dim;
    borCfgParamFltArr(cfg, "aabb", &aabb2, &len);
    for (i = 0; i < len; i++){
        aabb[i] = aabb2[i];
    }
    if (dim == 2){
        aabb[4] = -M_PI;
        aabb[5] = M_PI;
    }else{ // dim == 3
        for (i = 0; i < 3; i++){
            aabb[6 + 2 * i] = -M_PI;
            aabb[7 + 2 * i] =  M_PI;
        }
    }
    return 0;
}

void borCfgMapDestroy(void)
{
    if (cfg)
        borCfgDel(cfg);
    if (rnd)
        borRandMTDel(rnd);
    if (cd)
        hkeDel(cd);
}

int borCfgMapRobot(const char *name, bor_real_t *h,
                     bor_vec_t *init, bor_vec_t *goal)
{
    int ok = 1;
    char pname[300];
    char format[500];
    const bor_vec3_t *pts;
    const int *ids;
    size_t ids_len;
    bor_vec3_t rot;

    snprintf(pname, 300, "%s_pts", name);
    CHECK_PARAM(pname, BOR_CFG_PARAM_V3 | BOR_CFG_PARAM_ARR)
    snprintf(pname, 300, "%s_ids", name);
    CHECK_PARAM(pname, BOR_CFG_PARAM_INT | BOR_CFG_PARAM_ARR)
    snprintf(pname, 300, "%s_init", name);
    CHECK_PARAM(pname, BOR_CFG_PARAM_V3)
    snprintf(pname, 300, "%s_goal", name);
    CHECK_PARAM(pname, BOR_CFG_PARAM_V3)
    snprintf(pname, 300, "%s_h", name);
    CHECK_PARAM(pname, BOR_CFG_PARAM_FLT)
    if (!ok){
        fprintf(stderr, "Error: No `%s' robot defined.\n", name);
        return -1;
    }

    snprintf(format, 500, "%s_pts:v3[] %s_ids:i[] %s_ids:i# %s_h:f %s_init:v3 %s_goal:v3",
             name, name, name, name, name, name);
    borCfgScan(cfg, format, &pts, &ids, &ids_len, h, init, goal);
    robot = buildTriMesh(pts, ids, ids_len);
    robot_name = name;
    borVecSet(init, 3, BOR_ZERO);
    borVecSet(init, 4, BOR_ZERO);
    borVecSet(init, 5, BOR_ZERO);
    borVecSet(goal, 3, BOR_ZERO);
    borVecSet(goal, 4, BOR_ZERO);
    borVecSet(goal, 5, BOR_ZERO);

    if (use_rot && conf_dim == 6){
        snprintf(format, 500, "%s_init_rot", name);
        if (borCfgParamIsV3(cfg, format)){
            borCfgParamV3(cfg, format, &rot);
            borVecSet(init, 3, borVec3X(&rot));
            borVecSet(init, 4, borVec3Y(&rot));
            borVecSet(init, 5, borVec3Z(&rot));
        }

        snprintf(format, 500, "%s_goal_rot", name);
        if (borCfgParamIsV3(cfg, format)){
            borCfgParamV3(cfg, format, &rot);
            borVecSet(goal, 3, borVec3X(&rot));
            borVecSet(goal, 4, borVec3Y(&rot));
            borVecSet(goal, 5, borVec3Z(&rot));
        }
    }
    return 0;
}

void borCfgMapListRobots(FILE *out)
{
    char **robots;
    size_t robots_len, i;

    if (borCfgScan(cfg, "robots:s[] robots:s#", &robots, &robots_len) != 0){
        fprintf(stderr, "# No robots parameter in cfg file.\n");
        return;
    }

    for (i = 0; i < robots_len; i++){
        fprintf(out, "%s\n", robots[i]);
    }
}

int borCfgMapCollide(const bor_vec_t *conf)
{
    if (!robot)
        return 0;

    if (conf_dim == 2){
        hkeGeomSetTr3(cd, robot, borVecGet(conf, 0), borVecGet(conf, 1), BOR_ZERO);
    }else if (conf_dim == 3){
        hkeGeomSetTr3(cd, robot, borVecGet(conf, 0), borVecGet(conf, 1), BOR_ZERO);
        hkeGeomSetRotEuler(cd, robot, BOR_ZERO, BOR_ZERO, borVecGet(conf, 2));
    }else if (conf_dim == 6){
        hkeGeomSetTr3(cd, robot, borVecGet(conf, 0), borVecGet(conf, 1), borVecGet(conf, 2));
        hkeGeomSetRotEuler(cd, robot, borVecGet(conf, 3), borVecGet(conf, 4), borVecGet(conf, 5));
    }
    //hkeGeomSetRotEuler(cd, robot, BOR_ZERO, BOR_ZERO, borVecGet(conf, 2));
    return hkeGeomCollide(cd, map, robot);
}

const bor_vec_t *borCfgMapConf(void)
{
    int i;
    for (i = 0; i < conf_dim; i++){
        borVecSet(conf, i, borRandMT(rnd, aabb[2 * i], aabb[2 * i + 1]));
    }

    return conf;
}

const bor_real_t *borCfgMapAABB(void)
{
    return aabb;
}
int borCfgMapDim(void)
{
    return dim;
}

int borCfgMapConfDim(void)
{
    return conf_dim;
}

void borCfgMapUseRot(void)
{
    use_rot = 1;
    if (dim == 2){
        conf_dim = 3;
    }else{
        conf_dim = 6;
    }
}

void borCfgMapDumpSVT(FILE *out, const char *name)
{
    const bor_vec3_t *pts;
    size_t pts_len;
    const int *ids;
    size_t ids_len;
    size_t i;

    if (borCfgScan(cfg, "pts:v3[] pts:v3# ids:i[] ids:i#", &pts, &pts_len, &ids, &ids_len) != 0){
        fprintf(stderr, "Can't load map from cfg file.\n");
    }

    fprintf(out, "----\n");
    if (name){
        fprintf(out, "Name: %s\n", name);
    }else{
        fprintf(out, "Name: Map\n");
    }

    fprintf(out, "Points off: 1\n");
    fprintf(out, "Face color: 0.8 0.8 0.8\n");
    fprintf(out, "Points:\n");
    for (i = 0; i < pts_len; i++){
        borVec3Print(&pts[i], out);
        fprintf(out, "\n");
    }

    fprintf(out, "Faces:\n");
    for (i = 0; (i + 2) < ids_len; i += 3){
        fprintf(out, "%d %d %d\n", (int)ids[i], (int)ids[i + 1], (int)ids[i + 2]);
    }

    fprintf(out, "----\n");
}

void borCfgMapRobotDumpSVT(const bor_vec_t *conf, FILE *out, const char *name)
{
    char format[500];
    const bor_vec3_t *pts;
    size_t pts_len;
    const int *ids;
    size_t ids_len;
    size_t i;
    bor_vec3_t w;
    bor_mat3_t rot;
    bor_vec3_t tr;

    if (!robot)
        return;

    if (dim == 2){
        borVec3Set(&tr, borVecGet(conf, 0), borVecGet(conf, 1), BOR_ZERO);
        borMat3SetRot3D(&rot, BOR_ZERO, BOR_ZERO, borVecGet(conf, 2));
    }else{ //if (dim == 3)
        borVec3Set(&tr, borVecGet(conf, 0), borVecGet(conf, 1), borVecGet(conf, 2));
        borMat3SetRot3D(&rot, borVecGet(conf, 3), borVecGet(conf, 4), borVecGet(conf, 5));
    }

    snprintf(format, 500, "%s_pts:v3[] %s_pts:v3# %s_ids:i[] %s_ids:i#",
             robot_name, robot_name, robot_name, robot_name);
    borCfgScan(cfg, format, &pts, &pts_len, &ids, &ids_len);

    fprintf(out, "----\n");
    if (name){
        fprintf(out, "Name: %s\n", name);
    }else{
        fprintf(out, "Name: Robot\n");
    }

    fprintf(out, "Points off: 1\n");
    fprintf(out, "Face color: 0.1 0.8 0.1\n");
    fprintf(out, "Points:\n");
    for (i = 0; i < pts_len; i++){
        borMat3MulVec(&w, &rot, &pts[i]);
        borVec3Add(&w, &tr);
        borVec3Print(&w, out);
        fprintf(out, "\n");
    }

    /*
    fprintf(out, "Edges:\n");
    for (i = 0; (i + 2) < ids_len; i += 3){
        fprintf(out, "%d %d\n", (int)ids[i], (int)ids[i + 1]);
        fprintf(out, "%d %d\n", (int)ids[i + 1], (int)ids[i + 2]);
        fprintf(out, "%d %d\n", (int)ids[i], (int)ids[i + 2]);
    }
    */
    fprintf(out, "Faces:\n");
    for (i = 0; (i + 2) < ids_len; i += 3){
        fprintf(out, "%d %d %d\n", (int)ids[i], (int)ids[i + 1], (int)ids[i + 2]);
    }

    fprintf(out, "----\n");
}

static hke_geom_t *buildTriMesh(const bor_vec3_t *pts,
                                   const int *ids, size_t ids_len)
{
    hke_geom_t *g;
    size_t i;

    g = hkeGeomNew(cd);
    for (i = 0; (i + 2) < ids_len; i += 3){
        hkeGeomAddTri(cd, g, &pts[ids[i]], &pts[ids[i + 1]], &pts[ids[i + 2]]);
    }
    hkeGeomBuild(cd, g);

    return g;
}

static void loadMap(void)
{
    const bor_vec3_t *pts;
    const int *ids;
    size_t ids_len;

    if (borCfgScan(cfg, "pts:v3[] ids:i[] ids:i#", &pts, &ids, &ids_len) != 0){
        fprintf(stderr, "Can't load map from cfg file.\n");
    }
    map = buildTriMesh(pts, ids, ids_len);
}
