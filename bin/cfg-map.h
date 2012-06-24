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

#ifndef BOR_CFG_MAP_H_
#define BOR_CFG_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int borCfgMapInit(const char *fn);
void borCfgMapDestroy(void);
int borCfgMapRobot(const char *name, bor_real_t *h,
                   bor_vec_t *init, bor_vec_t *goal);
void borCfgMapListRobots(FILE *out);
int borCfgMapCollide(const bor_vec_t *conf);
const bor_vec_t *borCfgMapConf(void);

const bor_real_t *borCfgMapAABB(void);
int borCfgMapDim(void);
int borCfgMapConfDim(void);
void borCfgMapUseRot(void);

void borCfgMapDumpSVT(FILE *out, const char *name);
void borCfgMapRobotDumpSVT(const bor_vec_t *conf, FILE *out, const char *name);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BOR_CFG_MAP_H_ */
