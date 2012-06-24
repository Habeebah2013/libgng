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

#include <svoboda/ga.h>
#include <boruvka/dbg.h>

int terminate(svo_ga_t *ga, void *data)
{
    static int counter = 0;

    ++counter;
    if (counter == 1000)
        return 1;

    DBG("counter: %d", counter);
    return 0;
}

int main(int argc, char *argv[])
{
    svo_ga_ops_t ops;
    svo_ga_params_t params;
    svo_ga_t *ga;

    svoGAOpsParamsInt(&ops, &params, 5, 11);
    ops.terminate = terminate;
    params.pm = 0.01;

    ga = svoGANew(&ops, &params);
    svoGARun(ga);
    svoGADel(ga);

    return 0;
}
