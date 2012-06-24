#include <boruvka/cfg.h>

int main(int argc, char *argv[])
{
    char format[1000];
    char *name;
    bor_real_t *aabb;
    bor_real_t scale;
    bor_vec2_t m, w;
    const bor_vec2_t *pts;
    size_t pts_len;
    const int *ids;
    size_t ids_len;
    bor_vec2_t init, goal;
    bor_real_t h;
    size_t j, i;
    bor_cfg_t *cfg;
    char **robots;
    size_t robots_len;

    cfg = borCfgRead(argv[1]);

    borCfgScan(cfg, "name:s aabb:f[] pts:v2[] pts:v2# ids:i[] ids:i#",
            &name, &aabb, &pts, &pts_len, &ids, &ids_len);
    borVec2Set(&m, (aabb[0] + aabb[1]) / 2., (aabb[2] + aabb[3]) / 2.);
    scale = BOR_MAX(aabb[1] - aabb[0], aabb[3] - aabb[2]);
    scale = scale / 2.;
    if (strcmp(argv[2], "pi") == 0){
        scale = M_PI / scale;
    }else{
        scale = atof(argv[2]) / scale;
    }


    printf("name = '%s'\n", name);
    printf("dim:i = 2\n");
    printf("aabb:f[6] = %f %f %f %f 0 0\n",
            scale * (aabb[0] - borVec2X(&m)),
            scale * (aabb[1] - borVec2X(&m)),
            scale * (aabb[2] - borVec2Y(&m)),
            scale * (aabb[3] - borVec2Y(&m)));
    printf("\n");
    printf("pts:v3[%d] =\n", (int)pts_len);
    for (i = 0; i < pts_len; i++){
        borVec2Sub2(&w, &pts[i], &m);
        borVec2Scale(&w, scale);
        printf("    %f %f 0.\n", borVec2X(&w), borVec2Y(&w));
    }
    printf("\n");
    printf("ids:i[%d] =\n", (int)ids_len);
    for (i = 0; i < ids_len; i += 3){
        printf("    %d %d %d\n", ids[i], ids[i + 1], ids[i + 2]);
    }

    borCfgParamStrArr(cfg, "robots", &robots, &robots_len);
    printf("\nrobots:s[%d] =\n", (int)robots_len);
    for (j = 0; j < robots_len; j++){
        printf("    '%s'\n", robots[j]);
    }
    for (j = 0; j < robots_len; j++){
        snprintf(format, 1000, "%s_pts:v2[] %s_pts:v2# %s_ids:i[] %s_ids:i# %s_init:v2 %s_goal:v2 %s_h:f",
                robots[j], robots[j],
                robots[j], robots[j],
                robots[j], robots[j],
                robots[j]);
        borCfgScan(cfg, format, &pts, &pts_len, &ids, &ids_len, &init, &goal, &h);

        printf("\n");
        printf("%s_pts:v3[%d] =\n", robots[j], (int)pts_len);
        for (i = 0; i < pts_len; i++){
            borVec2Copy(&w, &pts[i]);
            borVec2Scale(&w, scale);
            printf("    %f %f 0.\n", borVec2X(&w), borVec2Y(&w));
        }
        printf("%s_ids:i[%d] =\n", robots[j], (int)ids_len);
        for (i = 0; i < ids_len; i += 3){
            printf("    %d %d %d\n", ids[i], ids[i + 1], ids[i + 2]);
        }
        borVec2Sub2(&w, &init, &m);
        borVec2Scale(&w, scale);
        printf("%s_init:v3 = %f %f 0.\n", robots[j], borVec2X(&w), borVec2Y(&w));
        borVec2Sub2(&w, &goal, &m);
        borVec2Scale(&w, scale);
        printf("%s_goal:v3 = %f %f 0.\n", robots[j], borVec2X(&w), borVec2Y(&w));
        printf("%s_h = %f\n", robots[j], h * scale);
    }

    return 0;
}
