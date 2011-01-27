#include <fermat/mesh/mesh3.h>
#include <fermat/alloc.h>

fer_mesh3_vertex_t *ferMesh3VertexNew(fer_real_t x, fer_real_t y, fer_real_t z)
{
    fer_mesh3_vertex_t *v;

#ifdef FER_SSE
    // Vec3 must be properly aligned if we are using SSE.
    // Because vec3 occupies beggining of struct its enough if we align
    // whole struct
    v = FER_ALLOC_ALIGN(fer_mesh3_vertex_t, 16);
#else /* FER_SSE */
    v = FER_ALLOC(fer_mesh3_vertex_t);
#endif /* FER_SSE */

    ferVec3Set(&v->v, x, y, z);

    return v;
}

fer_mesh3_vertex_t *ferMesh3VertexNew2(const fer_vec3_t *coords)
{
    return ferMesh3VertexNew(ferVec3X(coords),
                             ferVec3Y(coords),
                             ferVec3Z(coords));
}

void ferMesh3VertexDel(fer_mesh3_vertex_t *v)
{
    free(v);
}


fer_mesh3_edge_t *ferMesh3EdgeNew(void)
{
    fer_mesh3_edge_t *e;
    e = FER_ALLOC(fer_mesh3_edge_t);
    return e;
}

void ferMesh3EdgeDel(fer_mesh3_edge_t *e)
{
    free(e);
}

int ferMesh3EdgeTriCheck(const fer_mesh3_edge_t *e1,
                         const fer_mesh3_edge_t *e2,
                         const fer_mesh3_edge_t *e3)
{
    // TODO
    return 0;
}




fer_mesh3_face_t *ferMesh3FaceNew(void)
{
    fer_mesh3_face_t *f;
    f = FER_ALLOC(fer_mesh3_face_t);
    return f;
}

void ferMesh3FaceDel(fer_mesh3_face_t *f)
{
    free(f);
}



fer_mesh3_t *ferMesh3New(void)
{
    fer_mesh3_t *m;
    m = FER_ALLOC(fer_mesh3_t);

    ferListInit(&m->verts);
    m->verts_len = 0;
    ferListInit(&m->edges);
    m->edges_len = 0;
    ferListInit(&m->faces);
    m->faces_len = 0;

    return m;
}

void ferMesh3Del(fer_mesh3_t *m)
{
    ferMesh3Del2(m, NULL, NULL, NULL, NULL, NULL, NULL);
}

void ferMesh3Del2(fer_mesh3_t *m,
                  void (*delvertex)(fer_mesh3_vertex_t *, void *), void *vdata,
                  void (*deledge)(fer_mesh3_edge_t *, void *), void *edata,
                  void (*delface)(fer_mesh3_face_t *, void *), void *fdata)
{
    fer_mesh3_vertex_t *v;
    fer_mesh3_edge_t *e;
    fer_mesh3_face_t *f;
    fer_list_t *item;

    // disconnect all faces
    while (!ferListEmpty(&m->faces)){
        item = ferListNext(&m->faces);
        f = ferListEntry(item, fer_mesh3_face_t, list);
        ferMesh3RemoveFace(m, f);

        if (delface){
            delface(f, fdata);
        }
    }

    // disconnect all edges
    while (!ferListEmpty(&m->edges)){
        item = ferListNext(&m->edges);
        e = ferListEntry(item, fer_mesh3_edge_t, list);
        ferMesh3RemoveEdge(m, e);

        if (deledge){
            deledge(e, edata);
        }
    }

    // disconnect all vertices
    while (!ferListEmpty(&m->verts)){
        item = ferListNext(&m->verts);
        v = ferListEntry(item, fer_mesh3_vertex_t, list);
        ferMesh3RemoveVertex(m, v);

        if (delvertex){
            delvertex(v, edata);
        }
    }

    free(m);
}

void ferMesh3AddVertex(fer_mesh3_t *m, fer_mesh3_vertex_t *v)
{
    ferListAppend(&m->verts, &v->list);
    m->verts_len++;

    ferListInit(&v->edges);
    v->edges_len = 0;
}

int ferMesh3RemoveVertex(fer_mesh3_t *m, fer_mesh3_vertex_t *v)
{
    if (!ferListEmpty(&v->edges))
        return -1;

    ferListDel(&v->list);
    m->verts_len--;
    return 0;
}

void ferMesh3AddEdge(fer_mesh3_t *m, fer_mesh3_edge_t *e,
                     fer_mesh3_vertex_t *start, fer_mesh3_vertex_t *end)
{
    // assign start and end point
    e->v[0] = start;
    e->v[1] = end;

    // append edge to list of edges in vertices
    ferListAppend(&start->edges, &e->vlist[0]);
    start->edges_len++;
    ferListAppend(&end->edges, &e->vlist[1]);
    end->edges_len++;

    // add edge to list of all edges
    ferListAppend(&m->edges, &e->list);
    m->edges_len++;

    // initialize incidenting faces
    e->f[0] = e->f[1] = NULL;
}

int ferMesh3RemoveEdge(fer_mesh3_t *m, fer_mesh3_edge_t *e)
{
    if (e->f[0] != NULL || e->f[1] != NULL)
        return -1;

    // remove edge from lists in vertices
    ferListDel(&e->vlist[0]);
    e->v[0]->edges_len--;
    ferListDel(&e->vlist[1]);
    e->v[1]->edges_len--;

    // remove edge from list of all edges
    ferListDel(&e->list);
    m->edges_len--;

    // reset .v[]
    e->v[0] = e->v[1] = NULL;

    return 0;
}

int ferMesh3AddFace(fer_mesh3_t *m, fer_mesh3_face_t *f,
                    fer_mesh3_edge_t *e1, fer_mesh3_edge_t *e2,
                    fer_mesh3_edge_t *e3)
{
    if (ferMesh3EdgeFacesLen(e1) == 2
            || ferMesh3EdgeFacesLen(e2) == 2
            || ferMesh3EdgeFacesLen(e3) == 2)
        return -1;

    // assign edges and also back pointers
    f->e[0] = e1;
    if (!e1->f[0]){
        e1->f[0] = f;
    }else{
        e1->f[1] = f;
    }

    f->e[1] = e2;
    if (!e2->f[0]){
        e2->f[0] = f;
    }else{
        e2->f[1] = f;
    }

    f->e[2] = e3;
    if (!e3->f[0]){
        e3->f[0] = f;
    }else{
        e3->f[1] = f;
    }

    // add face to list of all faces
    ferListAppend(&m->faces, &f->list);
    m->faces_len++;

    return 0;
}

void ferMesh3RemoveFace(fer_mesh3_t *m, fer_mesh3_face_t *f)
{
    size_t i;

    // disconnect face from edges
    for (i = 0; i < 3; i++){
        if (f->e[i]->f[0] == f)
            f->e[i]->f[0] = f->e[i]->f[1];
        f->e[i]->f[1] = NULL;
    }

    // disconnect face from list of all faces
    ferListDel(&f->list);
    m->faces_len--;

    // zeroize pointers to edges
    f->e[0] = f->e[1] = f->e[2] = NULL;
}
