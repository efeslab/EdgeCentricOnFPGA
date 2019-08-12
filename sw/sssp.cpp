#include <inttypes.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "vai_svc_wrapper.h"

#define MMIO_CSR_DESC_CONF 0
#define APP_MMIO_BASE 1
#define MMIO_CSR_VERTEX_IDX (0 + APP_MMIO_BASE)
#define MMIO_CSR_LEVEL (1 + APP_MMIO_BASE)
#define MMIO_CSR_CONTROL (2 + APP_MMIO_BASE)

static int debug = 0;

extern "C" {

typedef struct {
    uint32_t weight;
    uint16_t level;
    uint16_t winf:1;
    uint16_t rsvd:15;
} vertex_t;

#define VERTEX_PER_CL (CL(1)/sizeof(vertex_t))
#define VERTEX(w,l,wi) (vertex_t) {  \
    .weight = (w),                      \
    .level = (l),                       \
    .winf = (wi),                       \
    .rsvd = 0                           \
}
#define IS_ACTIVE(v,curr_lvl) \
    ((v)->level == (curr_lvl))

typedef struct {
    uint32_t src;
    uint32_t dst;
    uint32_t weight;
    uint32_t rsvd;
} edge_t;

#define EDGE_PER_CL (CL(1)/sizeof(edge_t))
#define EDGE(s,d,w) (edge_t) {  \
    .src = (s),                 \
    .dst = (d),                 \
    .weight = (w),              \
    .rsvd = 0                   \
}

typedef struct {
    uint32_t offset;
    uint32_t count;
} v2e_t;

typedef struct {
    uint32_t vertex;
    uint32_t weight;
} update_t;

#define UPDATE(v,w) (update_t) {   \
    .vertex = (v),      \
    .weight = (w)       \
}

#define UPDATE_PER_CL (CL(1)/sizeof(update_t))

typedef struct {
      uint32_t edge_start_offset;
      uint64_t edge_start_cl;
      uint32_t num_edges;
      uint32_t num_cls;

      update_t *update_bin;
      uint32_t num_updates;
      uint32_t num_active_vertices;
} interval_t;

#define VERTEX_PER_INTERVAL 256
#define VERTEX_TO_INTERVAL(x) ((x)/VERTEX_PER_INTERVAL)
#define INTERVAL_TO_VERTEX(x) ((x)*VERTEX_PER_INTERVAL)

typedef struct {
    int num_v;
    int num_e;
    vertex_t *vertices;
    edge_t *edges;
    v2e_t *v2e;

    int num_intervals;
    interval_t *intervals;

} graph_t;

typedef struct {
    uint64_t valid;
    uint32_t size;
    uint32_t rsvd;
    uint64_t padding[4];
} status_t;

typedef struct {
    uint64_t in_cl_addr;
    uint32_t in_ncl;
    uint32_t rsvd0;
    uint64_t out_cl_addr;
    uint64_t notify_cl_addr;
} dma_desc_t;

} /* C */

graph_t *graph_init(VAI_SVC_WRAPPER *fpga, int num_v, int num_e, char *filename)
{
    int i, j;
    FILE *fp;
    uint32_t s, d;
    int num_intervals;

    if ((fp = fopen(filename, "r")) == NULL) {
        fprintf(stderr, "Cannot open file. Check the name.\n");
        return NULL;
    }

    graph_t *g = (graph_t *) fpga->allocBuffer(sizeof(graph_t));
    g->num_v = num_v;
    g->num_e = num_e;
    g->vertices = (vertex_t *) fpga->allocBuffer(sizeof(vertex_t) * ((num_v-1)/8+1)*8);
    g->edges = (edge_t *) fpga->allocBuffer(sizeof(edge_t) * ((num_e-1)/8+1)*8);
    g->v2e = (v2e_t *) malloc(sizeof(v2e_t) * num_v);

    for (i = 0; i < num_v; i++) {
        g->vertices[i] = VERTEX(0, 0, 1);
    }

    memset(g->edges, 0x0, ((num_e-1)/8+1)*8*sizeof(edge_t));
    memset(g->v2e, 0x0, num_v*sizeof(v2e_t));

    if (fscanf(fp, "%u %u\n", &s, &d) != EOF) {
        if (s >= num_v || d >= num_v) {
            goto error;
        }

        g->edges[0] = EDGE(s, d, (uint32_t)rand()%64);
        g->v2e[s].offset = 0;
        g->v2e[s].count++;
    }
        
    for (i = 1; i < num_e; i++) {
        if (fscanf(fp, "%u %u\n", &s, &d) != EOF) {
            g->edges[i] = EDGE(s, d, (uint32_t)rand()%64);
            g->v2e[s].count++;

            if (s != g->edges[i-1].src) {
                g->v2e[s].offset = i;
                if (s > g->edges[i-1].src + 1) {
                    for (j = g->edges[i-1].src + 1; j < s; j++) {
                        g->v2e[j].offset = i;
                    }
                }
            }
        }
    }

    fclose(fp);

    num_intervals = (num_v - 1) / VERTEX_PER_INTERVAL + 1;
    g->intervals = (interval_t *)malloc(sizeof(interval_t)*num_intervals);
    g->num_intervals = num_intervals;

    for (i = 0; i < num_intervals; i++) {
        int off = i * VERTEX_PER_INTERVAL;

        int start_off, end_off, ne;
        uint64_t start_cl, end_cl, ncl;
        if (i != num_intervals - 1) {
            start_off = g->v2e[i * VERTEX_PER_INTERVAL].offset;
            end_off = g->v2e[(i + 1) * VERTEX_PER_INTERVAL].offset;
            ne = end_off - start_off;
        }
        else {
            start_off = g->v2e[i * VERTEX_PER_INTERVAL].offset;
            end_off = g->num_e;
            ne = end_off - start_off;
        }

        start_cl = ((uint64_t) &g->edges[start_off]) / CL(1);
        end_cl = ((uint64_t) &g->edges[end_off]) / CL(1);

        if ((uint64_t) &g->edges[end_off] % CL(1) != 0) {
            end_cl += 1;
        }

        ncl = end_cl - start_cl;

        g->intervals[i].edge_start_offset = start_off;
        g->intervals[i].edge_start_cl = start_cl;
        g->intervals[i].num_edges = ne;
        g->intervals[i].num_cls = ncl;

        printf("[%d]: edge_off: %x, edge_cl: %#lx, num_edges: %x, num_cls: %x\n",
                    i, start_off, start_cl, ne, ncl);

        g->intervals[i].update_bin =
            (update_t *) fpga->allocBuffer(sizeof(update_t) * ne);
        g->intervals[i].num_updates = 0;
        g->intervals[i].num_active_vertices = 0;
    }

    for (i = 0; i < num_e; i++) {
        edge_t *e = &g->edges[i];
        printf("[%d]: src=%#lx, dst=%#lx, w=%#lx\n", i, e->src, e->dst, e->weight);
    }

    return g;

error:
    free(g->vertices);
    free(g->edges);
    free(g->v2e);
    free(g);
    return NULL;
}

int sssp(VAI_SVC_WRAPPER *fpga, graph_t *g, int root)
{
    int have_update;
    int current_level;
    int i, j, k;

    status_t *status = (status_t *)fpga->allocBuffer(sizeof(status_t));
    dma_desc_t *dma_desc = (dma_desc_t *)fpga->allocBuffer(sizeof(dma_desc_t));

    if (root >= g->num_v) {
        return -EFAULT;
    }
    g->vertices[root].winf = 0;
    g->vertices[root].level = 1;
    g->intervals[VERTEX_TO_INTERVAL(root)].num_active_vertices = 1;
    have_update = 1;
    current_level = 1;

    while (have_update) {
        have_update = 0;
        int active_cnt = 0;

        if (debug) {
            printf("\n------------ level %d -------------\n", current_level);
        }

        /* scatter */
		for (i = 0; i < g->num_intervals; i++) {
            interval_t *curr = &g->intervals[i];
            if (curr->num_active_vertices == 0) {
                continue;
            }

            curr->num_active_vertices = 0;

            uint64_t vertex_start_cl = (uint64_t)&g->vertices[i*VERTEX_PER_INTERVAL]/CL(1);
            uint64_t vertex_ncl;

            if (i < g->num_intervals - 1 || g->num_v % VERTEX_PER_INTERVAL == 0) {
                vertex_ncl = VERTEX_PER_INTERVAL * sizeof(vertex_t) / CL(1);
            }
            else {
                vertex_ncl =
                    (g->num_v % VERTEX_PER_INTERVAL * sizeof(vertex_t) - 1) / CL(1) + 1;
            }

            /* vertex stage */
            fpga->mmioWrite64(MMIO_CSR_VERTEX_IDX, i*VERTEX_PER_INTERVAL);
            fpga->mmioWrite64(MMIO_CSR_LEVEL, current_level);
            fpga->mmioWrite64(MMIO_CSR_CONTROL, 1);
            dma_desc->in_cl_addr = vertex_start_cl;
            dma_desc->in_ncl = vertex_ncl;
            dma_desc->out_cl_addr = 0;
            dma_desc->notify_cl_addr = (uint64_t)status/CL(1);
            status->valid = 0;
            fpga->mmioWrite64(MMIO_CSR_DESC_CONF, (uint64_t)dma_desc/CL(1));
            while (status->valid == 0) {
                usleep(500000);
                printf("pooling...\n");
            }

            printf("vertex done\n");

            /* edge stage */
            fpga->mmioWrite64(MMIO_CSR_CONTROL, 2);
            dma_desc->in_cl_addr = curr->edge_start_cl;
            dma_desc->in_ncl = curr->num_cls;
            dma_desc->out_cl_addr = (uint64_t)curr->update_bin/CL(1);
            dma_desc->notify_cl_addr = (uint64_t)status/CL(1);
            status->valid = 0;
            fpga->mmioWrite64(MMIO_CSR_DESC_CONF, (uint64_t)dma_desc/CL(1));
            while (status->valid == 0) {
                usleep(500000);
                printf("pooling...\n");
            }

            curr->num_updates = status->size;

            for (j = 0; j < curr->num_updates; j++) {
                printf("update: vertex %d to %d\n",
                        curr->update_bin[j].vertex,
                        curr->update_bin[j].weight);
            }
        }



        /* gather */
        for (i = 0; i < g->num_intervals; i++) {
            interval_t *curr = &g->intervals[i];
            if (curr->num_updates == 0) {
                continue;
            }

            for (j = 0; j < curr->num_updates; j++) {
                update_t *update = &curr->update_bin[j];
                vertex_t *update_vertex = &g->vertices[update->vertex];
                int vertex_interval = VERTEX_TO_INTERVAL(update->vertex);

                /* do the update */
                if (update_vertex->winf == 1 /* the value of the vertex is inf */
                        || update_vertex->weight > update->weight) {
                    if (debug) {
                        printf("interval %d: update interval %d vertex %d, %d -> %d\n",
                                j, vertex_interval, update->vertex,
                                update_vertex->winf?-1:update_vertex->weight,
                                update->weight);
                    }
                    update_vertex->weight = update->weight;
                    update_vertex->level = current_level + 1;
                    update_vertex->winf = 0;
                    g->intervals[vertex_interval].num_active_vertices++;
                    active_cnt++;
                }
            }

            curr->num_updates = 0;
        }

        for (i = 0; i < g->num_intervals; i++) {
            if (g->intervals[i].num_active_vertices != 0) {
                have_update = 1;
                break;
            }
        }

        printf("level %d: update %d vertices\n", current_level, active_cnt);
        fflush(stdout);

        current_level++;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    VAI_SVC_WRAPPER fpga;
    int opt;
    int num_v = -1, num_e = -1;
    int root = -1;
    char *filename = NULL;

    while ((opt = getopt (argc, argv, ":v:e:r:f:d")) != -1) {
        switch (opt) {
            case 'v':
                num_v = atoi(optarg);
                break;
            case 'e':
                num_e = atoi(optarg);
                break;
            case 'f':
                filename = optarg;
                break;
            case 'r':
                root = atoi(optarg);
                break;
            case 'd':
                debug = 1;
                break;
            case '?':
                printf("Unknown option: %c\n", opt);
                return -EINVAL;
        }
    }

    if (num_v < 0 || num_e < 0 || root < 0) {
        printf("Missing arguments.\n");
        return -EINVAL;
    }

    graph_t *graph = graph_init(&fpga, num_v, num_e, filename);

    if (debug) {
        printf("read done\n");
    }

    if (graph == NULL) {
        return -ENOENT;
    }

    if (graph->num_v <= root) {
        return -EFAULT;
    }

    sssp(&fpga, graph, root);

    int i, cnt = 0;
    for (i = 0; i < graph->num_v; i++) {
        if (graph->vertices[i].winf == 0) {
            cnt++;
        }
    }
    printf("vertex %d connects to %d of %d vertices\n", root, cnt, graph->num_v);

    return 0;
}

