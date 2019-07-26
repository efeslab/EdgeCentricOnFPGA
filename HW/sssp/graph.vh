`ifndef GRAPH_VH
`define GRAPH_VH

typedef struct packed {
    logic [31:0] weight;
    logic [15:0] level;
    logic winf;
    logic linf;
} vertex_t;

typedef struct packed {
    logic [31:0] src;
    logic [31:0] dst;
    logic [31:0] weight;
    logic [31:0] rsvd;
} edge_t;

typedef struct packed {
    logic [31:0] vertex;
    logic [31:0] weight;
} update_t;

`endif
