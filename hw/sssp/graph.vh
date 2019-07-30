`ifndef GRAPH_VH
`define GRAPH_VH

typedef struct packed {
    logic [31:0] weight;
    logic [15:0] level;
    logic winf;
} vertex_t;

function logic [63:0] vertex_to_int64(
    input vertex_t vertex
);
    logic [63:0] int64;
    int64[31:0] = vertex.weight;
    int64[47:32] = vertex.level;
    int64[48] = vertex.winf;
    int64[63:49] = 15'h0;
    return int64;
endfunction

function vertex_t int64_to_vertex(
    input logic [63:0] int64
);
    vertex_t vertex;
    vertex.weight = int64[31:0];
    vertex.level = int64[47:32];
    vertex.winf = int64[48];
    return vertex;
endfunction

typedef struct packed {
    logic [31:0] src;
    logic [31:0] dst;
    logic [31:0] weight;
    logic [31:0] rsvd;
} edge_t;

function logic [127:0] edge_to_int128(
    input edge_t e
);
    logic [127:0] int128;
    int128[31:0] = e.src;
    int128[63:32] = e.dst;
    int128[95:64] = e.weight;
    int128[127:96] = e.rsvd;
    return int128;
endfunction

function edge_t int128_to_edge(
    input logic [127:0] int128
);
    edge_t e;
    e.src = int128[31:0];
    e.dst = int128[63:32];
    e.weight = int128[95:64];
    e.rsvd = int128[127:96];
    return e;
endfunction

typedef struct packed {
    logic [31:0] vertex;
    logic [31:0] weight;
} update_t;

`endif
