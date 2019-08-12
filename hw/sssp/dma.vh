`ifndef DMA_VH
`define DMA_VH


typedef struct packed {
    logic [63:0] in_cl_addr;
    logic [31:0] in_ncl;
    logic [63:0] out_cl_addr;
    logic [63:0] notify_cl_addr;
} dma_desc_t;

function dma_desc_t int512_to_dma_desc(
    input [511:0] int512
);
    dma_desc_t desc;
    desc.in_cl_addr = int512[63:0];
    desc.in_ncl = int512[95:64];
    desc.out_cl_addr = int512[191:128];
    desc.notify_cl_addr = int512[255:192];
    return desc;
endfunction

`endif
