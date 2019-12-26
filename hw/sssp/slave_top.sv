`include "cci_mpf_if.vh"
`include "csr_mgr.vh"
`include "afu_json_info.vh"

module slave_top #(
    parameter NUM_CSRS = 10)
(
    input logic clk,
    input logic reset,

    input logic [63:0] cpu_wr_csr_data [NUM_CSRS-1:0],
    input logic cpu_wr_csr_en [NUM_CSRS-1:0],
    output logic [63:0] cpu_rd_csr_data [NUM_CSRS-1:0],
    output logic [127:0] afu_id,

    input logic [511:0] din,
    input logic din_valid,
    input logic din_done,

    output logic [511:0] dout,
    output logic dout_valid,
    output logic dout_done,

    output logic [511:0] nout,
    output logic nout_valid
);

    typedef enum {
        MAIN_FSM_IDLE,
        MAIN_FSM_META,
        MAIN_FSM_VERTEX,
        MAIN_FSM_EDGE,
        MAIN_FSM_WAIT_SSSP,
        MAIN_FSM_NOTIFY
    } main_fsm_state_t;
    main_fsm_state_t state;

    logic sssp_reset;
    logic sssp_last_input_in;
    logic sssp_done;
    logic [31:0] sssp_update_entry_count;
    logic [31:0] sssp_vertex_idx;
    logic [31:0] sssp_word_in_addr;
    logic [15:0] sssp_level;
    logic [1:0] sssp_control;
    logic csr_ctl_start;

    logic [511:0] din_q;
    logic din_valid_q, din_done_q;
    meta_t meta_q;

    assign meta_q = cl_to_meta(din_q);

    always_ff @(posedge clk)
    begin
        if (reset) begin
            din_valid_q <= 0;
            din_done_q <= 0;
            sssp_last_input_in <= 0;
        end
        else begin
            din_valid_q <= din_valid;
            din_done_q <= din_done;
            din_q <= din;

            sssp_last_input_in <= din_done;
        end
    end

    sssp sssp_inst(
        .clk(clk),
        .rst(sssp_reset | reset),
        .last_input_in(sssp_last_input_in),
        .word_in(din_q),
        .w_addr(sssp_word_in_addr),
        .word_in_valid(din_valid_q),
        .control(sssp_control),
        .current_level(sssp_level),
        .done(sssp_done),
        .update_entry_count(sssp_update_entry_count),
        .word_out(dout),
        .word_out_valid(dout_valid)
        );

    localparam MMIO_CSR_VERTEX_IDX = 0;
    localparam MMIO_CSR_LEVEL = 1;
    localparam MMIO_CSR_CONTROL = 2;

    always_comb
    begin
        afu_id = `AFU_ACCEL_UUID;

        for (int i = 0; i < NUM_APP_CSRS; i = i + 1)
        begin
            cpu_rd_csr_data[i] = 64'(0);
        end

		cpu_rd_csr_data[MMIO_CSR_VERTEX_IDX] = t_ccip_mmioData'(sssp_vertex_idx);
        cpu_rd_csr_data[MMIO_CSR_LEVEL] = t_ccip_mmioData'(sssp_level);
        cpu_rd_csr_data[MMIO_CSR_CONTROL] = t_ccip_mmioData'(sssp_control);
    end

    logic [31:0] vertex_count, vertex_total;
    logic [31:0] edge_count, edge_total;

    always_ff @(posedge clk)
    begin
        if (reset) begin
            sssp_reset <= 1;
            vertex_count <= 0;
            vertex_total <= 0;
            edge_count <= 0;
            edge_total <= 0;
            sssp_level <= 0;
            sssp_vertex_idx <= 0;
        end
        else begin
            sssp_reset <= 0;
            nout_valid <= 0;
            dout_done <= 0;
            case (state)
                MAIN_FSM_IDLE: begin
                    sssp_reset <= 1;
                    vertex_count <= 0;
                    vertex_total <= 0;
                    edge_count <= 0;
                    edge_total <= 0;
                    sssp_level <= 0;
                    sssp_vertex_idx <= 0;
                end
                MAIN_FSM_META: begin
                    vertex_total <= meta_q.vertex_ncl;
                    vertex_count <= 0;
                    edge_total <= meta_q.edge_ncl;
                    edge_count <= 0;
                    sssp_level <= meta_q.level;
                    sssp_vertex_idx <= meta_q.vertex_idx;

                    sssp_word_in_addr <= meta_q.vertex_idx;

                    $display("meta: %4x", din_q[15:0]);
                    $display("meta: %4x", din_q[31:16]);
                    $display("meta: %4x", din_q[47:32]);
                    $display("meta: %4x", din_q[63:48]);
                    $display("meta: %4x", din_q[64+15:64]);
                    $display("meta: %4x", din_q[64+31:64+16]);
                    $display("meta: %4x", din_q[64+47:64+32]);
                    $display("meta: %4x", din_q[64+63:64+48]);
                end
                MAIN_FSM_VERTEX: begin
                    vertex_count <= vertex_count + din_valid_q;
                    sssp_word_in_addr <= (sssp_word_in_addr + (din_valid_q << 3));
                end
                MAIN_FSM_EDGE: begin
                    edge_count <= edge_count + din_valid_q;
                end
                MAIN_FSM_WAIT_SSSP: begin
                    /* do nothing */
                end
                MAIN_FSM_NOTIFY: begin
                    nout <= {384'h0, 32'h0, sssp_update_entry_count, 64'h1};
                    nout_valid <= 1;
                    dout_done <= 1;
                end
            endcase
        end
    end
                
    always_ff @(posedge clk)
    begin
        if (reset) begin
            state <= MAIN_FSM_IDLE;
            sssp_control <= 0;
        end
        else begin
            case (state)
                MAIN_FSM_IDLE: begin
                    sssp_control <= 0;
                    if (din_valid) begin
                        state <= MAIN_FSM_META;
                    end
                end
                MAIN_FSM_META: begin
                    if (din_done_q) begin
                        state <= MAIN_FSM_WAIT_SSSP;
                    end
                    else if (din_valid_q) begin
                        state <= MAIN_FSM_VERTEX;
                        sssp_control <= 1;
                    end
                end
                MAIN_FSM_VERTEX: begin
                    if (vertex_count + din_valid_q == vertex_total) begin
                        state <= MAIN_FSM_EDGE;
                        sssp_control <= 2;
                    end
                end
                MAIN_FSM_EDGE: begin
                    if (edge_count + din_valid_q == edge_total) begin
                        state <= MAIN_FSM_META;
                        sssp_control <= 0;
                    end
                end
                MAIN_FSM_WAIT_SSSP: begin
                    if (sssp_done)
                        state <= MAIN_FSM_NOTIFY;
                end
                MAIN_FSM_NOTIFY: begin
                    state <= MAIN_FSM_IDLE;
                end
            endcase
        end
    end

endmodule
                    




