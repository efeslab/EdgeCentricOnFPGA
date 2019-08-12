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
        MAIN_FSM_VERTEX,
        MAIN_FSM_WAIT_EDGE,
        MAIN_FSM_EDGE,
        MAIN_FSM_NOTIFY
    } main_fsm_state_t;
    main_fsm_state_t state;

    logic sssp_reset;
    logic sssp_last_input_in;
    logic sssp_done;
    logic [31:0] sssp_update_entry_count;
    logic [31:0] csr_vertex_idx;
    logic [31:0] sssp_word_in_addr;
    logic [15:0] csr_level;
    logic [1:0] csr_control;
    logic csr_ctl_start;

    logic [511:0] din_q;
    logic din_valid_q, din_done_q;

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

            if (csr_control == 2'b10) begin
                sssp_last_input_in <= din_done;
            end
            else begin
                sssp_last_input_in <= 0;
            end
        end
    end

    sssp sssp_inst(
        .clk(clk),
        .rst(sssp_reset | reset),
        .last_input_in(sssp_last_input_in & (csr_control == 2)),
        .word_in(din_q),
        .w_addr(sssp_word_in_addr),
        .word_in_valid(din_valid_q),
        .control(csr_control),
        .current_level(csr_level),
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

		cpu_rd_csr_data[MMIO_CSR_VERTEX_IDX] = t_ccip_mmioData'(csr_vertex_idx);
        cpu_rd_csr_data[MMIO_CSR_LEVEL] = t_ccip_mmioData'(csr_level);
        cpu_rd_csr_data[MMIO_CSR_CONTROL] = t_ccip_mmioData'(csr_control);
    end

    always_ff @(posedge clk)
    begin
        if (reset) begin
            csr_vertex_idx <= 32'h0;
            csr_level <= 16'h0;
            csr_control <= 2'h0;
        end
        else begin

            if (cpu_wr_csr_en[MMIO_CSR_VERTEX_IDX]) begin
                csr_vertex_idx <= cpu_wr_csr_data[MMIO_CSR_VERTEX_IDX][31:0];
            end

            if (cpu_wr_csr_en[MMIO_CSR_LEVEL]) begin
                csr_level <= cpu_wr_csr_data[MMIO_CSR_LEVEL][15:0];
            end

            if (cpu_wr_csr_en[MMIO_CSR_CONTROL]) begin
                csr_control <= cpu_wr_csr_data[MMIO_CSR_CONTROL][1:0];
                csr_ctl_start <= 1'b1;
            end
            else begin
                csr_ctl_start <= 1'b0;
            end
        end
    end


    always_ff @(posedge clk)
    begin
        if (reset) begin
            state <= MAIN_FSM_IDLE;
        end
        else begin
            case (state)
                MAIN_FSM_IDLE: begin
                    if (csr_ctl_start) begin
                        state <= MAIN_FSM_VERTEX;
                    end
                end
                MAIN_FSM_VERTEX: begin
                    if (din_done) begin
                        state <= MAIN_FSM_WAIT_EDGE;
                    end
                end
                MAIN_FSM_WAIT_EDGE: begin
                    if (csr_ctl_start && csr_control == 2'b10) begin
                        state <= MAIN_FSM_EDGE;
                    end
                    else if (csr_ctl_start && csr_control == 2'b01) begin
                        state <= MAIN_FSM_VERTEX;
                    end
                end
                MAIN_FSM_EDGE: begin
                    if (sssp_done) begin
                        state <= MAIN_FSM_NOTIFY;
                    end
                end
                MAIN_FSM_NOTIFY: begin
                    state <= MAIN_FSM_IDLE;
                end
            endcase
        end
    end

    logic vertex_notified;

    always_ff @(posedge clk)
    begin
        if (reset) begin
            sssp_reset <= 1;
            vertex_notified <= 0;
        end
        else begin
            sssp_reset <= 0;
            nout_valid <= 0;
            dout_done <= 0;
            
            case (state)
                MAIN_FSM_IDLE: begin
                    sssp_reset <= 1;
                    sssp_word_in_addr <= csr_vertex_idx;
                    vertex_notified <= 0;
                end
                MAIN_FSM_VERTEX: begin
                    sssp_word_in_addr <= (sssp_word_in_addr + (din_valid_q << 3));
                    vertex_notified <= 0;
                end
                MAIN_FSM_WAIT_EDGE: begin
                    if (~vertex_notified) begin
                        nout <= {448'h0, 64'h1};
                        nout_valid <= din_done;
                        dout_done <= 1;
                    end
                    else begin
                        nout_valid <= 0;
                    end
                end
                MAIN_FSM_EDGE: begin
                    /* do nothing here */
                end
                MAIN_FSM_NOTIFY: begin
                    nout <= {384'h0, 32'h0, sssp_update_entry_count, 64'h1};
                    nout_valid <= 1;
                    dout_done <= 1;
                end
            endcase
        end
    end

endmodule
