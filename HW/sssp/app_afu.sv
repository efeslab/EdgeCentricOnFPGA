`include "cci_mpf_if.vh"
`include "csr_mgr.vh"
`include "afu_json_info.vh"

module app_afu
(
    input logic clk,
    cci_mpf_if.to_fiu fiu,
    app_csrs.app csrs,
    input logic c0NotEmpty,
    input logic c1NotEmpty
);

    logic reset = 1'b1;
    always @(posedge clk)
    begin
        reset <= fiu.reset;
    end

    t_if_ccip_Rx mpf2af_sRx;
    t_if_ccip_Tx af2mpf_sTx;

    always_comb
    begin
		mpf2af_sRx.c0 = fiu.c0Rx;
        mpf2af_sRx.c1 = fiu.c1Rx;

        mpf2af_sRx.c0TxAlmFull = fiu.c0TxAlmFull;
        mpf2af_sRx.c1TxAlmFull = fiu.c1TxAlmFull;

        fiu.c0Tx = cci_mpf_cvtC0TxFromBase(af2mpf_sTx.c0);
        if (cci_mpf_c0TxIsReadReq(fiu.c0Tx))
        begin
            fiu.c0Tx.hdr.ext.addrIsVirtual = 1'b1;
            fiu.c0Tx.hdr.ext.mapVAtoPhysChannel = 1'b1;
            fiu.c0Tx.hdr.ext.checkLoadStoreOrder = 1'b1;
        end

        fiu.c1Tx = cci_mpf_cvtC1TxFromBase(af2mpf_sTx.c1);
        if (cci_mpf_c1TxIsWriteReq(fiu.c1Tx))
        begin
            fiu.c1Tx.hdr.ext.addrIsVirtual = 1'b1;
            fiu.c1Tx.hdr.ext.mapVAtoPhysChannel = 1'b1;
            fiu.c1Tx.hdr.ext.checkLoadStoreOrder = 1'b1;
            fiu.c1Tx.hdr.pwrite = t_cci_mpf_c1_PartialWriteHdr'(0);
        end

        fiu.c2Tx = af2mpf_sTx.c2;
    end

    sssp_app_top app_cci(
        .clk,
        .reset,
        .cp2af_sRx(mpf2af_sRx),
        .af2cp_sTx(af2mpf_sTx),
        .csrs,
        .c0NotEmpty,
        .c1NotEmpty
        );

endmodule

module sssp_app_top
(
    input logic clk,
    input logic reset,
    input t_if_ccip_Rx cp2af_sRx,
    output t_if_ccip_Tx af2cp_sTx,
    app_csrs.app csrs,
    input logic c0NotEmpty,
    input logic c1NotEmpty
);

    logic reset_r;
    assign reset_r = ~reset;

    t_if_ccip_Rx sRx;
    always_ff @(posedge clk)
    begin
        sRx <= cp2af_sRx;
    end

    t_if_ccip_Tx sTx, pre_sTx;
	always_ff @(posedge clk)
    begin
        af2cp_sTx.c0 <= sTx.c0;
    end
    assign af2cp_sTx.c2.mmioRdValid = 1'b0;

    /* sTx.c1 needs a buffer */
    logic fifo_c1tx_rdack, fifo_c1tx_dout_v, fifo_c1tx_full, fifo_c1tx_almFull;
    t_if_ccip_c1_Tx fifo_c1tx_dout;
    logic [7:0] fifo_c1tx_count;
	sync_C1Tx_fifo_copy #(
		.DATA_WIDTH($bits(t_if_ccip_c1_Tx)),
		.CTL_WIDTH(0),
		.DEPTH_BASE2($clog2(64)),
		.GRAM_MODE(3),
		.FULL_THRESH(64-8)
	)
	inst_fifo_c1tx(
		.Resetb(reset_r),
		.Clk(clk),
		.fifo_din(sTx.c1),
		.fifo_ctlin(),
		.fifo_wen(sTx.c1.valid),
		.fifo_rdack(fifo_c1tx_rdack),
		.T2_fifo_dout(fifo_c1tx_dout),
		.T0_fifo_ctlout(),
		.T0_fifo_dout_v(fifo_c1tx_dout_v),
		.T0_fifo_empty(),
		.T0_fifo_full(fifo_c1tx_full),
		.T0_fifo_count(fifo_c1tx_count),
		.T0_fifo_almFull(fifo_c1tx_almFull),
		.T0_fifo_underflow(),
		.T0_fifo_overflow()
		);

    logic fifo_c1tx_dout_v_q, fifo_c1tx_dout_v_qq;
    assign fifo_c1tx_rdack = fifo_c1tx_dout_v;
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            fifo_c1tx_dout_v_q <= 0;
            fifo_c1tx_dout_v_qq <= 0;
            af2cp_sTx.c1.valid <= 0;
        end
        else
        begin
            fifo_c1tx_dout_v_q <= fifo_c1tx_dout_v;
            fifo_c1tx_dout_v_qq <= fifo_c1tx_dout_v_q;

            if (fifo_c1tx_dout_v_qq)
                af2cp_sTx.c1 <= fifo_c1tx_dout;
            else
                af2cp_sTx.c1 <= t_if_ccip_c1_Tx'(0);
        end
    end
        
    logic [127:0] afu_id = `AFU_ACCEL_UUID;

    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(sRx.c0.hdr);


	localparam MMIO_CSR_STATUS_ADDR = 0;
    localparam MMIO_CSR_VERTEX_ADDR = 1;
    localparam MMIO_CSR_VERTEX_NCL = 2;
    localparam MMIO_CSR_EDGE_ADDR = 3;
    localparam MMIO_CSR_EDGE_NCL = 4;
    localparam MMIO_CSR_UPDATE_BIN_ADDR = 5;
    localparam MMIO_CSR_LEVEL = 6;
    localparam MMIO_CSR_CONTROL = 7;

    t_ccip_clAddr csr_status_addr;
    t_ccip_clAddr csr_vertex_addr;
    t_ccip_clAddr csr_edge_addr;
    t_ccip_clAddr csr_update_bin_addr;
    logic [31:0] csr_vertex_ncl;
    logic [31:0] csr_edge_ncl;
    logic [15:0] csr_level;
    logic csr_ctl_start; 

    always_comb
    begin
        csrs.afu_id = `AFU_ACCEL_UUID;

        for (int i = 0; i < NUM_APP_CSRS; i = i + 1)
        begin
            csrs.cpu_rd_csrs[i].data = 64'(0);
        end

        csrs.cpu_rd_csrs[MMIO_CSR_STATUS_ADDR].data = t_ccip_mmioData'(csr_status_addr);
        csrs.cpu_rd_csrs[MMIO_CSR_VERTEX_ADDR].data = t_ccip_mmioData'(csr_vertex_addr);
		csrs.cpu_rd_csrs[MMIO_CSR_VERTEX_NCL].data = t_ccip_mmioData'(csr_vertex_ncl);
        csrs.cpu_rd_csrs[MMIO_CSR_EDGE_ADDR].data = t_ccip_mmioData'(csr_edge_addr);
		csrs.cpu_rd_csrs[MMIO_CSR_EDGE_NCL].data = t_ccip_mmioData'(csr_edge_ncl);
        csrs.cpu_rd_csrs[MMIO_CSR_UPDATE_BIN_ADDR].data = t_ccip_mmioData'(csr_update_bin_addr);
        csrs.cpu_rd_csrs[MMIO_CSR_LEVEL].data = t_ccip_mmioData'(csr_level);
        csrs.cpu_rd_csrs[MMIO_CSR_CONTROL].data = t_ccip_mmioData'(0);

    end


	logic csr_ctl_start;
    always_ff @(posedge clk)
    begin
		if (reset)
		begin
			csr_status_addr <= t_ccip_clAddr'(0);
            csr_vertex_addr <= t_ccip_clAddr'(0);
            csr_edge_addr <= t_ccip_clAddr'(0);
            csr_vertex_ncl <= 32'h0;
            csr_edge_ncl <= 32'h0;
            csr_level <= 32'h0;
		end
        else
        begin

            if (csrs.cpu_wr_csrs[MMIO_CSR_STATUS_ADDR].en) begin
                csr_status_addr <= t_ccip_clAddr'(csrs.cpu_wr_csrs[MMIO_CSR_STATUS_ADDR].data);
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_VERTEX_ADDR].en) begin
                csr_vertex_addr <= t_ccip_clAddr'(csrs.cpu_wr_csrs[MMIO_CSR_VERTEX_ADDR].data);
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_VERTEX_NCL].en) begin
                csr_vertex_ncl <= csrs.cpu_wr_csrs[MMIO_CSR_VERTEX_NCL].data[31:0];
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_EDGE_ADDR].en) begin
                csr_edge_addr <= t_ccip_clAddr'(csrs.cpu_wr_csrs[MMIO_CSR_EDGE_ADDR].data);
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_EDGE_NCL].en) begin
                csr_edge_ncl <= csrs.cpu_wr_csrs[MMIO_CSR_EDGE_NCL].data[31:0];
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_UPDATE_BIN_ADDR].en) begin
                csr_edge_ncl <= csrs.cpu_wr_csrs[MMIO_CSR_UPDATE_BIN_ADDR].data[31:0];
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_LEVEL].en) begin
                csr_level <= csrs.cpu_wr_csrs[MMIO_CSR_LEVEL].data[15:0];
            end

            if (csrs.cpu_wr_csrs[MMIO_CSR_CONTROL].en)
			begin
                if (csrs.cpu_wr_csrs[MMIO_CSR_CONTROL].data[0] == 1'b1) begin
                    csr_ctl_start <= 1'b1;
                end
			end
            else
            begin
                csr_ctl_start <= 1'b0;
            end

        end
    end

    typedef enum {
        MAIN_FSM_IDLE,
        MAIN_FSM_READ_VERTEX,
        MAIN_FSM_PROCESS_EDGE,
        MAIN_FSM_WRITE_RESULT,
        MAIN_FSM_FINISH
    } main_fsm_state_t;

    t_ccip_clAddr dma_src_addr;
    logic [31:0] dma_src_ncl;
    logic dma_start;
    logic dma_drop;
    logic [511:0] dma_out;
    logic dma_out_valid;
    logic dma_done;

    dma_read_engine dma_read(
        .clk(clk),
        .reset(reset),
        .src_addr(dma_src_addr),
        .src_ncl(dma_src_ncl),
        .start(dma_start),
        .drop(dma_drop),
        .c0rx(sRx.c0),
        .c0TxAlmFull(sRx.c0TxAlmFull),
        .c0tx(sTx.c0),
        .out(dma_out),
        .out_valid(dma_out_valid),
        .done(dma_done)
        );

    logic [31:0] sssp_word_in_addr;
    logic [1:0] sssp_control;
    logic [15:0] sssp_current_level;
    logic sssp_done;
    logic [31:0] sssp_update_entry_count;
    logic [511:0] sssp_word_out;
    logic sssp_word_out_valid;
    logic sssp_reset;

    sssp sssp_inst(
        .clk(clk),
        .rst(reset),
        .last_input_in(sssp_last_input_in),
        .word_in(dma_out),
        .w_addr(sssp_word_in_addr),
        .word_in_valid(dma_out_valid),
        .control(sssp_control),
        .current_level(sssp_current_level),
        .done(sssp_done),
        .update_entry_count(sssp_update_entry_count),
        .word_out(sssp_word_out),
        .word_out_valid(sssp_word_out_valid)
        );

    logic write_result_done;
    logic responses_received;

    /* the main state machine */
    main_fsm_state_t state;
    always_ff @(posedge clk)
    begin
        if (reset) begin
            state <= MAIN_FSM_IDLE;
        end
        else begin
            case (state)
                MAIN_FSM_IDLE: begin
                     if (csr_ctl_start) begin
                         state <= MAIN_FSM_READ_VERTEX;
                     end
                end
                MAIN_FSM_READ_VERTEX: begin
                    if (dma_done) begin
                        state <= MAIN_FSM_PROCESS_EDGE;
                    end
                end
                MAIN_FSM_PROCESS_EDGE: begin
                    if (sssp_done) begin
                        state <= MAIN_FSM_WRITE_RESULT;
                    end
                end
                MAIN_FSM_WRITE_RESULT: begin
                    state <= MAIN_FSM_FINISH;
                end
                MAIN_FSM_FINISH: begin
                    if (responses_received) begin
                        state <= MAIN_FSM_IDLE;
                    end
                end
            endcase
        end
    end

    logic vertex_dma_started;
    logic edge_dma_started;
    logic [31:0] write_cls;
    logic [31:0] num_write_req;
    logic [31:0] num_write_rsp;

    always_comb
    begin
        sTx.c1.hdr.sop 1= 1'b1;
        sTx.c1.hdr.vc_sel = eVC_VA;
        sTx.c1.hdr.cl_len = eCL_LEN_1;
        sTx.c1.hdr.req_type = eREQ_WRLINE_I;
        sTx.c1.hdr.mdata = 0;
        sTx.c1.hdr.rsvd0 = 0;
        sTx.c1.hdr.rsvd1 = 0;
    end

    /* dma read, sssp, and write request */
    always_ff @(posedge clk)
    begin
        if (reset) begin
            sssp_reset <= 1;
            vertex_dma_started <= 0;
            edge_dma_started <= 0;
        end
        else begin
            case (state)
                sssp_reset <= 0;
                dma_drop <= (fifo_c1tx_count >= 2);

                MAIN_FSM_IDLE: begin
                    dma_start <= 0;
                    vertex_dma_started <= 0;
                    edge_dma_started <= 0;

                    sssp_reset <= 1;
                    sssp_control <= 0;
                    sssp_word_in_addr <= 0;
                    write_cls <= 0;

                    num_write_req <= 0;
                    num_write_rsp <= 0;
                end
                MAIN_FSM_READ_VERTEX: begin
                    /* start dma when entering this state */
                    if (vertex_dma_started) begin
                        dma_start <= 0;
                    end
                    else begin
                        dma_start <= 1;
                        vertex_dma_started <= 1;
                    end

                    /* configure dma address */
                    dma_src_addr <= csr_vertex_addr;
                    dma_src_ncl <= csr_vertex_ncl;

                    /* configure sssp */
                    sssp_control <= 2'b01;
                    sssp_word_in_addr <= (sssp_word_in_addr + dma_out_valid) << 3;
                end
                MAIN_FSM_PROCESS_EDGE: begin
                    /* send out requests */
                    sTx.c1.valid <= sssp_word_out_valid;
                    sTx.c1.hdr.address <= csr_update_bin_addr + write_cls;
                    sTx.c1.data <= sssp_word_out;
                    write_cls <= write_cls + word_out_valid;
                    num_write_req <= num_write_req + word_out_valid;

                    /* start dma when entering this state */
                    if (edge_dma_started) begin
                        dma_start <= 0;
                    end
                    else begin
                        dma_start <= 1;
                        edge_dma_started <= 1;
                    end

                    /* configure dma address */
                    dma_src_addr <= csr_edge_addr;
                    dma_src_ncl <= csr_edge_ncl;

                    /* configure sssp */
                    sssp_control <= 2'b10;
                    sssp_level <= csr_level;
                end
                MAIN_FSM_WRITE_RESULT: begin
                    num_write_req <= num_write_req + 1;
                    sTx.c1.valid <= 1;
                    sTx.c1.hdr.address <= csr_status_addr;
                    sTx.c1.data <= {256'h0, 32'h0, update_entry_count, 64'h1};
                end
                MAIN_FSM_FINISH: begin
                    /* do nothing here */
                end
            endcase
        end
    end

	/* handle write response */
	always_ff @(posedge clk)
	begin
		if (reset) begin
			num_write_rsp <= 32'h0;
            responses_received <= 0;
		end
		else if (sRx.c1.rspValid == 1'b1) begin
            if (sRx.c1.hdr.format == 1'b0)
			    num_write_rsp <= num_write_rsp + 1;
            else begin
                case (sRx.c1.hdr.cl_num)
                    eCL_LEN_1: num_write_rsp <= num_write_rsp + 1;
                    eCL_LEN_2: num_write_rsp <= num_write_rsp + 2;
                    eCL_LEN_4: num_write_rsp <= num_write_rsp + 4;
                endcase
            end

            responses_received <= (num_write_req == num_write_rsp);
		end
	end
endmodule
