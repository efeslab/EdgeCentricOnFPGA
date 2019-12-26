`include "cci_mpf_if.vh"
`include "csr_mgr.vh"
`include "afu_json_info.vh"
`include "dma.vh"

module app_afu
(
    input logic clk,
    cci_mpf_if.to_fiu fiu,
    app_csrs.app csrs,
    input logic c0NotEmpty,
    input logic c1NotEmpty
);

    localparam NUM_SLAVE_CSRS = 10;

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

    master_top app_cci(
        .clk,
        .reset,
        .cp2af_sRx(mpf2af_sRx),
        .af2cp_sTx(af2mpf_sTx),
        .csrs,
        .c0NotEmpty,
        .c1NotEmpty
        );

endmodule

module master_top
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
        if (reset) begin
            af2cp_sTx.c0.valid <= 0;
        end
        else begin
            af2cp_sTx.c0 <= sTx.c0;
        end
    end
    assign af2cp_sTx.c2.mmioRdValid = 1'b0;

    /* sTx.c1 needs a buffer */
    logic fifo_c1tx_rdack, fifo_c1tx_dout_v, fifo_c1tx_full, fifo_c1tx_almFull;
    t_if_ccip_c1_Tx fifo_c1tx_dout;
    logic [7:0] fifo_c1tx_count;
	sync_C1Tx_fifo_copy #(
		.DATA_WIDTH($bits(t_if_ccip_c1_Tx)),
		.CTL_WIDTH(0),
		.DEPTH_BASE2($clog2(256)),
		.GRAM_MODE(3),
		.FULL_THRESH(256-8)
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

    logic fifo_c1tx_rdack_q, fifo_c1tx_rdack_qq;
    assign fifo_c1tx_rdack = fifo_c1tx_dout_v & ~sRx.c1TxAlmFull;
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            fifo_c1tx_rdack_q <= 0;
            fifo_c1tx_rdack_qq <= 0;
            af2cp_sTx.c1.valid <= 0;
        end
        else
        begin
            fifo_c1tx_rdack_q <= fifo_c1tx_dout_v & ~sRx.c1TxAlmFull;
            fifo_c1tx_rdack_qq <= fifo_c1tx_rdack_q;

            if (fifo_c1tx_rdack_qq)
                af2cp_sTx.c1 <= fifo_c1tx_dout;
            else
                af2cp_sTx.c1 <= t_if_ccip_c1_Tx'(0);
        end
    end

    t_ccip_clAddr dma_src_addr;
    logic [31:0] dma_src_ncl;
    logic dma_start;
    logic [511:0] dma_out, dma_out_q;
    logic dma_out_valid, dma_out_valid_q;
    logic dma_done;
    logic dma_pause;

    dma_read_engine dma_read(
        .clk(clk),
        .reset(reset),
        .src_addr(dma_src_addr),
        .src_ncl(dma_src_ncl),
        .start(dma_start),
        .pause(dma_pause),
        .c0rx(sRx.c0),
        .c0TxAlmFull(sRx.c0TxAlmFull),
        .c0tx(sTx.c0),
        .out(dma_out),
        .out_valid(dma_out_valid),
        .done(dma_done)
        );

    typedef enum {
        MASTER_IDLE,
        MASTER_DESC,
        MASTER_CONFIG,
        MASTER_RUN,
        MASTER_FENCE,
        MASTER_RESULT,
        MASTER_FINISH
    } master_fsm_state_t;
    master_fsm_state_t state;

    logic [63:0] slave_cpu_wr_csr_data [NUM_APP_CSRS-2:0];
    logic slave_cpu_wr_csr_en [NUM_APP_CSRS-2:0];
    logic [63:0] slave_cpu_rd_csr_data [NUM_APP_CSRS-2:0];
    logic [511:0] slave_dout, slave_nout, slave_dma_in;
    logic slave_dout_valid, slave_dout_done, slave_nout_valid, slave_dma_in_valid, slave_dma_done;

    slave_top #(.NUM_CSRS(NUM_APP_CSRS-1))
    slave_top_inst(
        .clk(clk),
        .reset(reset),
        .cpu_wr_csr_data(slave_cpu_wr_csr_data),
        .cpu_wr_csr_en(slave_cpu_wr_csr_en),
        .cpu_rd_csr_data(slave_cpu_rd_csr_data),
        .afu_id(csrs.afu_id),
        .din(slave_dma_in),
        .din_valid(slave_dma_in_valid),
        .din_done(slave_dma_done),
        .dout(slave_dout),
        .dout_valid(slave_dout_valid),
        .dout_done(slave_dout_done),
        .nout(slave_nout),
        .nout_valid(slave_nout_valid)
        );

    localparam MMIO_CSR_DESC_CONF = 0;

    always_comb
    begin
        csrs.cpu_rd_csrs[MMIO_CSR_DESC_CONF].data = t_ccip_mmioData'(0);

        for (int i = 0; i < NUM_APP_CSRS-1; i++) begin
            csrs.cpu_rd_csrs[i+1].data = slave_cpu_rd_csr_data[i];
            slave_cpu_wr_csr_data[i] = csrs.cpu_wr_csrs[i+1].data;
            slave_cpu_wr_csr_en[i] = csrs.cpu_wr_csrs[i+1].en;
        end
    end

    logic csr_ctl_start, csr_ctl_start_q;
    t_ccip_clAddr csr_desc_addr;
    t_ccip_clAddr conf_rd_buf_addr;
    t_ccip_clAddr conf_wr_buf_addr;
    t_ccip_clAddr conf_no_buf_addr;
    logic [31:0] conf_rd_buf_len;

    always_ff @(posedge clk)
    begin
        if (reset) begin
            csr_ctl_start <= 1'b0;
            csr_ctl_start_q <= 1'b0;
            csr_desc_addr <= t_ccip_clAddr'(0);
        end
        else begin
            if (csrs.cpu_wr_csrs[0].en) begin
                csr_ctl_start <= 1'b1;
                csr_desc_addr <= t_ccip_clAddr'(csrs.cpu_wr_csrs[0].data);
            end
            else begin
                csr_ctl_start <= 1'b0;
            end

            csr_ctl_start_q <= csr_ctl_start;
        end
    end

    logic rsp_all;
    dma_desc_t desc;

    always_ff @(posedge clk)
    begin
        if (reset) begin
            state <= MASTER_IDLE;
        end
        else begin

            case (state)
                MASTER_IDLE: begin
                    if (csr_ctl_start_q) begin
                        state <= MASTER_DESC;
                    end
                end
                MASTER_DESC: begin
                    if (dma_done) begin
                        state <= MASTER_CONFIG;
                    end
                end
                MASTER_CONFIG: begin
                    state <= MASTER_RUN;
                end
                MASTER_RUN: begin
                    if (slave_dout_done) begin
                        state <= MASTER_FENCE;
                    end
                end
                MASTER_FENCE: begin
                    state <= MASTER_RESULT;
                end
                MASTER_RESULT: begin
                    state <= MASTER_FINISH;
                end
                MASTER_FINISH: begin
                    if (rsp_all) begin
                        state <= MASTER_IDLE;
                    end
                end
            endcase
        end
    end

    t_ccip_c1_ReqMemHdr default_c1_memhdr;
    t_ccip_c1_ReqFenceHdr default_c1_fencehdr;

    always_comb
    begin
        default_c1_memhdr.sop = 1'b1;
        default_c1_memhdr.vc_sel = eVC_VA;
        default_c1_memhdr.cl_len = eCL_LEN_1;
        default_c1_memhdr.req_type = eREQ_WRLINE_I;
        default_c1_memhdr.address = t_ccip_clAddr'(0);
        default_c1_memhdr.mdata = 0;
        default_c1_memhdr.rsvd0 = 0;
        default_c1_memhdr.rsvd1 = 0;
        default_c1_memhdr.rsvd2 = 0;

        default_c1_fencehdr.vc_sel = eVC_VA;
        default_c1_fencehdr.req_type = eREQ_WRFENCE;
        default_c1_fencehdr.mdata = 0;
        default_c1_fencehdr.rsvd0 = 0;
        default_c1_fencehdr.rsvd1 = 0;
        default_c1_fencehdr.rsvd2 = 0;
    end

    logic main_dma_started, desc_dma_started;
    logic [31:0] write_cls;
    logic [31:0] num_write_req, num_write_rsp;

    always_ff @(posedge clk)
    begin
        if (reset) begin
            dma_start <= 0;
            num_write_req <= 0;
            main_dma_started <= 0;
            desc_dma_started <= 0;
            write_cls <= 0;
            dma_pause <= 0;
            sTx.c1.valid <= 0;
        end
        else begin
            sTx.c1.valid <= 0;
            slave_dma_in_valid <= 0;
            slave_dma_done <= 0;

            if (fifo_c1tx_count >= 12) begin
                dma_pause <= 1;
            end
            else if (fifo_c1tx_count <= 4) begin
                dma_pause <= 0;
            end

            case (state)
                MASTER_IDLE: begin
                    dma_start <= 0;
                    num_write_req <= 0;
                    write_cls <= 0;

                    main_dma_started <= 0;
                    desc_dma_started <= 0;
                end
                MASTER_DESC: begin
                    if (desc_dma_started) begin
                        dma_start <= 0;
                    end
                    else begin
                        dma_start <= 1;
                        desc_dma_started <= 1;
                    end

                    dma_src_addr <= csr_desc_addr;
                    dma_src_ncl <= 1;

                    if (dma_out_valid) begin
                        desc = int512_to_dma_desc(dma_out);

                        conf_rd_buf_addr <= t_ccip_clAddr'(desc.in_cl_addr);
                        conf_rd_buf_len <= desc.in_ncl;
                        conf_wr_buf_addr <= t_ccip_clAddr'(desc.out_cl_addr);
                        conf_no_buf_addr <= t_ccip_clAddr'(desc.notify_cl_addr);
                    end
                end
                MASTER_CONFIG: begin
                    dma_src_addr <= conf_rd_buf_addr;
                    dma_src_ncl <= conf_rd_buf_len;
                end
                MASTER_RUN: begin
                    sTx.c1.valid <= slave_dout_valid;
                    sTx.c1.hdr <= default_c1_memhdr;
                    sTx.c1.hdr.address <= conf_wr_buf_addr + write_cls;
                    sTx.c1.data <= slave_dout;
                    write_cls <= write_cls + slave_dout_valid;
                    num_write_req <= num_write_req + slave_dout_valid;

                    if (main_dma_started) begin
                        dma_start <= 0;
                    end
                    else begin
                        dma_start <= 1;
                        main_dma_started <= 1;
                    end

                    slave_dma_in_valid <= dma_out_valid;
                    slave_dma_done <= dma_done;
                    slave_dma_in <= dma_out;
                end
                MASTER_FENCE: begin
                    if (write_cls != 0) begin
                        sTx.c1.valid <= 1;
                        sTx.c1.hdr <= default_c1_fencehdr;
                        num_write_req <= num_write_req + 1;
                    end
                end
                MASTER_RESULT: begin
                    sTx.c1.valid <= 1;
                    sTx.c1.hdr <= default_c1_memhdr;
                    sTx.c1.hdr.address <= conf_no_buf_addr;
                    sTx.c1.data <= slave_nout;

                    main_dma_started <= 0;
                    desc_dma_started <= 0;
                end
                MASTER_FINISH: begin
                    /* nothing to do */
                end
            endcase
        end
    end

	/* handle write response */
	always_ff @(posedge clk)
	begin
		if (reset) begin
			num_write_rsp <= 32'h0;
            rsp_all <= 0;
		end
        else begin
            if (state == MASTER_IDLE) begin
                num_write_rsp <= 0;
            end
            else if (sRx.c1.rspValid == 1'b1) begin
                num_write_rsp <= num_write_rsp + 1;
            end
            rsp_all <= (num_write_req == num_write_rsp);
		end
	end

endmodule
