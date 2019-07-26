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
		.T0_fifo_count(),
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
        
    // =========================================================================
    //
    //   CSR (MMIO) handling.
    //
    // =========================================================================

    // The AFU ID is a unique ID for a given program.  Here we generated
    // one with the "uuidgen" program and stored it in the AFU's JSON file.
    // ASE and synthesis setup scripts automatically invoke afu_json_mgr
    // to extract the UUID into afu_json_info.vh.
    logic [127:0] afu_id = `AFU_ACCEL_UUID;

    //
    // A valid AFU must implement a device feature list, starting at MMIO
    // address 0.  Every entry in the feature list begins with 5 64-bit
    // words: a device feature header, two AFU UUID words and two reserved
    // words.
    //

    // Is a CSR read request active this cycle?
    logic is_csr_read;
    assign is_csr_read = sRx.c0.mmioRdValid;

    // Is a CSR write request active this cycle?
    logic is_csr_write;
    assign is_csr_write = sRx.c0.mmioWrValid;

    // The MMIO request header is overlayed on the normal c0 memory read
    // response data structure.  Cast the c0Rx header to an MMIO request
    // header.
    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(sRx.c0.hdr);


    //
    // Implement the device feature list by responding to MMIO reads.
    //
	//RW
	localparam MMIO_CSR_STATUS_ADDR = 0;
    localparam MMIO_CSR_VERTEX_ADDR = 1;
    localparam MMIO_CSR_VERTEX_NCL = 2;
    localparam MMIO_CSR_EDGE_ADDR = 3;
    localparam MMIO_CSR_EDGE_NCL = 4;
    localparam MMIO_CSR_LEVEL = 5;
    localparam MMIO_CSR_CONTROL = 6;

    t_ccip_clAddr csr_status_addr;
    t_ccip_clAddr csr_vertex_addr;
    t_ccip_clAddr csr_edge_addr;
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
        csrs.cpu_rd_csrs[MMIO_CSR_LEVEL].data = t_ccip_mmioData'(csr_level);
        csrs.cpu_rd_csrs[MMIO_CSR_CONTROL].data = t_ccip_mmioData'(0);

    end

    //
    // CSR write handling.  Host software must tell the AFU the memory address
    // to which it should be writing.  The address is set by writing a CSR.
    //


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
        MAIN_FSM_PROCESS_EDGE
    } main_fsm_state_t;


	/*
	 * handle memory write response
	 */
	always_ff @(posedge clk)
	begin
		if (reset)
		begin
			write_resp_cnt <= 32'h0;
		end
		else if (state == STATE_RUN && sRx.c1.rspValid == 1'b1)
        begin
            if (sRx.c1.hdr.format == 1'b0)
			    write_resp_cnt <= write_resp_cnt + 1;
            else
            begin
                case (sRx.c1.hdr.cl_num)
                    eCL_LEN_1: write_resp_cnt <= write_resp_cnt + 1;
                    eCL_LEN_2: write_resp_cnt <= write_resp_cnt + 2;
                    eCL_LEN_4: write_resp_cnt <= write_resp_cnt + 4;
                endcase
            end
		end
	end
endmodule
