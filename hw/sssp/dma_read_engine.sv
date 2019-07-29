`include "cci_mpf_if.vh"

/* In this module we assume read responses come back in,
 * sequence, which can be achieved with the help of MPF. */
module dma_read_engine
(
    input logic clk,
    input logic reset,

    input t_ccip_clAddr src_addr,
    input logic [31:0] src_ncl,
    input logic start,
    input logic drop,

    input t_if_ccip_c0_Rx c0rx,
    input logic c0TxAlmFull,
    output t_if_ccip_c0_Tx c0tx,
    
    output logic [511:0] out,
    output logic out_valid,
    output logic done
);

    typedef enum {
        STATE_IDLE,
        STATE_READ_RUN,
        STATE_READ_PAUSE,
        STATE_READ_DROP,
        STATE_READ_WAIT,
        STATE_FINISH
    } rd_state_t;
    rd_state_t state;

    logic [31:0] req_idx, rsp_idx;
    logic [7:0] drop_id;
    logic req_done, rsp_done;

    assign req_done = req_idx == src_ncl;
    assign rsp_done = rsp_idx == src_ncl;

    /* state machine */
    always_ff @(posedge clk)
    begin
        if (reset) begin
            state <= STATE_IDLE;
        end
        else begin
            case (state)
                STATE_IDLE: begin
                    if (start) begin
                        state <= STATE_READ_RUN;
                    end
                end
                STATE_READ_RUN: begin
                    if (drop) begin
                        state <= STATE_READ_DROP;
                    end
                    else if (c0TxAlmFull) begin
                        state <= STATE_READ_PAUSE;
                    end
                    else if (req_done) begin
                        state <= STATE_READ_WAIT;
                    end
                end
                STATE_READ_PAUSE: begin
                    if (drop) begin
                        state <= STATE_READ_DROP;
                    end
                    else if (!c0TxAlmFull) begin
                        state <= STATE_READ_RUN;
                    end
                end
                STATE_READ_DROP: begin
                    if (!drop && !c0TxAlmFull) begin
                        state <= STATE_READ_RUN;
                    end
                    else if (!drop) begin
                        state <= STATE_READ_PAUSE;
                    end
                end
                STATE_READ_WAIT: begin
                    if (rsp_done) begin
                        state <= STATE_FINISH;
                    end
                    else if (drop) begin
                        state <= STATE_READ_DROP;
                    end
                end
                STATE_FINISH: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

    /* requset */
    always_ff @(posedge clk)
    begin
        if (reset) begin
            req_idx <= 0;
        end
        else begin
            c0tx.valid <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    req_idx <= 0;
                end
                STATE_READ_RUN: begin
                    /* Check the valid signal here. After sending out
                     * the last request, req_idx becomes src_ncl. However,
                     * the FSM may keep in the run state for an additional
                     * cycle. */
                    c0tx <= t_if_ccip_c0_Tx'(0);
                    c0tx.valid <= (req_idx != src_ncl);
                    c0tx.hdr.vc_sel <= eVC_VA;
                    c0tx.hdr.cl_len <= eCL_LEN_1;
                    c0tx.hdr.req_type <= eREQ_RDLINE_S;
                    c0tx.hdr.address <= src_addr + req_idx;
                    c0tx.hdr.mdata <= t_ccip_mdata'(drop_id);

                    req_idx <= req_idx + (req_idx != src_ncl);

                    /* debug */
                    if (req_idx > src_ncl) begin
                        $display("fatal error. req_idx > src_ncl!\n");
                        $finish;
                    end
                end
                STATE_READ_PAUSE,
                STATE_READ_WAIT,
                STATE_FINISH: begin
                    /* do nothing here */
                end
                STATE_READ_DROP: begin
                    /* drop all requests that are on the fly */
                    req_idx <= rsp_idx;
                end
            endcase
        end
    end
                    
    logic drop_id_increased;
    /* response */
    always_ff @(posedge clk)
    begin
        if (reset) begin
            rsp_idx <= 0;
            drop_id_increased <= 0;
            drop_id <= 0;
        end
        else begin
            /* It's fine to always feed the data to 'out',
             * since we can control the valid signal. */
            out <= c0rx.data;
            out_valid <= 1'b0;
            done <= 0;
            drop_id_increased <= 0;

            case (state)
                STATE_IDLE: begin
                    rsp_idx <= 0;
                    drop_id <= 0;
                end
                STATE_READ_PAUSE,
                STATE_READ_WAIT,
                STATE_READ_RUN: begin
                    /* Here we need to check the drop id, when mdata and drop id
                     * are not the same, the response should be dropped */
                    if (c0rx.rspValid && c0rx.hdr.mdata[7:0] == drop_id) begin
                        out_valid <= 1'b1;
                        rsp_idx <= rsp_idx + 1;
                    end
                end
                STATE_READ_DROP: begin
                    /* If the drop id is already increased, we do not need to
                     * increase it agian. We only increase the id after receiving
                     * the first response after entering the drop state, because
                     * if no response coming back during the drop, we don't need
                     * to resend the read request. */
                    if (c0rx.rspValid & !drop_id_increased) begin
                        drop_id_increased <= 1;
                        drop_id <= drop_id + 1;
                    end
                end
                STATE_FINISH: begin
                    done <= 1;
                end
            endcase
        end
    end

endmodule
