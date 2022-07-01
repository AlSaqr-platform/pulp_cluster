// Copyright 2020 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Authors:
// - Andreas Kurth <akurth@iis.ee.ethz.ch>

`include "common_cells/registers.svh"

module tlb_miss_queue #(
  parameter int unsigned NUM_CORES = 0,
  parameter int unsigned NUM_ENTRIES = 0
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        test_mode_i,
  XBAR_TCDM_BUS.Slave ctrl_slave[NUM_CORES-1:0]
);

  typedef logic [cf_math_pkg::idx_width(NUM_CORES)-1:0] core_id_t;
  typedef logic [51:0] pfn_t;
  typedef enum logic {OK, ERR} resp_e;

  typedef struct packed {
    pfn_t     pfn;
    core_id_t id;
  } miss_t;

  // FIFO storing the TLB misses
  miss_t  fifo_inp_data,
          fifo_oup_data;
  logic   fifo_inp_valid, fifo_inp_ready,
          fifo_oup_valid, fifo_oup_ready;
  stream_fifo #(
    .FALL_THROUGH (1'b0),
    .DATA_WIDTH   ($bits(miss_t)),
    .DEPTH        (NUM_ENTRIES),
    .T            (miss_t)
  ) i_miss_fifo (
    .clk_i,
    .rst_ni,
    .flush_i    (1'b0),
    .testmode_i (test_mode_i),
    .usage_o    (/* unused */),
    .data_i     (fifo_inp_data),
    .valid_i    (fifo_inp_valid),
    .ready_o    (fifo_inp_ready),
    .data_o     (fifo_oup_data),
    .valid_o    (fifo_oup_valid),
    .ready_i    (fifo_oup_ready)
  );

  // Interface from cores
  core_id_t [NUM_CORES-1:0]             core_miss_handled_id;
  miss_t [NUM_CORES-1:0]                core_miss;
  logic [NUM_CORES-1:0]                 core_miss_valid,            core_miss_ready,
                                        core_miss_handled_inp_req,  core_miss_handled_inp_gnt,
                                        core_miss_handled_oup_req,  core_miss_handled_oup_gnt,
                                        core_miss_handler_req,      core_miss_handler_gnt,
                                        core_miss_handler_valid,    core_miss_handler_ready;
  for (genvar i = 0; i < NUM_CORES; i++) begin : gen_core_if
    tlb_miss_queue_core_if #(
      .core_id_t  (core_id_t),
      .miss_t     (miss_t),
      .pfn_t      (pfn_t)
    ) i_core_if (
      .clk_i,
      .rst_ni,
      .test_mode_i,
      .core_id_i              (core_id_t'(i)),
      .ctrl_slave             (ctrl_slave[i]),
      .miss_o                 (core_miss[i]),
      .miss_valid_o           (core_miss_valid[i]),
      .miss_ready_i           (core_miss_ready[i]),
      .miss_handler_miss_i    (fifo_oup_data),
      .miss_handler_req_o     (core_miss_handler_req[i]),
      .miss_handler_gnt_i     (core_miss_handler_gnt[i]),
      .miss_handler_valid_i   (core_miss_handler_valid[i]),
      .miss_handler_ready_o   (core_miss_handler_ready[i]),
      .miss_handled_inp_req_i (core_miss_handled_inp_req[i]),
      .miss_handled_inp_gnt_o (core_miss_handled_inp_gnt[i]),
      .miss_handled_oup_id_o  (core_miss_handled_id[i]),
      .miss_handled_oup_req_o (core_miss_handled_oup_req[i]),
      .miss_handled_oup_gnt_i (core_miss_handled_oup_gnt[i])
    );
  end

  // Crossbar between cores for signaling handled misses
  stream_xbar #(
    .NumInp       (NUM_CORES),
    .NumOut       (NUM_CORES),
    .DataWidth    (32'd1),
    .OutSpillReg  (1'b0),
    .ExtPrio      (1'b0),
    .AxiVldRdy    (1'b1),
    .LockIn       (1'b1)
  ) i_miss_handled_xbar (
    .clk_i,
    .rst_ni,
    .flush_i  (1'b0),
    .rr_i     ('0),
    .data_i   ('0),
    .sel_i    (core_miss_handled_id),
    .valid_i  (core_miss_handled_oup_req),
    .ready_o  (core_miss_handled_oup_gnt),
    .data_o   (/* unused */),
    .idx_o    (/* unused */),
    .valid_o  (core_miss_handled_inp_req),
    .ready_i  (core_miss_handled_inp_gnt)
  );

  // Arbiter for enqueuing into the TLB miss FIFO
  stream_arbiter #(
    .DATA_T   (miss_t),
    .N_INP    (NUM_CORES),
    .ARBITER  ("rr")
  ) i_miss_inp_arbiter (
    .clk_i,
    .rst_ni,
    .inp_data_i   (core_miss),
    .inp_valid_i  (core_miss_valid),
    .inp_ready_o  (core_miss_ready),
    .oup_data_o   (fifo_inp_data),
    .oup_valid_o  (fifo_inp_valid),
    .oup_ready_i  (fifo_inp_ready)
  );

  // Single-producer multiple-consumer (only one consumer at a time) dequeuing from the FIFO
  single_prod_multiple_cons #(
    .NumConsumers (NUM_CORES)
  ) i_miss_oup_spmc (
    .clk_i,
    .rst_ni,
    .prod_valid_i (fifo_oup_valid),
    .prod_ready_o (fifo_oup_ready),
    .cons_req_i   (core_miss_handler_req),
    .cons_gnt_o   (core_miss_handler_gnt),
    .cons_valid_o (core_miss_handler_valid),
    .cons_ready_i (core_miss_handler_ready)
  );
endmodule

module single_prod_multiple_cons #(
  parameter int unsigned NumConsumers = 32'd0,
  parameter bit FairArb = 1'b0
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    prod_valid_i,
  output logic                    prod_ready_o,
  input  logic [NumConsumers-1:0] cons_req_i,
  output logic [NumConsumers-1:0] cons_gnt_o,
  output logic [NumConsumers-1:0] cons_valid_o,
  input  logic [NumConsumers-1:0] cons_ready_i
);

  typedef logic [cf_math_pkg::idx_width(NumConsumers)-1:0] idx_t;

  logic [NumConsumers-1:0] cons_req;
  idx_t arb_idx,
        arb_idx_d,  arb_idx_q;
  logic arb_req,    arb_gnt;
  rr_arb_tree #(
    .NumIn      (NumConsumers),
    .DataWidth  (1'b1),
    .ExtPrio    (1'b0),
    .AxiVldRdy  (1'b1),
    .LockIn     (1'b1),
    .FairArb    (FairArb)
  ) i_arbiter (
    .clk_i,
    .rst_ni,
    .flush_i      (1'b0),
    .rr_i         ('0),
    .req_i        (cons_req),
    .gnt_o        (/* unused */),
    .data_i       ('0),
    .req_o        (arb_req),
    .gnt_i        (arb_gnt),
    .data_o       (/* unused */),
    .idx_o        (arb_idx)
  );

  typedef enum logic {Ready, AwaitConsReady} state_e;

  logic   prod_valid_d, prod_valid_q;
  state_e state_d,      state_q;

  always_comb begin
    arb_gnt = 1'b0;
    arb_idx_d = arb_idx_q;
    cons_req = cons_req_i;
    cons_gnt_o = '0;
    cons_valid_o = '0;
    prod_ready_o = 1'b0;
    prod_valid_d = prod_valid_q;
    state_d = state_q;

    case (state_q)
      Ready: begin
        if (arb_req) begin
          cons_gnt_o[arb_idx] = 1'b1;
          if (prod_valid_i) begin
            cons_valid_o[arb_idx] = 1'b1;
            if (cons_ready_i[arb_idx]) begin
              arb_gnt = 1'b1;
              prod_ready_o = 1'b1;
            end else begin
              arb_idx_d = arb_idx;
              prod_valid_d = 1'b1;
              state_d = AwaitConsReady;
            end
          end else begin
            arb_gnt = 1'b1;
          end
        end
      end

      AwaitConsReady: begin
        cons_req = '0;
        cons_req[arb_idx_q] = 1'b1;
        cons_gnt_o[arb_idx] = 1'b1;
        cons_valid_o[arb_idx] = prod_valid_q;
        if (cons_ready_i[arb_idx]) begin
          arb_gnt = 1'b1;
          prod_ready_o = prod_valid_q;
          state_d = Ready;
        end
      end

    endcase
  end

  `FFARN(arb_idx_q, arb_idx_d, '0, clk_i, rst_ni)
  `FFARN(prod_valid_q, prod_valid_d, '0, clk_i, rst_ni)
  `FFARN(state_q, state_d, Ready, clk_i, rst_ni)

endmodule

module tlb_miss_queue_core_if #(
  parameter type core_id_t = logic,
  parameter type miss_t = logic,
  parameter type pfn_t = logic
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        test_mode_i,
  input  core_id_t    core_id_i,
  XBAR_TCDM_BUS.Slave ctrl_slave,
  // add TLB miss in queue
  output miss_t       miss_o,
  output logic        miss_valid_o,
  input  logic        miss_ready_i,
  // get TLB miss from queue
  input  miss_t       miss_handler_miss_i,
  output logic        miss_handler_req_o,
  input  logic        miss_handler_gnt_i,
  input  logic        miss_handler_valid_i,
  output logic        miss_handler_ready_o,
  // miss of this core handled by another core
  input  logic        miss_handled_inp_req_i,
  output logic        miss_handled_inp_gnt_o,
  // this core handled the miss of another core
  output core_id_t    miss_handled_oup_id_o,
  output logic        miss_handled_oup_req_o,
  input  logic        miss_handled_oup_gnt_i
);

  typedef enum logic [1:0] {Idle, AwaitMissHandled, ReadMissUpper} state_e;

  logic [19:0]  pfn_lower_d,  pfn_lower_q;
  state_e       state_d,      state_q;

  assign ctrl_slave.r_opc = '0;
  assign miss_o.id = core_id_i;
  assign miss_o.pfn[19:0] = pfn_lower_q;
  assign miss_o.pfn[$bits(pfn_t)-1:20] = ctrl_slave.wdata[$bits(pfn_t)-21:0];

  // Multiplex response sources to TCDM response channel.
  localparam int unsigned NumRespSources = 4;
  typedef enum logic [cf_math_pkg::idx_width(NumRespSources)-1:0] {
    Okay, Error, PfnLower, PfnUpper
  } resp_idx_e;
  resp_idx_e                        resp_sel;
  logic                             resp_valid;
  logic [NumRespSources-1:0][31:0]  resp_data;
  logic [NumRespSources-1:0]        resp_gnt;
  tcdm_resp_mux #(
    .DataWidth  (32'd32),
    .NumInputs  (NumRespSources)
  ) i_resp_mux (
    .clk_i,
    .rst_ni,
    .sel_i        (resp_sel),
    .sel_valid_i  (resp_valid),
    .data_i       (resp_data),
    .gnt_o        (resp_gnt),
    .rdata_o      (ctrl_slave.r_rdata),
    .rvalid_o     (ctrl_slave.r_valid)
  );
  assign resp_data[Okay] = '0; // generic okay response
  assign resp_data[Error] = 32'd1; // generic error response
  assign resp_data[PfnLower][31:12] = miss_handler_miss_i.pfn[19:0]; // lower bits of PFN ..
  assign resp_data[PfnLower][11:0] = miss_handler_miss_i.id; // .. and core ID
  assign resp_data[PfnUpper] = miss_handler_miss_i.pfn[51:20]; // upper bits of PFN
  assign miss_handler_ready_o = resp_gnt[PfnUpper];

  // Handle requests and trigger responses.
  always_comb begin
    ctrl_slave.gnt = 1'b0;
    miss_handled_inp_gnt_o = 1'b0;
    miss_handled_oup_id_o = '0;
    miss_handled_oup_req_o = 1'b0;
    miss_handler_req_o = 1'b0;
    miss_valid_o = 1'b0;
    pfn_lower_d = pfn_lower_q;
    resp_sel = Error;
    resp_valid = 1'b0;
    state_d = state_q;

    unique case (state_q)
      Idle: begin
        if (ctrl_slave.req) begin
          // Handle new request.
          unique case ({ctrl_slave.add[7:0], ctrl_slave.wen})
            {8'h00, 1'b0}: begin // write lower 32 bit of VA
              pfn_lower_d = ctrl_slave.wdata[31:12];
              ctrl_slave.gnt = 1'b1;
              resp_sel = Okay;
              resp_valid = 1'b1;
            end
            {8'h04, 1'b0}: begin // write upper 32 bit of VA
              miss_valid_o = 1'b1;
              if (miss_ready_i) begin
                ctrl_slave.gnt = 1'b1;
                state_d = AwaitMissHandled;
              end
            end
            {8'h10, 1'b1}: begin // read lower 32 bit of miss
              miss_handler_req_o = 1'b1;
              if (miss_handler_gnt_i) begin
                ctrl_slave.gnt = 1'b1;
                if (miss_handler_valid_i) begin
                  resp_sel = PfnLower;
                  state_d = ReadMissUpper;
                end else begin
                  resp_sel = Error;
                end
                resp_valid = 1'b1;
              end
            end
            {8'h20, 1'b0}: begin // write ID to complete miss handling
              miss_handled_oup_id_o = ctrl_slave.wdata[$bits(core_id_t)-1:0];
              miss_handled_oup_req_o = 1'b1;
              if (miss_handled_oup_gnt_i) begin
                ctrl_slave.gnt = 1'b1;
                resp_sel = Okay;
                resp_valid = 1'b1;
              end
            end
            default: begin
              ctrl_slave.gnt = 1'b1;
              resp_sel = Error;
              resp_valid = 1'b1;
            end
          endcase
        end
      end

      AwaitMissHandled: begin
        if (miss_handled_inp_req_i) begin
          miss_handled_inp_gnt_o = 1'b1;
          resp_sel = Okay;
          resp_valid = 1'b1;
          state_d = Idle;
        end
      end

      ReadMissUpper: begin
        if (ctrl_slave.req) begin
          ctrl_slave.gnt = 1'b1;
          resp_valid = 1'b1;
          state_d = Idle;
          unique case ({ctrl_slave.add[7:0], ctrl_slave.wen})
            {8'h14, 1'b1}: begin // read upper 32 bit of miss
              resp_sel = PfnUpper;
            end
            default: begin
              resp_sel = Error;
            end
          endcase
        end
      end

      default: state_d = Idle;
    endcase
  end

  `FFARN(pfn_lower_q, pfn_lower_d, '0, clk_i, rst_ni)
  `FFARN(state_q, state_d, Idle, clk_i, rst_ni)

// pragma translate_off
`ifndef VERILATOR
  default disable iff rst_ni;
  assert property (@(posedge clk_i) (state_q == ReadMissUpper && resp_valid |-> resp_sel != Error))
    else $fatal(1, "Unsupported access, TLB miss may have been lost!");
`endif
// pragma translate_on

endmodule

module tcdm_resp_mux #(
  parameter int unsigned DataWidth = 32'd0,
  parameter int unsigned NumInputs = 32'd0,
  parameter type idx_t = logic [cf_math_pkg::idx_width(NumInputs)-1:0]
) (
  input  logic                                clk_i,
  input  logic                                rst_ni,
  input  idx_t                                sel_i,
  input  logic                                sel_valid_i,
  input  logic [NumInputs-1:0][DataWidth-1:0] data_i,
  output logic [NumInputs-1:0]                gnt_o,
  output logic                [DataWidth-1:0] rdata_o,
  output logic                                rvalid_o
);

  idx_t sel_d,  sel_q;
  logic         sel_valid_q;

  assign rdata_o = data_i[sel_q];
  assign rvalid_o = sel_valid_q;
  assign sel_d = sel_valid_i ? sel_i : sel_q;

  always_comb begin
    gnt_o = '0;
    if (sel_valid_q) begin
      gnt_o[sel_q] = 1'b1;
    end
  end

  `FFARN(sel_q, sel_d, '0, clk_i, rst_ni)
  `FFARN(sel_valid_q, sel_valid_i, '0, clk_i, rst_ni)

endmodule
