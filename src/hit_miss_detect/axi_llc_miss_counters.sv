// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Wolfgang Roenninger <wroennin@iis.ee..ethz.ch>
// Date:   12.06.2019

/// This module counts if there are any descriptors of a given transaction ID
/// in the miss pipeline, and combinationally counts up or down the respective ID
/// of the cnt_t struct matches. There is a counter for all IDs and one for writes
/// as all writes have to be transferred in order.
///
/// Behaviour is controlled by the package-level feature flags:
///   axi_llc_pkg::EnPartWriteCounter — replicates the write counter per partition
///   axi_llc_pkg::EnPartReadCounter  — replicates the per-ID read counters per partition
/// Both flags are overridden to 0 (single-counter) when MaxPartition == 0.
module axi_llc_miss_counters #(
  /// Static LLC parameter configuration.
  parameter axi_llc_pkg::llc_cfg_t Cfg = axi_llc_pkg::llc_cfg_t'{default: '0},
  /// Maximum number of partitions (0 = partitioning disabled, single write counter).
  parameter int unsigned MaxPartition = 32'd0,
  /// Counter request type.
  ///
  /// typedef struct packed {
  ///   axi_slv_id_t id;        // AXI ID of the descriptor operating the counter
  ///   axi_user_t   patid;     // partition ID for per-partition write ordering
  ///   logic        rw;        // 0:read, 1:write
  ///   logic        valid;     // valid, equals enable of the counter
  /// } cnt_t;
  parameter type cnt_t  = logic
) (
  /// Clock, positive edge triggered.
  input  logic clk_i,
  /// Asynchronous reset, active low.
  input  logic rst_ni,
  /// The descriptor gets transferred into the miss pipeline.
  input  cnt_t cnt_up_i,
  /// The descriptor gets transferred out of the miss pipeline.
  input  cnt_t cnt_down_i,
  /// Tells that a descriptor should go to the miss pipeline.
  output logic to_miss_o,
  /// One of the counters is overflowing, stall descriptor!
  output logic stall_o
);
  localparam int unsigned NoCounters = 2**axi_llc_pkg::UseIdBits;

  // Effective partition counts for each counter type.
  // When a patch flag is 0 (or MaxPartition==0), NoPartsX=1 collapses the array
  // to a single slot, reproducing the original single-counter behaviour exactly.
  localparam int unsigned NoPartsR  =
    (MaxPartition == 0 || !axi_llc_pkg::EnPartReadCounter)  ? 1 : (MaxPartition + 1);
  localparam int unsigned NoPartsW  =
    (MaxPartition == 0 || !axi_llc_pkg::EnPartWriteCounter) ? 1 : (MaxPartition + 1);
  localparam int unsigned PIDWidthR = (NoPartsR <= 1) ? 1 : $clog2(NoPartsR);
  localparam int unsigned PIDWidthW = (NoPartsW <= 1) ? 1 : $clog2(NoPartsW);

  // Effective partition IDs used for counter indexing.
  // Forced to 0 when the corresponding patch is disabled so that everything
  // maps to slot [0], giving single-counter semantics.
  logic [PIDWidthR-1:0] up_patid_r, dn_patid_r;
  logic [PIDWidthW-1:0] up_patid_w, dn_patid_w;
  assign up_patid_r = (NoPartsR == 1) ? '0 : cnt_up_i.patid[0+:PIDWidthR];
  assign dn_patid_r = (NoPartsR == 1) ? '0 : cnt_down_i.patid[0+:PIDWidthR];
  assign up_patid_w = (NoPartsW == 1) ? '0 : cnt_up_i.patid[0+:PIDWidthW];
  assign dn_patid_w = (NoPartsW == 1) ? '0 : cnt_down_i.patid[0+:PIDWidthW];

  // Per-ID read-miss counters (replicated per partition when EnPartReadCounter=1)
  logic [NoPartsR-1:0][NoCounters-1:0] stall_id;
  logic [NoPartsR-1:0][NoCounters-1:0] en;
  logic [NoPartsR-1:0][NoCounters-1:0] down;
  logic [NoPartsR-1:0][NoCounters-1:0][axi_llc_pkg::MissCntWidth-1:0] q_miss;

  // Write-miss counters (replicated per partition when EnPartWriteCounter=1)
  logic [NoPartsW-1:0] stall_w;
  logic [NoPartsW-1:0] en_w;
  logic [NoPartsW-1:0] down_w;
  logic [NoPartsW-1:0][axi_llc_pkg::MissCntMaxWWidth-1:0] q_write;

  assign stall_o = |stall_id | |stall_w;

  always_comb begin : proc_control
    to_miss_o = 1'b0;
    for (int unsigned p = 0; p < NoPartsR; p++) begin
      for (int unsigned i = 0; i < NoCounters; i++) begin
        en[p][i]   = 1'b0;
        down[p][i] = 1'b0;
        if ((cnt_up_i.id[0+:axi_llc_pkg::UseIdBits] == i) &&
            (up_patid_r == PIDWidthR'(p)) && cnt_up_i.valid)
          en[p][i] = 1'b1;
        if ((cnt_down_i.id[0+:axi_llc_pkg::UseIdBits] == i) &&
            (dn_patid_r == PIDWidthR'(p)) && cnt_down_i.valid) begin
          if (en[p][i]) en[p][i] = 1'b0;
          else begin en[p][i] = 1'b1; down[p][i] = 1'b1; end
        end
        // to_miss: check this partition's ID counter
        if ((cnt_up_i.id[0+:axi_llc_pkg::UseIdBits] == i) && (up_patid_r == PIDWidthR'(p)))
          to_miss_o = |q_miss[p][i];
      end
    end
    // to_miss: additionally check write counter for the requesting partition
    if (cnt_up_i.rw)
      to_miss_o = to_miss_o || (|q_write[up_patid_w]);
  end

  // Write counter control: XOR of (up-write-for-p) and (down-write-for-p)
  for (genvar p = 0; unsigned'(p) < NoPartsW; p++) begin : gen_wpart_ctrl
    assign en_w[p] =
      (cnt_up_i.rw   & cnt_up_i.valid   & (up_patid_w == PIDWidthW'(p))) ^
      (cnt_down_i.rw & cnt_down_i.valid & (dn_patid_w == PIDWidthW'(p)));
    assign down_w[p] =
      ~(cnt_up_i.rw & cnt_up_i.valid & (up_patid_w == PIDWidthW'(p)));
  end

  for (genvar p = 0; unsigned'(p) < NoPartsR; p++) begin : gen_ppart_miss_counters
    for (genvar j = 0; unsigned'(j) < NoCounters; j++) begin : gen_cmiss_counters
      counter #(
        .WIDTH      ( axi_llc_pkg::MissCntWidth )
      ) i_miss_cnt (
        .clk_i      (          clk_i ),
        .rst_ni     (         rst_ni ),
        .clear_i    (             '0 ),
        .en_i       (       en[p][j] ),
        .load_i     (             '0 ),
        .down_i     (     down[p][j] ),
        .d_i        (             '0 ),
        .q_o        ( q_miss[p][j]   ),
        .overflow_o ( stall_id[p][j] )
      );
    end
  end

  for (genvar p = 0; unsigned'(p) < NoPartsW; p++) begin : gen_write_counters
    counter #(
      .WIDTH      ( axi_llc_pkg::MissCntMaxWWidth )
    ) i_miss_w_cnt (
      .clk_i      ( clk_i      ),
      .rst_ni     ( rst_ni     ),
      .clear_i    ( '0         ),
      .en_i       ( en_w[p]    ),
      .load_i     ( '0         ),
      .down_i     ( down_w[p]  ),
      .d_i        ( '0         ),
      .q_o        ( q_write[p] ),
      .overflow_o ( stall_w[p] )
    );
  end
endmodule
