// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Wolfgang Roenninger <wroennin@iis.ee..ethz.ch>
// Date:   12.06.2019

/// This module counts if there are any descriptors of a given transaction ID
/// in the miss pipeline, and combinationally counts up of down the respective ID
/// of the cnt_t struct matches. There is a counter for all id's and one for writes
/// as all writes have to be transferred in order within the same partition.
/// When MaxPartition > 0, a separate write counter is maintained per partition so
/// that writes from different partitions do not unnecessarily stall each other.
/// Likewise, the per-id read/miss counters are also replicated per partition, so
/// that two descriptors with the same lower AXI ID bits but from different
/// partitions never share a counter and never stall each other.
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
  localparam int unsigned NoCounters   = 2**axi_llc_pkg::UseIdBits;
  // Number of per-partition write counters: at least 1 (covers the disabled/single case).
  localparam int unsigned NoPartitions = (MaxPartition == 0) ? 1 : (MaxPartition + 1);
  // Minimum 1 bit so bit-selects on patid are always legal.
  localparam int unsigned PIDWidth     = (NoPartitions <= 1) ? 1 : $clog2(NoPartitions);

  // stall from the per-partition, per-id counters
  logic [NoPartitions-1:0][NoCounters-1:0] stall_id;
  // stall from per-partition write counters
  logic [NoPartitions-1:0] stall_w;

  logic [NoPartitions-1:0][NoCounters-1:0] en;
  logic [NoPartitions-1:0][NoCounters-1:0] down;

  // outputs of the per-partition, per-id counters
  logic [NoPartitions-1:0][NoCounters-1:0][axi_llc_pkg::MissCntWidth-1:0] q_miss;

  // per-partition write counter outputs and controls
  logic [NoPartitions-1:0][axi_llc_pkg::MissCntMaxWWidth-1:0] q_write;
  logic [NoPartitions-1:0] en_w;
  logic [NoPartitions-1:0] down_w;

  assign stall_o = |stall_id | |stall_w;

  always_comb begin : proc_control
    to_miss_o = 1'b0;
    for (int unsigned p = 0; p < NoPartitions; p++) begin
      for (int unsigned i = 0; i < NoCounters; i++) begin
        // default assignments
        en[p][i]   = 1'b0;
        down[p][i] = 1'b0;
        // we should count up
        if ((cnt_up_i.id[0+:axi_llc_pkg::UseIdBits] == i) &&
            (cnt_up_i.patid[0+:PIDWidth]            == PIDWidth'(p)) && cnt_up_i.valid) begin
          en[p][i]   = 1'b1;
        end

        // we should count down, or do nothing, if we are already counting up
        if ((cnt_down_i.id[0+:axi_llc_pkg::UseIdBits] == i) &&
            (cnt_down_i.patid[0+:PIDWidth]            == PIDWidth'(p)) && cnt_down_i.valid) begin
          if (en[p][i] == 1'b1) begin
            en[p][i]   = 1'b0;
          end else begin
            en[p][i]   = 1'b1;
            down[p][i] = 1'b1;
          end
        end
        // do we have to send the descriptor to the miss pipeline?
        if ((cnt_up_i.id[0+:axi_llc_pkg::UseIdBits] == i) &&
            (cnt_up_i.patid[0+:PIDWidth]            == PIDWidth'(p))) begin
          // first check the counter mapped to this partition's id
          to_miss_o = |q_miss[p][i];
          // if it is a write also check the per-partition write counter for this partition
          if (cnt_up_i.rw) begin
            to_miss_o = to_miss_o || (|q_write[p]);
          end
        end
      end
    end
  end

  // Per-partition write counter control.
  // For each partition p: enable when exactly one of (up-write-for-p, down-write-for-p) is active;
  // count down when the up side is not active for that partition.
  for (genvar p = 0; unsigned'(p) < NoPartitions; p++) begin : gen_wpart_ctrl
    assign en_w[p]   =
      (cnt_up_i.rw   & cnt_up_i.valid   & (cnt_up_i.patid[0+:PIDWidth]   == PIDWidth'(p))) ^
      (cnt_down_i.rw & cnt_down_i.valid & (cnt_down_i.patid[0+:PIDWidth] == PIDWidth'(p)));
    assign down_w[p] =
      ~(cnt_up_i.rw & cnt_up_i.valid & (cnt_up_i.patid[0+:PIDWidth] == PIDWidth'(p)));
  end

  for (genvar p = 0; unsigned'(p) < NoPartitions; p++) begin : gen_ppart_miss_counters
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

  for (genvar p = 0; unsigned'(p) < NoPartitions; p++) begin : gen_write_counters
    counter #(
      .WIDTH      ( axi_llc_pkg::MissCntMaxWWidth )
    ) i_miss_w_cnt (
      .clk_i      ( clk_i        ),
      .rst_ni     ( rst_ni       ),
      .clear_i    ( '0           ),
      .en_i       ( en_w[p]      ),
      .load_i     ( '0           ),
      .down_i     ( down_w[p]    ),
      .d_i        ( '0           ),
      .q_o        ( q_write[p]   ),
      .overflow_o ( stall_w[p]   )
    );
  end
endmodule
