// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

/// Per-partition fair round-robin arbiter for LLC descriptor streams.
///
/// Incoming descriptors are steered into one of `NoPartitions` per-partition
/// FIFOs (depth `FifoDepth`) based on `desc_i.patid`, then served to the
/// single output in round-robin order across non-empty FIFOs.
///
/// When `MaxPartition == 0` (partitioning disabled) the module degenerates to
/// a single FIFO of depth `FifoDepth` with no arbitration overhead.
module axi_llc_partition_arbiter #(
  /// Max partition index (matches MaxPartition in axi_llc_top).
  parameter int unsigned MaxPartition = 0,
  /// Depth of each per-partition FIFO.
  parameter int unsigned FifoDepth    = 2,
  /// Descriptor type — must contain a `patid` field carrying the partition ID.
  parameter type         desc_t       = logic
) (
  input  logic   clk_i,
  input  logic   rst_ni,
  input  logic   test_i,
  // Input stream
  input  desc_t  desc_i,
  input  logic   valid_i,
  output logic   ready_o,
  // Output stream (round-robin arbitrated across non-empty partition FIFOs)
  output desc_t  desc_o,
  output logic   valid_o,
  input  logic   ready_i
);
  localparam int unsigned NoPartitions = (MaxPartition == 0) ? 1 : (MaxPartition + 1);
  localparam int unsigned PIDWidth     = (NoPartitions <= 1) ? 1 : $clog2(NoPartitions);

  // Push-side signals per FIFO (stream_fifo input handshake)
  logic  [NoPartitions-1:0] fifo_push;     // valid to each FIFO (only one active per cycle)
  logic  [NoPartitions-1:0] fifo_push_rdy; // ready_o of each FIFO (~full)

  // Pop-side signals per FIFO (stream_fifo output handshake)
  desc_t [NoPartitions-1:0] fifo_data_out;
  logic  [NoPartitions-1:0] fifo_pop_vld;  // valid_o of each FIFO (~empty)
  logic  [NoPartitions-1:0] fifo_pop;      // pop grant from arbiter

  // Demux: steer the input descriptor to the partition's FIFO.
  // We propagate valid only to the selected FIFO; data is shared (all FIFOs
  // receive desc_i but only the selected one is pushed).
  always_comb begin
    fifo_push = '0;
    fifo_push[desc_i.patid[0+:PIDWidth]] = valid_i;
  end
  // Input ready: back-pressure from the selected partition FIFO
  assign ready_o = fifo_push_rdy[desc_i.patid[0+:PIDWidth]];

  // Per-partition FIFOs
  for (genvar p = 0; unsigned'(p) < NoPartitions; p++) begin : gen_part_fifo
    stream_fifo #(
      .FALL_THROUGH ( 1'b0      ),
      .DEPTH        ( FifoDepth ),
      .T            ( desc_t    )
    ) i_part_fifo (
      .clk_i,
      .rst_ni,
      .flush_i    ( 1'b0               ),
      .testmode_i ( test_i             ),
      .usage_o    ( /* unused */       ),
      // push side
      .data_i     ( desc_i             ),
      .valid_i    ( fifo_push[p]       ),
      .ready_o    ( fifo_push_rdy[p]   ),
      // pop side
      .data_o     ( fifo_data_out[p]   ),
      .valid_o    ( fifo_pop_vld[p]    ),
      .ready_i    ( fifo_pop[p]        )
    );
  end

  // Round-robin arbiter: select among non-empty FIFOs
  rr_arb_tree #(
    .NumIn     ( NoPartitions ),
    .DataType  ( desc_t       ),
    .AxiVldRdy ( 1'b1         ),
    .LockIn    ( 1'b1         )
  ) i_rr_arb (
    .clk_i,
    .rst_ni,
    .flush_i ( '0             ),
    .rr_i    ( '0             ),
    .req_i   ( fifo_pop_vld   ),
    .gnt_o   ( fifo_pop       ),
    .data_i  ( fifo_data_out  ),
    .gnt_i   ( ready_i        ),
    .req_o   ( valid_o        ),
    .data_o  ( desc_o         ),
    .idx_o   ( /* unused */   )
  );

endmodule
