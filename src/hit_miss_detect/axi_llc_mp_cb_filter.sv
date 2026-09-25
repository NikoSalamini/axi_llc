// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

/// Multi-lookup-port counting bloom filter.
///
/// This is a local re-implementation of common_cells' `cb_filter` (see
/// `cb_filter.sv`), used instead of forking/modifying that (git-pinned, shared)
/// module in place. It reuses `cb_filter`'s own `hash_block` and `counter`
/// primitives unmodified, and its bucket/increment/decrement logic is otherwise
/// identical to `cb_filter` -- the only difference is that the *lookup* side is
/// replicated into `NumLookupPorts` fully independent, purely combinational ports,
/// all reading the SAME shared bucket array (`bucket_occupied`). This is cheap
/// because a lookup is just a hash computation plus a comparison against the
/// (single, shared) bucket-occupied vector -- replicating it does not replicate the
/// bucket storage itself.
///
/// There is still only ONE increment port and ONE decrement port (as in
/// `cb_filter`): callers that only ever issue one lock/unlock per cycle (such as
/// `axi_llc_lock_box_bloom`) do not need those replicated, only the "is this line
/// currently locked" query needs to be checked from multiple places in parallel.
///
/// With `NumLookupPorts == 1` this module is functionally equivalent to `cb_filter`,
/// but callers should prefer instantiating `cb_filter` directly in that case (as
/// `axi_llc_lock_box_bloom` does) to keep the default, non-experimental path using
/// the original, unmodified module.
module axi_llc_mp_cb_filter #(
  /// Number of independent, concurrently-live lookup ports.
  parameter int unsigned NumLookupPorts =  32'd1,
  parameter int unsigned KHashes        =  32'd3,  // Number of hash functions
  parameter int unsigned HashWidth      =  32'd4,  // Number of counters is 2**HashWidth
  parameter int unsigned HashRounds     =  32'd1,  // Number of permutation substitution rounds
  parameter int unsigned InpWidth       =  32'd32, // Input data width
  parameter int unsigned BucketWidth    =  32'd4,  // Width of Bucket counters
  // the seeds used for seeding the PRG's inside each hash, one `cb_seed_t` per hash function.
  parameter cb_filter_pkg::cb_seed_t [KHashes-1:0] Seeds = cb_filter_pkg::EgSeeds
) (
  input  logic                                      clk_i,   // Clock
  input  logic                                       rst_ni,  // Active low reset
  // data lookup, one independent port per lane
  input  logic [NumLookupPorts-1:0][InpWidth-1:0]   look_data_i,
  output logic [NumLookupPorts-1:0]                 look_valid_o,
  // data increment (single port)
  input  logic [InpWidth-1:0]                       incr_data_i,
  input  logic                                      incr_valid_i,
  // data decrement (single port)
  input  logic [InpWidth-1:0]                       decr_data_i,
  input  logic                                      decr_valid_i,
  // status signals
  input  logic                                      filter_clear_i,
  output logic [HashWidth-1:0]                      filter_usage_o,
  output logic                                      filter_full_o,
  output logic                                      filter_empty_o,
  output logic                                      filter_error_o
);

  localparam int unsigned NoCounters = 2**HashWidth;

  // signal declarations
  logic [NumLookupPorts-1:0][NoCounters-1:0] look_ind;   // hash function pointers, per lookup port
  logic [NoCounters-1:0] incr_ind; // hash function pointers
  logic [NoCounters-1:0] decr_ind; // hash function pointers
  // bucket (counter signals) -- single, shared storage
  logic [NoCounters-1:0] bucket_en;
  logic [NoCounters-1:0] bucket_down;
  logic [NoCounters-1:0] bucket_occupied;
  logic [NoCounters-1:0] bucket_overflow;
  logic [NoCounters-1:0] bucket_full;
  logic [NoCounters-1:0] bucket_empty;
  // membership lookup signals, per lookup port
  logic [NumLookupPorts-1:0][NoCounters-1:0] data_in_bucket;
  // tot count signals (filter usage)
  logic cnt_en;
  logic cnt_down;
  logic cnt_overflow;

  // -----------------------------------------
  // Lookup Hashes - Membership Detection (one independent instance per lane)
  // -----------------------------------------
  for (genvar l = 0; unsigned'(l) < NumLookupPorts; l++) begin : gen_lookup_ports
    hash_block #(
      .NoHashes     ( KHashes         ),
      .InpWidth     ( InpWidth        ),
      .HashWidth    ( HashWidth       ),
      .NoRounds     ( HashRounds      ),
      .Seeds        ( Seeds           )
    ) i_look_hashes (
      .data_i       ( look_data_i[l]  ),
      .indicator_o  ( look_ind[l]     )
    );
    assign data_in_bucket[l] = look_ind[l] & bucket_occupied;
    assign look_valid_o[l]   = (data_in_bucket[l] == look_ind[l]) ? 1'b1 : 1'b0;
  end

  // -----------------------------------------
  // Increment Hash - Add Member to Set
  // -----------------------------------------
  hash_block #(
    .NoHashes     ( KHashes      ),
    .InpWidth     ( InpWidth     ),
    .HashWidth    ( HashWidth    ),
    .NoRounds     ( HashRounds   ),
    .Seeds        ( Seeds        )
  ) i_incr_hashes (
    .data_i       ( incr_data_i  ),
    .indicator_o  ( incr_ind     )
  );

  // -----------------------------------------
  // Decrement Hash - Remove Member from Set
  // -----------------------------------------
  hash_block #(
    .NoHashes     ( KHashes      ),
    .InpWidth     ( InpWidth     ),
    .HashWidth    ( HashWidth    ),
    .NoRounds     ( HashRounds   ),
    .Seeds        ( Seeds        )
  ) i_decr_hashes (
    .data_i       ( decr_data_i  ),
    .indicator_o  ( decr_ind     )
  );

  // -----------------------------------------
  // Control the incr/decr of buckets
  // -----------------------------------------
  assign bucket_down = decr_valid_i ? decr_ind : '0;

  always_comb begin : proc_bucket_control
    case ({incr_valid_i, decr_valid_i})
      2'b00 : bucket_en = '0;
      2'b10 : bucket_en = incr_ind;
      2'b01 : bucket_en = decr_ind;
      2'b11 : bucket_en = incr_ind ^ decr_ind;
      default: bucket_en = '0; // unreachable
    endcase
  end

  // -----------------------------------------
  // Counters (single, shared bucket storage)
  // -----------------------------------------
  for (genvar i = 0; i < NoCounters; i++) begin : gen_buckets
    logic [BucketWidth-1:0] bucket_content;
    counter #(
      .WIDTH( BucketWidth )
    ) i_bucket (
      .clk_i      ( clk_i             ),
      .rst_ni     ( rst_ni            ),
      .clear_i    ( filter_clear_i    ),
      .en_i       ( bucket_en[i]      ),
      .load_i     ( '0                ),
      .down_i     ( bucket_down[i]    ),
      .d_i        ( '0                ),
      .q_o        ( bucket_content    ),
      .overflow_o ( bucket_overflow[i])
    );
    assign bucket_full[i]     =  bucket_overflow[i] | (&bucket_content);
    assign bucket_occupied[i] = |bucket_content;
    assign bucket_empty[i]    = ~bucket_occupied[i];
  end

  // -----------------------------------------
  // Filter tot item counter
  // -----------------------------------------
  assign cnt_en   = incr_valid_i ^ decr_valid_i;
  assign cnt_down = decr_valid_i;
  counter #(
    .WIDTH ( HashWidth )
  ) i_tot_count (
    .clk_i     ( clk_i          ),
    .rst_ni    ( rst_ni         ),
    .clear_i   ( filter_clear_i ),
    .en_i      ( cnt_en         ),
    .load_i    ( '0             ),
    .down_i    ( cnt_down       ),
    .d_i       ( '0             ),
    .q_o       ( filter_usage_o ),
    .overflow_o( cnt_overflow   )
  );

  // -----------------------------------------
  // Filter Output Flags
  // -----------------------------------------
  assign filter_full_o  = |bucket_full;
  assign filter_empty_o = &bucket_empty;
  assign filter_error_o = |bucket_overflow | cnt_overflow;
endmodule
