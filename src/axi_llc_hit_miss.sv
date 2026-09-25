// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author:
// - Wolfgang Roenninger <wroennin@iis.ee.ethz.ch>
// - Hong Pang <hongpang@ethz.ch>
// - Diyou Shen <dishen@ethz.ch>
// Date:   11.06.2019

/// This module houses the hit miss detection logic and the tag storage.
/// When a descriptor gets loaded into the unit the respective tag operation happens
/// depending on the input descriptor.
/// The unit starts uninitialized and starts in the first cycle after each reset
/// the tag pattern generator to perform a march X BIST onto the macros.
/// After the BIST is finished, the macros are initialyzed to all zero.
/// During initialisation no descriptors can enter the unit.
///
/// This unit keeps track of which cache lines are currently in use by descriptors
/// downstream with the help of a bloom filter. If there is a new descriptor, which
/// will access a cache line currently in use, it will be stalled until the line is
/// unlocked. This is to prevent data corruption.
///
/// There is an array of counters which keep track which IDs of descriptors
/// are currently in the miss pipeline. All subsequent hits which normally would go
/// through the bypass will get sent also towards the miss pipeline. However their
/// eviction and refill fields will not be set. This is to clear the unit from
/// descriptors, so that new ones from other IDs can use the hit bypass.
///
/// `axi_llc_pkg::EnMultilaneFilter`: when 1, this unit admits up to
/// `axi_llc_pkg::NumHitMissLanes` descriptors concurrently instead of one, so that a
/// descriptor stalled on a locked cache line no longer head-of-line-blocks admission
/// of later, unrelated descriptors. See the parameter's doc comment in
/// `axi_llc_pkg.sv` and `cheshire/CLAUDE.md` ("EnMultilaneFilter") for the full
/// design write-up. EXPERIMENTAL, default OFF, not yet verified in simulation - the
/// entire original single-slot implementation is preserved untouched (its own
/// generate branch below) when this is 0.
module axi_llc_hit_miss #(
  /// Stattic LLC configuration struct.
  parameter axi_llc_pkg::llc_cfg_t     Cfg            = axi_llc_pkg::llc_cfg_t'{default: '0},
  /// AXI parameter configuration
  parameter axi_llc_pkg::llc_axi_cfg_t AxiCfg         = axi_llc_pkg::llc_axi_cfg_t'{default: '0},
  /// Cache partitioning enabling parameter
  parameter logic                      CachePartition = 1,
  /// Maximum number of partitions (same as top-level MaxPartition).
  /// Used to size per-partition write counters in the miss counter unit.
  parameter int unsigned               MaxPartition   = 32'd0,
  /// Index remapping hash function used in cache partitioning
  parameter axi_llc_pkg::algorithm_e   RemapHash      = axi_llc_pkg::Modulo,
  /// LLC descriptor type
  parameter type                       desc_t         = logic,
  /// Lock struct definition. The lock signal indicate that a cache line is unlocked.
  ///
  ///  typedef struct packed {
  ///    logic [Cfg.IndexLength-1:0]      index;        // index of lock (cacheline)
  ///    logic [Cfg.SetAssociativity-1:0] way_ind;      // way which is locked
  ///  } lock_t;
  parameter type                       lock_t         = logic,
  /// Expected type definition definition of the miss counting struct
  ///
  /// typedef struct packed {
  ///   axi_slv_id_t id;    // Axi id of the count operation
  ///   axi_user_t   patid; // partition ID for per-partition write ordering
  ///   logic        rw;    // 0:read, 1:write
  ///   logic        valid; // valid, equals enable
  /// } cnt_t;
  parameter type                       cnt_t          = logic,
  /// Way indicator, is a onehot signal with width: `Cfg.SetAssociativity`.
  parameter type                       way_ind_t      = logic,
  parameter type                       set_ind_t      = logic,
  /// Cache partition table
  parameter type                       partition_table_t = logic,
  /// Whether to print SRAM configs
  parameter bit                        PrintSramCfg   = 0
) (
  /// Clock, positive edge triggered.
  input  logic     clk_i,
  /// Asynchronous reset, active low.
  input  logic     rst_ni,
  /// Testmode enable, active high.
  input  logic     test_i,
  /// Input descriptor payload.
  input  desc_t    desc_i,
  /// Input descriptor is valid.
  input  logic     valid_i,
  /// Module is ready to accept a new input descriptor.
  output logic     ready_o,
  /// Descriptor Output TODO
  output desc_t    desc_o,
  output logic     miss_valid_o,
  input  logic     miss_ready_i,
  output logic     hit_valid_o,
  input  logic     hit_ready_i,
  // Configuration input
  input  way_ind_t spm_lock_i,
  input  way_ind_t flushed_i,
  input  set_ind_t flushed_set_i,
  // unlock inputs from the units
  input  lock_t    w_unlock_i,
  input  logic     w_unlock_req_i,
  output logic     w_unlock_gnt_o,
  input  lock_t    r_unlock_i,
  input  logic     r_unlock_req_i,
  output logic     r_unlock_gnt_o,
  // counter inputs to count down
  input  cnt_t     cnt_down_i,
  // bist aoutput
  output way_ind_t bist_res_o,
  output logic     bist_valid_o
);
  `include "common_cells/registers.svh"
  localparam int unsigned IndexBase   = Cfg.ByteOffsetLength + Cfg.BlockOffsetLength;
  localparam int unsigned TagBase     = Cfg.ByteOffsetLength + Cfg.BlockOffsetLength +
                                        Cfg.IndexLength;
  // Effective partition count for the bloom filter.
  // Collapses to 1 (single shared filter = original behaviour) when
  // EnPartBloomFilter=0 or MaxPartition=0.
  localparam int unsigned NoPartsBF  =
    (MaxPartition == 0 || !axi_llc_pkg::EnPartBloomFilter) ? 1 : (MaxPartition + 1);
  localparam int unsigned PIDWidthBF = (NoPartsBF <= 1) ? 1 : $clog2(NoPartsBF);

  // Effective number of bloom-filter lookup ports / admission lanes.
  // Collapses to 1 (single shared lookup port = original behaviour) when
  // EnMultilaneFilter=0.
  localparam int unsigned NumLanes  =
    axi_llc_pkg::EnMultilaneFilter ? axi_llc_pkg::NumHitMissLanes : 32'd1;
  localparam int unsigned LaneIdxW  = (NumLanes <= 1) ? 1 : $clog2(NumLanes);

  // Type definitions for the requests and responses to/from the tag storage
  // typedef logic [Cfg.SetAssociativity-1:0] way_ind_t;
  typedef logic [Cfg.IndexLength-1:0]      index_t;
  typedef logic [Cfg.TagLength-1:0]        tag_t;

  /// Request struct to the tag storage.
  typedef struct packed {
    /// The request mode. What operation the tag storage should perform with the request.
    axi_llc_pkg::tag_mode_e mode;
    /// The indicatior encodes with a hot signal, to which ways the request should be made.
    way_ind_t               indicator;
    /// The index points to the cache line, for which the request is made.
    index_t                 index;
    /// The tag for which the request to the tag storage is made.
    tag_t                   tag;
    /// The tag is dirty, comes from a write.
    logic                   dirty;
  } store_req_t;

  /// The response will only come out of the tag storage, id a lookup request was made.
  typedef struct packed {
    /// The descriptor has to operate on this way
    way_ind_t indicator;
    /// The request has hit on a cache line.
    logic     hit;
    /// The tag currently stored is dirty.
    /// The tag storage wants to evict the current line stored at this position.
    logic     evict;
    /// The tag which is evicted.
    tag_t     evict_tag;
  } store_res_t;

  // Signals to/from the tag store (shared by both generate branches below)
  store_req_t store_req;
  logic       store_req_valid;
  logic       store_req_ready;

  store_res_t store_res;
  logic       store_res_valid;
  logic       store_res_ready;

  // lock signal (increment side; always reflects "whichever descriptor is currently
  // being dispatched", single-ported, unchanged regardless of EnMultilaneFilter)
  lock_t lock;
  logic  lock_req;
  logic  locked;
  logic [NoPartsBF-1:0] w_unlock_gnt_part;
  logic [NoPartsBF-1:0] r_unlock_gnt_part;
  // Effective partition IDs for bloom-filter routing.
  // Forced to 0 when EnPartBloomFilter=0 so all requests map to filter[0].
  logic [PIDWidthBF-1:0] lock_patid_bf, w_uf_patid_bf, r_uf_patid_bf;
  // up counting signal
  cnt_t  cnt_up;
  logic  cnt_stall;
  logic  to_miss;

  desc_t desc_temp;

  // -----------------------------------------------------------------------
  // Bloom-filter lookup ports: one per lane (NumLanes==1 in the default,
  // non-experimental configuration - see gen_single_slot below).
  // -----------------------------------------------------------------------
  lock_t [NumLanes-1:0]                lookup_arr;
  logic  [NumLanes-1:0][PIDWidthBF-1:0] lane_patid_bf;
  logic  [NumLanes-1:0]                lane_locked;
  logic  [NoPartsBF-1:0][NumLanes-1:0] locked_part_lane;

  for (genvar p = 0; unsigned'(p) < NoPartsBF; p++) begin : gen_lock_box
    axi_llc_lock_box_bloom #(
      .Cfg            ( Cfg    ),
      .lock_t         ( lock_t ),
      .NumLookupPorts ( NumLanes )
    ) i_lock_box_bloom (
      .clk_i,
      .rst_ni,
      .test_i,
      .lookup_i       ( lookup_arr                                           ),
      .locked_o       ( locked_part_lane[p]                                  ),
      .lock_i         ( lock                                                 ),
      .lock_req_i     ( lock_req & (lock_patid_bf == PIDWidthBF'(p))         ),
      .w_unlock_i     ( w_unlock_i                                          ),
      .w_unlock_req_i ( w_unlock_req_i & (w_uf_patid_bf == PIDWidthBF'(p))  ),
      .w_unlock_gnt_o ( w_unlock_gnt_part[p]                                ),
      .r_unlock_i     ( r_unlock_i                                          ),
      .r_unlock_req_i ( r_unlock_req_i & (r_uf_patid_bf == PIDWidthBF'(p))  ),
      .r_unlock_gnt_o ( r_unlock_gnt_part[p]                                )
    );
  end

  for (genvar l = 0; unsigned'(l) < NumLanes; l++) begin : gen_lane_locked_select
    assign lane_locked[l] = locked_part_lane[lane_patid_bf[l]][l];
  end
  // In the default (single-slot) configuration NumLanes==1 and lane 0's lookup is
  // always the currently-locking descriptor, so this reproduces the original
  // `locked = locked_part[desc_patid_bf]` exactly.
  assign locked = lane_locked[0];

  assign lock_patid_bf  = (NoPartsBF == 1) ? '0 : lock.patid[0+:PIDWidthBF];
  assign w_uf_patid_bf  = (NoPartsBF == 1) ? '0 : w_unlock_i.patid[0+:PIDWidthBF];
  assign r_uf_patid_bf  = (NoPartsBF == 1) ? '0 : r_unlock_i.patid[0+:PIDWidthBF];

  assign w_unlock_gnt_o = w_unlock_gnt_part[w_uf_patid_bf];
  assign r_unlock_gnt_o = r_unlock_gnt_part[r_uf_patid_bf];

  // inputs to the miss counter unit -- always derived from "whichever descriptor is
  // currently being dispatched" (desc_o), exactly as in the original design; only
  // WHICH descriptor that is differs between the two generate branches below.
  assign cnt_up.id    = desc_o.a_x_id;
  assign cnt_up.patid = desc_o.patid;
  assign cnt_up.rw    = desc_o.rw;
  assign cnt_up.valid = ~desc_o.flush & miss_valid_o & miss_ready_i;

  axi_llc_miss_counters #(
    .Cfg          ( Cfg          ),
    .MaxPartition ( MaxPartition ),
    .cnt_t        ( cnt_t        )
  ) i_miss_counters (
    .clk_i      (      clk_i ),
    .rst_ni     (     rst_ni ),
    .cnt_up_i   (     cnt_up ),
    .cnt_down_i ( cnt_down_i ),
    .to_miss_o  (    to_miss ),
    .stall_o    (  cnt_stall )
  );

  // inputs to the lock box (increment side)
  // Cache-Partition: the lock signal also needs to use the new index
  assign lock = '{
    patid:   desc_o.patid,
    index:   CachePartition ? desc_o.index_partition :
                              desc_o.a_x_addr[(Cfg.ByteOffsetLength + Cfg.BlockOffsetLength)+:Cfg.IndexLength],
    way_ind: desc_o.way_ind
  };
  // Lock it if a transfer happens on either channel and no flush!
  assign lock_req = ~desc_o.flush & ((miss_valid_o & miss_ready_i) | (hit_valid_o & hit_ready_i));

generate
  if (CachePartition && (RemapHash == (axi_llc_pkg::TruncDual))) begin
    axi_llc_trdl_index #(
      .Cfg    ( Cfg       ),
      .desc_t ( desc_t    )
    ) i_axi_llc_trdl_index (
      .desc_i ( desc_i    ),
      .desc_o ( desc_temp )
    );
  end else begin
    assign desc_temp = desc_i;
  end
endgenerate

  axi_llc_tag_store #(
    .Cfg         ( Cfg         ),
    .way_ind_t   ( way_ind_t   ),
    .store_req_t ( store_req_t ),
    .store_res_t ( store_res_t ),
    .PrintSramCfg ( PrintSramCfg )
  ) i_tag_store (
    .clk_i,
    .rst_ni,
    .test_i,
    .spm_lock_i   ( spm_lock_i      ),
    .flushed_i    ( flushed_i       ),
    .req_i        ( store_req       ),
    .valid_i      ( store_req_valid ),
    .ready_o      ( store_req_ready ),
    .res_o        ( store_res       ),
    .valid_o      ( store_res_valid ),
    .ready_i      ( store_res_ready ),
    .bist_res_o   ( bist_res_o      ),
    .bist_valid_o ( bist_valid_o    )
  );

  // =========================================================================
  // gen_single_slot: EnMultilaneFilter == 0 (default). This is the original,
  // pre-EnMultilaneFilter implementation, preserved byte-for-byte in its own
  // generate branch so the default configuration is provably unaffected by
  // this patch.
  // =========================================================================
  if (!axi_llc_pkg::EnMultilaneFilter) begin : gen_single_slot

    // Flipflops
    logic  busy_d,    busy_q, load_busy; // we have a valid descriptor in the unit
    logic  init_d,    init_q, load_init; // is the tag storage initialized?
    desc_t desc_d,    desc_q;            // descriptor residing in unit
    logic  load_desc;

    assign lookup_arr[0]    = lock;
    assign lane_patid_bf[0] = lock_patid_bf;

    // control
    always_comb begin
      // default assignments
      init_d    = init_q;
      load_init = 1'b0;
      busy_d    = busy_q;
      load_busy = 1'b0;
      desc_d    = desc_q;
      // output
      // Cache-Partition: If flush, recalculate the new index use old method to ensure the flush-by-set correct
      desc_o    = desc_q; // some fields get combinatorically overwritten from the tag lookup
      if (CachePartition) begin
        desc_o.index_partition = desc_q.flush ? desc_q.a_x_addr[IndexBase+:Cfg.IndexLength] : desc_q.index_partition;
      end
      load_desc = 1'b0;
      // unit handshaking
      ready_o      = 1'b0;
      miss_valid_o = 1'b0;
      hit_valid_o  = 1'b0;
      // inputs to the tag store
      store_req       = store_req_t'{mode: axi_llc_pkg::Bist, default: '0};
      store_req_valid = 1'b0;

      store_res_ready = 1'b0;

      // we are initialized, can operate on input descriptors
      if (init_q) begin

        // we have a valid descriptor in the unit and made the request to the tag store
        if (busy_q) begin
          if (desc_q.spm) begin
            /////////////////////////////////////////////////////////
            // SPM descriptor in unit
            /////////////////////////////////////////////////////////
            // check if the spm access would go onto a way configured as cache, if yes error
            if (|(desc_q.way_ind & (~spm_lock_i))) begin
              desc_o.x_resp = axi_pkg::RESP_SLVERR;
            end

            // only do something if we are not stalled or locked
            if (!(locked | cnt_stall)) begin
              // check if we have to go to hit or bypass
              if (!to_miss) begin
                hit_valid_o = 1'b1;
                // transfer
                if (hit_ready_i) begin
                  busy_d    = 1'b0;
                  load_busy = 1'b1;
                end
              end else begin
                miss_valid_o = 1'b1;
                // transfer
                if (miss_ready_i) begin
                  busy_d    = 1'b0;
                  load_busy = 1'b1;
                end
              end
            end
          end else begin
            ////////////////////////////////////////////////////////////////
            // NORMAL or FLUSH descriptor in unit, made req to tag_store
            // wait for the response
            ////////////////////////////////////////////////////////////////
            if (store_res_valid) begin
              if (desc_q.flush) begin
                // We have to send further, update desc_o
                desc_o.evict     = store_res.evict;
                desc_o.evict_tag = store_res.evict_tag;
                // check that the line is not locked!
                if (!locked) begin
                  miss_valid_o     = 1'b1;
                  // transfer of flush descriptor to miss unit
                  if (miss_ready_i) begin
                    store_res_ready = 1'b1;
                    busy_d          = 1'b0;
                    load_busy       = 1'b1;
                  end
                end
              end else begin
                /////////////////////////////////////////////////////////////
                // NORMAL lookup - differentiate between hit / miss
                /////////////////////////////////////////////////////////////
                // set out descriptor
                desc_o.way_ind   = store_res.indicator;
                desc_o.evict     = store_res.evict;
                desc_o.evict_tag = store_res.evict_tag;
                desc_o.refill    = store_res.hit ? 1'b0 : 1'b1;
                // determine if it has to go to the bypass or not if we are not stalled
                if (!(locked || cnt_stall)) begin
                  hit_valid_o  = ~to_miss &  store_res.hit;
                  miss_valid_o =  to_miss | ~store_res.hit;
                  // check for a transfer, do not update hit_valid or miss_valid from this point on!
                  if ((hit_valid_o && hit_ready_i) || (miss_valid_o && miss_ready_i)) begin
                    store_res_ready = 1'b1;
                    // New tag is written with the lookup or flush if it was necessary and the storage
                    // will go to ready, if it can take a new request.
                    // Does the module have a new descriptor at its input and we can take it?
                    if (valid_i) begin
                      // snoop at the descriptors spm, we do not have to make a lookup if it is spm
                      if (desc_temp.spm) begin
                        // load directly, if it is spm
                        ready_o   = 1'b1;
                        desc_d    = desc_temp;
                        load_desc = 1'b1;
                      end else begin
                        if (CachePartition) begin
                          // use the new index and tag to store the tag
                          store_req = store_req_t'{
                            mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                            indicator: desc_temp.flush ? desc_temp.way_ind     : ~flushed_i,
                            index:     desc_temp.flush ? desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength] : desc_temp.index_partition,
                            tag:       desc_temp.flush ? tag_t'(0)          : desc_temp.a_x_addr[IndexBase+:Cfg.TagLength],
                            dirty:     desc_temp.rw,
                            default:   '0
                          };
                        end else begin
                          // make the request to the tag store,
                          store_req = store_req_t'{
                            mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                            indicator: desc_temp.flush ? desc_temp.way_ind     : ~flushed_i,
                            index:     desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength],
                            tag:       desc_temp.flush ? tag_t'(0)          : desc_temp.a_x_addr[TagBase+:Cfg.TagLength],
                            dirty:     desc_temp.rw,
                            default:   '0
                          };
                        end
                        store_req_valid = 1'b1;
                        // transfer
                        if (store_req_ready) begin
                          ready_o   = 1'b1;
                          desc_d    = desc_temp;
                          load_desc = 1'b1;
                        end else begin
                          // go to idle and do nothing
                          busy_d    = 1'b0;
                          load_busy = 1'b1;
                        end
                      end
                    end else begin
                      // Go to IDLE otherwise
                      busy_d    = 1'b0;
                      load_busy = 1'b1;
                    end
                  end
                end
              end
            end
          end

        //////////////////////////////////////////////////////////////////////////////
        // we do not have a descriptor in our unit (not busy)
        //////////////////////////////////////////////////////////////////////////////
        end else begin
          // we signal that we are ready only, if there is a valid input descriptor
          if (valid_i) begin
            // snoop at the descriptors spm, we do not have to make a lookup if it is spm
            if (desc_temp.spm) begin
              // load directly, if it is spm
              ready_o   = 1'b1;
              busy_d    = 1'b1;
              load_busy = 1'b1;
              desc_d    = desc_temp;
              load_desc = 1'b1;
            end else begin
              if (CachePartition) begin
                // use the new index and tag to store the tag
                store_req = store_req_t'{
                  mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                  indicator: desc_temp.flush ? desc_temp.way_ind  : ~flushed_i,
                  index:     desc_temp.flush ? desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength] : desc_temp.index_partition,
                  tag:       desc_temp.flush ? tag_t'(0)          : desc_temp.a_x_addr[IndexBase+:Cfg.TagLength],
                  dirty:     desc_temp.rw,
                  default:   '0
                };
              end else begin
                // make the request to the tag store,
                store_req = store_req_t'{
                  mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                  indicator: desc_temp.flush ? desc_temp.way_ind  : ~flushed_i,
                  index:     desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength],
                  tag:       desc_temp.flush ? tag_t'(0)          : desc_temp.a_x_addr[TagBase+:Cfg.TagLength],
                  dirty:     desc_temp.rw,
                  default:   '0
                };
              end
              store_req_valid = 1'b1;
              // transfer
              if (store_req_ready) begin
                ready_o   = 1'b1;
                busy_d    = 1'b1;
                load_busy = 1'b1;
                desc_d    = desc_temp;
                load_desc = 1'b1;
              end
            end
          end // we had a new descriptor for loading
        end

      ///////////////////////////////////////////////////////////////////////////////
      // we come out of a reset, initialize the tag sram makros
      ///////////////////////////////////////////////////////////////////////////////
      end else begin
        // first cycle after reset start initialization of the sram makros
        store_req = store_req_t'{
          mode:      axi_llc_pkg::Bist,
          indicator: {Cfg.SetAssociativity{1'b1}},
          default:   '0
        };
        store_req_valid = 1'b1;
        if (store_req_ready) begin
          init_d    = 1'b1;
          load_init = 1'b1;
        end
      end
    end

    // registers
    `FFLARN(busy_q, busy_d, load_busy, '0, clk_i, rst_ni)
    `FFLARN(init_q, init_d, load_init, '0, clk_i, rst_ni)
    `FFLARN(desc_q, desc_d, load_desc, '0, clk_i, rst_ni)

  // =========================================================================
  // gen_multi_slot: EnMultilaneFilter == 1. EXPERIMENTAL, see axi_llc_pkg.sv and
  // cheshire/CLAUDE.md ("EnMultilaneFilter") for the design rationale and the
  // list of what has/has not been verified.
  // =========================================================================
  end else begin : gen_multi_slot

    typedef struct packed {
      logic  busy;         // lane holds a descriptor (from admission to dispatch)
      logic  lookup_done;  // tag-store response latched / SPM bypass -> desc fields valid
      desc_t desc;
    } lane_t;

    lane_t [NumLanes-1:0] lane_q, lane_d;

    logic                  init_d, init_q, load_init;
    logic                  owner_valid_q, owner_valid_d;
    logic [LaneIdxW-1:0]   owner_idx_q,   owner_idx_d;

    logic                  free_found;
    logic [LaneIdxW-1:0]   free_idx;

    logic [NumLanes-1:0]   cand_req, cand_gnt;
    logic                  cand_valid;
    logic [LaneIdxW-1:0]   cand_idx;

    // per-lane bloom-filter lookup data: continuously reflects each lane's own
    // (possibly not-yet-resolved) descriptor. Only lanes with lookup_done=1 are
    // ever selected by the completion arbiter below, so a not-yet-resolved lane's
    // transient lookup value is never actually consulted.
    for (genvar l = 0; unsigned'(l) < NumLanes; l++) begin : gen_lane_lookup
      assign lookup_arr[l] = '{
        patid:   lane_q[l].desc.patid,
        index:   CachePartition ?
                   (lane_q[l].desc.flush ? lane_q[l].desc.a_x_addr[IndexBase+:Cfg.IndexLength]
                                         : lane_q[l].desc.index_partition) :
                   lane_q[l].desc.a_x_addr[(Cfg.ByteOffsetLength+Cfg.BlockOffsetLength)+:Cfg.IndexLength],
        way_ind: lane_q[l].desc.way_ind
      };
      assign lane_patid_bf[l] = (NoPartsBF == 1) ? '0 : lookup_arr[l].patid[0+:PIDWidthBF];
    end

    // free-lane priority encoder (lowest free index wins)
    always_comb begin
      free_found = 1'b0;
      free_idx   = '0;
      for (int unsigned l = 0; l < NumLanes; l++) begin
        if (!free_found && !lane_q[l].busy) begin
          free_found = 1'b1;
          free_idx   = LaneIdxW'(l);
        end
      end
    end

    // completion / dispatch arbiter: round-robin among busy, resolved, unlocked
    // lanes. `gnt_i` is tied high so the round-robin pointer always advances when
    // some lane is eligible, independent of whether the miss-counter stall
    // (checked further below, on the chosen candidate only) actually allows a
    // dispatch this cycle -- this bounds the arbiter's fairness/liveness so a
    // repeatedly-stalled candidate cannot monopolize it. See design write-up.
    for (genvar l = 0; unsigned'(l) < NumLanes; l++) begin : gen_cand_req
      assign cand_req[l] = lane_q[l].busy & lane_q[l].lookup_done & ~lane_locked[l];
    end

    rr_arb_tree #(
      .NumIn     ( NumLanes ),
      .DataWidth ( 32'd1    ),
      .AxiVldRdy ( 1'b1     ),
      .LockIn    ( 1'b0     )
    ) i_dispatch_arb (
      .clk_i,
      .rst_ni,
      .flush_i ( 1'b0       ),
      .rr_i    ( '0         ),
      .req_i   ( cand_req   ),
      .gnt_o   ( cand_gnt   ),
      .data_i  ( '0         ),
      .gnt_i   ( 1'b1       ),
      .req_o   ( cand_valid ),
      .data_o  (            ),
      .idx_o   ( cand_idx   )
    );

    always_comb begin
      // ---- defaults: hold everything ----
      init_d    = init_q;
      load_init = 1'b0;
      for (int unsigned l = 0; l < NumLanes; l++) begin
        lane_d[l] = lane_q[l];
      end
      owner_valid_d = owner_valid_q;
      owner_idx_d   = owner_idx_q;

      ready_o      = 1'b0;
      miss_valid_o = 1'b0;
      hit_valid_o  = 1'b0;
      desc_o       = lane_q[0].desc; // don't-care default, see below

      store_req       = store_req_t'{mode: axi_llc_pkg::Bist, default: '0};
      store_req_valid = 1'b0;
      store_res_ready = 1'b0;

      if (!init_q) begin
        //////////////////////////////////////////////////////////////////////
        // Same one-time BIST/init sequence as the single-slot implementation.
        //////////////////////////////////////////////////////////////////////
        store_req = store_req_t'{
          mode:      axi_llc_pkg::Bist,
          indicator: {Cfg.SetAssociativity{1'b1}},
          default:   '0
        };
        store_req_valid = 1'b1;
        if (store_req_ready) begin
          init_d    = 1'b1;
          load_init = 1'b1;
        end
      end else begin

        //////////////////////////////////////////////////////////////////////
        // (1) Consume a pending tag-store response as soon as it arrives.
        // Acknowledged unconditionally (regardless of lock/cnt_stall) so the
        // single tag-store port is freed immediately and a locked descriptor
        // "parks" in its lane instead of blocking admission of others.
        //////////////////////////////////////////////////////////////////////
        if (owner_valid_q && store_res_valid) begin
          store_res_ready                    = 1'b1;
          lane_d[owner_idx_q].lookup_done    = 1'b1;
          lane_d[owner_idx_q].desc.way_ind   = store_res.indicator;
          lane_d[owner_idx_q].desc.evict     = store_res.evict;
          lane_d[owner_idx_q].desc.evict_tag = store_res.evict_tag;
          lane_d[owner_idx_q].desc.refill    = ~store_res.hit;
          owner_valid_d                      = 1'b0;
        end

        //////////////////////////////////////////////////////////////////////
        // (2) Admit a new descriptor into a free lane. SPM descriptors bypass
        // the tag store entirely (as in the original design). Non-SPM
        // descriptors are admitted only if the (single) tag-store port can
        // accept their lookup/flush request this same cycle -- this mirrors
        // the original ready_o semantics, just gated on "tag store free"
        // instead of "the single slot free".
        //
        // The non-SPM branch also requires `!owner_valid_q`: tag_store has an
        // internal fast path that can accept a new Lookup request the very
        // same cycle it hands off a clean-hit response (see
        // axi_llc_tag_store.sv, the nested `res_valid && res_ready` case) --
        // the single-slot design relies on exactly that overlap, and it is
        // safe there because there is only ever one context (`desc_q`).  With
        // multiple lanes it is NOT safe: the response for the *current* owner
        // only becomes externally visible (`store_res_valid`) one cycle later
        // (the output spill register adds a stage), so presenting a new
        // request before that arrives would let a second lane's admission
        // overwrite `owner_idx_q` before the first lane's response has been
        // consumed in step (1) above -- corrupting whichever lane happens to
        // be `owner_idx_q` when that stale response finally arrives. Waiting
        // for `!owner_valid_q` gives up that one-cycle pipelining opportunity
        // but guarantees at most one request is ever in flight between this
        // unit and the tag store, exactly as in the original design.
        //////////////////////////////////////////////////////////////////////
        if (valid_i && free_found) begin
          if (desc_temp.spm) begin
            ready_o          = 1'b1;
            lane_d[free_idx] = '{busy: 1'b1, lookup_done: 1'b1, desc: desc_temp};
          end else if (!owner_valid_q) begin
            if (CachePartition) begin
              store_req = store_req_t'{
                mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                indicator: desc_temp.flush ? desc_temp.way_ind  : ~flushed_i,
                index:     desc_temp.flush ? desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength]
                                            : desc_temp.index_partition,
                tag:       desc_temp.flush ? tag_t'(0) : desc_temp.a_x_addr[IndexBase+:Cfg.TagLength],
                dirty:     desc_temp.rw,
                default:   '0
              };
            end else begin
              store_req = store_req_t'{
                mode:      desc_temp.flush ? axi_llc_pkg::Flush : axi_llc_pkg::Lookup,
                indicator: desc_temp.flush ? desc_temp.way_ind  : ~flushed_i,
                index:     desc_temp.a_x_addr[IndexBase+:Cfg.IndexLength],
                tag:       desc_temp.flush ? tag_t'(0) : desc_temp.a_x_addr[TagBase+:Cfg.TagLength],
                dirty:     desc_temp.rw,
                default:   '0
              };
            end
            store_req_valid = 1'b1;
            if (store_req_ready) begin
              ready_o          = 1'b1;
              lane_d[free_idx] = '{busy: 1'b1, lookup_done: 1'b0, desc: desc_temp};
              owner_valid_d    = 1'b1;
              owner_idx_d      = free_idx;
            end
          end
        end

        //////////////////////////////////////////////////////////////////////
        // (3) Dispatch the arbiter's chosen, unlocked, resolved lane.
        //////////////////////////////////////////////////////////////////////
        if (cand_valid) begin
          desc_o = lane_q[cand_idx].desc;
          if (CachePartition) begin
            desc_o.index_partition = lane_q[cand_idx].desc.flush ?
              lane_q[cand_idx].desc.a_x_addr[IndexBase+:Cfg.IndexLength] :
              lane_q[cand_idx].desc.index_partition;
          end

          if (lane_q[cand_idx].desc.spm) begin
            // SPM: no tag lookup happened, decide purely from the miss counter.
            if (|(lane_q[cand_idx].desc.way_ind & (~spm_lock_i))) begin
              desc_o.x_resp = axi_pkg::RESP_SLVERR;
            end
            if (!cnt_stall) begin
              if (!to_miss) begin
                hit_valid_o = 1'b1;
                if (hit_ready_i) lane_d[cand_idx].busy = 1'b0;
              end else begin
                miss_valid_o = 1'b1;
                if (miss_ready_i) lane_d[cand_idx].busy = 1'b0;
              end
            end
          end else if (lane_q[cand_idx].desc.flush) begin
            // FLUSH: not subject to the miss-counter stall (matches the
            // original single-slot design, which only gates flush on `locked`).
            miss_valid_o = 1'b1;
            if (miss_ready_i) lane_d[cand_idx].busy = 1'b0;
          end else begin
            // NORMAL lookup: hit/miss was already latched into desc.refill
            // when the tag-store response was consumed in step (1) above
            // (refill = ~store_res.hit, exactly as the original design uses
            // store_res.hit directly).
            if (!cnt_stall) begin
              hit_valid_o  = ~to_miss & ~lane_q[cand_idx].desc.refill;
              miss_valid_o =  to_miss |  lane_q[cand_idx].desc.refill;
              if ((hit_valid_o && hit_ready_i) || (miss_valid_o && miss_ready_i)) begin
                lane_d[cand_idx].busy = 1'b0;
              end
            end
          end
        end
      end
    end

    `FFARN(lane_q, lane_d, '{default: '0}, clk_i, rst_ni)
    `FFLARN(init_q, init_d, load_init, '0, clk_i, rst_ni)
    `FFARN(owner_valid_q, owner_valid_d, 1'b0, clk_i, rst_ni)
    `FFARN(owner_idx_q, owner_idx_d, '0, clk_i, rst_ni)

  end

  // pragma translate_off
  `ifndef VERILATOR
    valid_o : assert property(
      @(posedge clk_i) disable iff (!rst_ni) !(miss_valid_o & hit_valid_o))
      else $fatal (1, "Duplicated descriptors, both valid outs are active.");

    detect_way_onehot : assert property(
      @(posedge clk_i) disable iff (!rst_ni) $onehot0(desc_o.way_ind))
      else $fatal(1, "[hit_miss.desc_o.way_ind] More than two bit set in the one-hot signal!");
  `endif
  // pragma translate_on
endmodule
