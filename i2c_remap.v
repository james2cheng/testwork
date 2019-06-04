`ifndef I2C_REMAP
    `define I2C_REMAP
module i2c_remap
  (
   input         clk_i,
   input         rstn_i,

   input         rw_dw_mode_i,
   input         rw_addmode_i,
   input         rw_ack_i,
   input         rw_pec_i,
   input         rw_start_i,
   input         rw_stop_i,
   input [9:0]   rw_add_i,
   input [6:0]   rw_add2_i,
   input         rw_engc_i,
   input         rw_enpec_i,
   input [7:0]   rw_dr_i,
   input         rw_endual_i,
   input         rw_smbus_i, // 1:smbus mode
   input         rw_enarp_i,
   input         rw_alert_i,
   input         rw_smbtype_i, // 0:device , 1:host
   input         rw_nostretch_i,
   input         rw_pos_i,
   input         rw_swrst_i,
   input         rw_timeout_i,
   input         rr_addr_i,
   input         rr_msl_i,
   input         rr_busy_i,
   input         rr_tra_i, // 0:rx , 1:tx
   input         rr_txe_i,
   input         rr_rxne_i,
   input         rr_btf_i,
   input [7:0]   rr_pec_i,
   input         rr_sb_i,
   // timing
   input [5:0]   rw_freq_i,
   input [11:0]  rw_ccr_i,
   input         rw_fsmode_i,
   input         rw_duty_i,

   input         p_det_i,
   input         s_det_i,
   input         rw_pe_i,
   input         rd_dr_i,

   input         nack_by_dma_i,
   
   input         ph_addr_i,
   input         rx_push_i,
   input         tx_pop_i,
   input         mst_set_add10_i,
   input         mst_set_addr_i,

   output [8:0]  tx_pop_data_o,
   output        ic_enable_o,
   output        ic_master_o,
   output        ic_slave_en_o,
   output        ic_10bit_mst_o,
   output        ic_10bit_slv_o,
   output        ic_ack_general_call_o,
   output [11:0] ic_tar_o,
   output [9:0]  ic_sar_o,
   output [15:0] ic_hcnt_o,
   output [15:0] ic_lcnt_o,
   output [15:0] ic_sda_hold_o, // need 300ns in ss/fs
   output        ic_srst_o,
   output        tx_empty_o,

   output        rd_dr_sync_o,
   output        rw_dw_mode_sync_o,
   output        rw_start_sync_o,
   output        rw_stop_sync_o,
   output        rw_ack_sync_o,
   output reg    rw_pec_sync_o,
   output        rw_endual_sync_o,
   output        rw_alert_sync_o,
   output        rw_nostretch_sync_o,
   output        rw_timeout_sync_o,
   output [6:0]  rw_add2_sync_o,
   output [7:0]  rw_dr_sync_o,
   output        rr_msl_sync_o,
   output        rr_busy_sync_o,
   output        rr_sb_sync_o,
   output        rr_tra_sync_o,
   output        rr_txe_sync_o,
   output        rr_rxne_sync_o,
   output        rr_btf_sync_o
   );

    wire         rw_pec_sync_raw ;
    reg          pec_sync_raw_r ;
    
    wire [15:0]  hcnt_ss  = {4'b0,rw_ccr_i};
    wire [15:0]  lcnt_ss  = {4'b0,rw_ccr_i};

    wire [15:0]  hcnt_fs  = ~rw_duty_i ? {4'b0,rw_ccr_i} : {1'b0,rw_ccr_i,3'b0} + {4'b0,rw_ccr_i} ;
    wire [15:0]  lcnt_fs  = ~rw_duty_i ? {3'b0,rw_ccr_i,1'b0} : {rw_ccr_i,4'b0} ;
    
    assign ic_hcnt_o  = rw_fsmode_i ? hcnt_fs : hcnt_ss ;
    assign ic_lcnt_o  = rw_fsmode_i ? lcnt_fs : lcnt_ss ;
    // **************************************************************** 
    // **                    SDA hold need 300ns                     **
    // **  set 500ns for easy
    // **  ic_sda_hold = 0.5*rw_freq_i = rw_freq >> 1
    // **************************************************************** 
    assign ic_sda_hold_o = {10'b0,rw_freq_i} >> 1 ;

    assign ic_enable_o = rw_pe_i;
    assign ic_master_o  = rr_msl_i;
    assign ic_slave_en_o  = ~rr_msl_i ;
    assign ic_10bit_slv_o = rw_addmode_i ;
    assign ic_10bit_mst_o  = rw_addmode_i ;
    assign ic_ack_general_call_o  = rw_engc_i ;
    assign ic_sar_o  = rw_addmode_i ? rw_add_i : {3'b0,rw_add_i[7:1]} ;
    assign tx_empty_o = 1'b0 ;
    assign tx_pop_data_o  = {~rr_tra_i, (rw_pec_sync_o ? rr_pec_i : rw_dr_i)} ;
    assign ic_tar_o  = 10'h3ff ;
    assign ic_srst_o = rw_swrst_i | ~rr_msl_i & rw_timeout_i;  

    // **************************************************************** 
    // **                         go through                         ** 
    // ****************************************************************
    reg  rw_ack_delay ;
    reg  rw_pec_delay ;
    reg  ph_addr_r ;

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            ph_addr_r <= 1'b0;
        end
        else begin
            ph_addr_r <= ph_addr_i;
        end
    end
    assign rr_msl_sync_o  = rr_msl_i;
    assign rr_busy_sync_o = rr_busy_i;
    assign rd_dr_sync_o   = rd_dr_i;
    assign rr_sb_sync_o   = rr_sb_i;
    assign rw_dr_sync_o      = rw_dr_i;
    assign rw_dw_mode_sync_o = rw_dw_mode_i;
    assign rr_tra_sync_o    = rr_tra_i;    
    // if mst-rev : rw_ack is 0 for last byte , so it not effect, soft will config rw_ack to 0
    // if slv-rev : rw_ack is 1 for last byte , it will see rr_pec also
    assign rw_ack_sync_o    = (rw_pos_i ? rw_ack_delay : rw_ack_i) & (~rw_pec_sync_o | (rr_pec_i == 8'h0)) & ~nack_by_dma_i;
    assign rw_pec_sync_raw  = rw_pos_i ? rw_pec_delay : rw_pec_i;    
    assign rw_start_sync_o  = rw_start_i;  
    assign rw_stop_sync_o  = rw_stop_i & ~(rw_pec_sync_o & rr_tra_i);  // only used for mst tx
    assign rw_dr_sync_o   = rw_dr_i;
    assign rw_nostretch_sync_o = rw_nostretch_i ;  
    assign rr_txe_sync_o  = ph_addr_i ? 1'b1 : (rr_txe_i &  ~rw_pec_sync_o); // when tx , pec is data
    assign rr_rxne_sync_o  = ph_addr_i ? 1'b1 : rr_rxne_i;
    assign rr_btf_sync_o  = ph_addr_i ? 1'b0 : rr_btf_i;
    assign rw_timeout_sync_o  = rw_timeout_i ;
    
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rw_ack_delay <= 1'b1;
            rw_pec_delay <= 1'b0;
        end
        else if(mst_set_addr_i) begin
            rw_ack_delay <= 1'b1;
            rw_pec_delay <= 1'b0;
        end
        else begin
            rw_ack_delay <= rx_push_i ? rw_ack_i : rw_ack_delay ;
            rw_pec_delay <= rx_push_i ? rw_enpec_i & rw_pec_i : rw_pec_delay ;
        end
    end
    // add2 shared with smb host/device addr
    assign rw_endual_sync_o  = ~rw_smbus_i ? (rw_endual_i & ~rw_addmode_i) : rw_enarp_i ;
    assign rw_add2_sync_o    = ~rw_smbus_i ? rw_add2_i : rw_smbtype_i ? 7'b0001000 : 7'b1100001 ;
    assign rw_alert_sync_o   = rw_smbus_i & ~rw_smbtype_i & rw_alert_i ; // only for dev

    // only for transmit : pec will be auto clear to 0
    // if receive , it is like rw_pec_raw
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            pec_sync_raw_r <= 1'b0;
            rw_pec_sync_o  <= 1'b0;
        end
        else if(~rw_pe_i | p_det_i | s_det_i) begin
            pec_sync_raw_r <= 1'b0;
            rw_pec_sync_o  <= 1'b0;
        end
        else begin
            pec_sync_raw_r <= rw_pec_sync_raw ;
            rw_pec_sync_o  <= (rw_pec_sync_raw & ~pec_sync_raw_r) ? 1'b1 : tx_pop_i ? 1'b0 : rw_pec_sync_o ;
        end
    end
endmodule
`endif
