/**************************************************************************
   Copyright (c) 2014 Realsil MicroElectronics Inc. 
   All rights reserved. Copying, compilation, modification, distribution or 
   any other use whatsover of this material is strictly prohibited except in 
   accordance with a softwart license agreement with Realsil MicroElectronics Inc.
 -----------------------------------------------------------------------------
   Project             : Unknown
   File                : i2c_sr1
   Author              : zj
   Version             : 1.0
   Modify Date         : Sun Feb 25 15:03:18 2018
   Description         : TBD
 **************************************************************************/
`ifndef I2C_SR1
`define I2C_SR1
module i2c_sr1
  (
   input       clk_i,
   input       rstn_i,

   input       smb_alert_in_i,
   
   input       rw_pe_clr_i,
   input       rw_start_i,
   input       rw_nostretch_i,
   input       rw_smbus_i,
   input       rw_smbtype_i,
   input       rw_alert_i,
   input       rw_iterren_i,
   input       rw_itevten_i,
   input       rw_itbufen_i,
   input [5:0] rw_freq_i,
   input       rr_msl_i, // from sr2 , 0:slv 1:mst
   input       rr_tra_i, // from sr2 , 0:rx  1:tx
   input [7:0] rr_pec_i,
   
   output reg  rr_sb_o,
   output reg  rr_addr_o,
   output reg  rr_btf_o,
   output reg  rr_stopf_o,
   output      rr_rxne_o,
   output      rr_txe_o,
   output reg  rr_add10_o,
   input       rw_berr_i,
   output      rw_berr_set_o,
   output      rw_berr_val_o,
   input       rw_arlo_i,
   output      rw_arlo_set_o,
   output      rw_arlo_val_o,
   input       rw_af_i,
   output      rw_af_set_o,
   output      rw_af_val_o,
   input       rw_ovr_i,
   output      rw_ovr_set_o,
   output      rw_ovr_val_o,
   input       rw_pecerr_i,
   output      rw_pecerr_set_o,
   output      rw_pecerr_val_o,
   input       rw_timeout_i,
   output      rw_timeout_set_o,
   output      rw_timeout_val_o,
   input       rw_smbalert_i,
   output      rw_smbalert_set_o,
   output      rw_smbalert_val_o,

   output      it_event_o,
   output      it_error_o,
   output      ph_addr_o,

   input       mst_state_idle_i,
   input       mst_set_sb_i,
   input       mst_set_addr_i,
   input       slv_set_addr_i,
   input       mst_set_btf_i,
   input       slv_set_btf_i,
   input       mst_set_add10_i,
   input       mst_set_berr_i,
   input       slv_set_berr_i,
   input       mst_set_af_i,
   input       slv_set_af_i,
   input       slv_set_alert_i,
   input       slv_set_perr_i,
   input       mst_set_perr_i,
   input       mst_tx_cmplt_i,
   input       mst_rxbyte_rdy_i,
   input       ic_clk_oe_i,
   input       scl_int_i,

   input       s_det_i,
   input       p_det_i,
   input       tx_pop_i,
   input       rx_push_i,
   input       arb_lost_i,
   
   input       wr_dr_i,
   input       rd_dr_i,
   input       rd_sr2_i,
   input       wr_cr1_i
   );
    // **************************************************************** 
    // **                            intr                            ** 
    // ****************************************************************
    assign it_event_o  = rw_itevten_i & (rr_sb_o | rr_addr_o | rr_add10_o | rr_btf_o | rr_stopf_o | 
                                         (rw_itbufen_i & (rr_txe_o | rr_rxne_o)));
    assign it_error_o = rw_iterren_i & (rw_berr_i | rw_arlo_i | rw_af_i | rw_ovr_i | rw_pecerr_i | rw_timeout_i | rw_smbalert_i);
    // **************************************************************** 
    // **                        all reg flag                        ** 
    // **************************************************************** 
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rr_sb_o <= 1'b0;
        end
        else if(rw_pe_clr_i | wr_dr_i) begin
            rr_sb_o <= 1'b0;
        end
        else if(mst_set_sb_i) begin
            rr_sb_o <= 1'b1 ;
        end
    end
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rr_addr_o <= 1'b0;
        end
        else if(rw_pe_clr_i | rd_sr2_i) begin
            rr_addr_o <= 1'b0;
        end
        else if(mst_set_addr_i | slv_set_addr_i) begin
            rr_addr_o <= 1'b1;
        end
    end
    // use st_dr[1:0] to instead of btf/txe/rxne
    reg [1:0] st_dr;
    assign ph_addr_o = ~st_dr[1] ;
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            st_dr <= 2'b0;
        end
        else if(rw_pe_clr_i | rw_arlo_i | s_det_i | (p_det_i & rr_tra_i)) begin
            st_dr <= 2'b0;
        end
        else begin
            // [1] : valid after addr phase
            // [0] : 0:empty , 1:full for DR reg
            st_dr[1] <= rr_addr_o & rd_sr2_i ? 1'b1 : st_dr[1] ;
            st_dr[0] <= (wr_dr_i | rx_push_i) ? 1'b1 : (rd_dr_i | tx_pop_i) ? 1'b0 : st_dr[0] ;
        end
    end

    assign rr_txe_o  =  rr_tra_i & st_dr[1] & ~st_dr[0] ;
    assign rr_rxne_o  = ~rr_tra_i &st_dr[1] & st_dr[0] ;

    reg btf_set_r;
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            btf_set_r <= 1'b0;
        end
        else begin
            btf_set_r <= (mst_set_btf_i | slv_set_btf_i) ;
        end
    end

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rr_btf_o <= 1'b0;
        end
        else if(rw_pe_clr_i | p_det_i | s_det_i | rw_nostretch_i) begin
            rr_btf_o <= 1'b0;
        end
        else begin
            //rr_btf_o <= (mst_set_btf_i | slv_set_btf_i) ? 1'b1 : (tx_pop_i | rx_push_i) ? 1'b0 : rr_btf_o ;
            rr_btf_o <= btf_set_r ? 1'b1 : (tx_pop_i | rx_push_i) ? 1'b0 : rr_btf_o ;
        end
    end

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rr_add10_o <= 1'b0;
        end
        else begin
            // only for master mode
            rr_add10_o <= (rw_pe_clr_i | wr_dr_i) ? 1'b0 : mst_set_add10_i ? 1'b1 : rr_add10_o ;
        end
    end
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rr_stopf_o <= 1'b0;
        end
        else begin
            // only for slave mode
            rr_stopf_o <= (rw_pe_clr_i | rr_msl_i | wr_cr1_i) ? 1'b0 : (p_det_i & ~rw_af_i) ? 1'b1 : rr_stopf_o;
        end
    end
    // **************************************************************** 
    // **                            rc_w0                            ** 
    // ****************************************************************
    assign rw_berr_set_o  = rw_pe_clr_i | (mst_set_berr_i | slv_set_berr_i);
    assign rw_berr_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;

    assign rw_arlo_set_o  = rw_pe_clr_i | arb_lost_i ;
    assign rw_arlo_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;

    assign rw_af_set_o  = rw_pe_clr_i | (mst_set_af_i | slv_set_af_i) ;
    assign rw_af_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;

    assign rw_ovr_set_o  = rw_pe_clr_i | (slv_set_btf_i & rw_nostretch_i) ;
    assign rw_ovr_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;

    assign rw_pecerr_set_o  = rw_pe_clr_i | slv_set_perr_i | mst_set_perr_i;
    assign rw_pecerr_val_o  = rw_pe_clr_i ? 1'b0 : (rr_pec_i != 8'h0) ;

    // **************************************************************** 
    // **                       smbus timeout                        **
    // 1. scl low for 25ms                        : scl_int   = 0
    // 2. mst cumulative clock low more than 10ms : ic_clk_oe = 1
    // 3. slv cumulative clock low more than 25ms : ic_clk_oe = 1
    // max freq = 50M , N = 35*50 * 1000 = (/ (* 35 50 1000) 1024.0) = 1.7 * 1024 * 1024 = 21 bit
    // cnt 0 for Timout
    // cnt 1 for Tlow_mext or Tlow_sext
    // ****************************************************************
    wire ps0_us,ps0_ms,ps0_tout;
    wire ps1_us,ps1_ms,ps1_tout;
    reg [5:0] cnt0_ck,cnt1_ck;
    reg [9:0] cnt0_us,cnt1_us;
    reg [4:0] cnt0_ms,cnt1_ms;
    reg       cnt1_en;
    wire      ext_det;
    wire [4:0] tout1_ms;
    wire       cnt1_beg,cnt1_end;
    

    assign ps0_us = (cnt0_ck == rw_freq_i - 6'b1) ;
    assign ps0_ms = ps0_us & (cnt0_us == 10'd999) ;
    assign ps0_tout  = ps0_ms & (cnt0_ms == 5'd24) ; // 25ms tout
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            cnt0_ck  <= 6'b0;
            cnt0_us  <= 10'b0;
            cnt0_ms  <= 5'b0;
        end
        else if(~rw_smbus_i | scl_int_i)begin
            cnt0_ck  <= 6'b0;
            cnt0_us  <= 10'b0;
            cnt0_ms  <= 5'b0;
        end
        else begin
            cnt0_ck  <= ps0_us   ? 6'b0  : cnt0_ck + 6'b1 ;
            cnt0_us  <= ps0_ms   ? 10'b0 : cnt0_us + {9'd0,ps0_us} ;
            cnt0_ms  <= ps0_tout ? 5'b0  : cnt0_ms + {4'd0,ps0_ms} ;
        end
    end
    assign ext_det  = ic_clk_oe_i ;
    assign ps1_us = (cnt1_ck == rw_freq_i - 6'b1) ;
    assign ps1_ms  = ps1_us & (cnt1_us == 10'd999) ;
    assign tout1_ms  = rr_msl_i ? 5'd9 : 5'd24 ;
    // rr_msl_i default is 0 , so s_det_i always valid for mst and slv    
    assign cnt1_beg = (mst_tx_cmplt_i | mst_rxbyte_rdy_i | s_det_i) ; 
    assign cnt1_end = (mst_tx_cmplt_i | mst_rxbyte_rdy_i | p_det_i) ; 
    assign ps1_tout = ps1_ms & (cnt1_ms == tout1_ms) ;
    
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            cnt1_ck <= 6'b0;
            cnt1_us <= 10'b0;
            cnt1_ms <= 5'b0;
            cnt1_en <= 1'b0;
        end
        else if(~rw_smbus_i | p_det_i)begin
            // reset after p_det
            cnt1_ck <= 6'b0;
            cnt1_us <= 10'b0;
            cnt1_ms <= 5'b0;
            cnt1_en <= 1'b0;
        end
        else begin
            // reset after beg pulse , check at end pulse
            // beg and end pulse may be same time
            cnt1_en  <= cnt1_beg ? 1'b1  : cnt1_end ? 1'b0  : cnt1_en ;
            cnt1_ck  <= cnt1_beg ? 6'b0  : ps1_us   ? 6'b0  : cnt1_ck + {5'd0,cnt1_en & ext_det};
            cnt1_us  <= cnt1_beg ? 10'b0 : ps1_ms   ? 10'b0 : cnt1_us + {9'd0,ps1_us} ;
            cnt1_ms  <= cnt1_beg ? 5'b0  : ps1_tout ? 5'b0  : cnt1_ms + {4'd0,ps1_ms} ;
        end
    end
    
    assign rw_timeout_set_o  = rw_pe_clr_i | (ps0_tout | ps1_tout) ;
    assign rw_timeout_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;

    reg [1:0] alert_in_sync;
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            alert_in_sync <= 2'b11;
        end
        else if(rw_smbus_i & rw_smbtype_i) begin
            // only for smb host
            alert_in_sync <= {alert_in_sync[0],smb_alert_in_i};
        end
    end

    assign rw_smbalert_set_o  = rw_pe_clr_i | ~alert_in_sync[1] | slv_set_alert_i ;
    assign rw_smbalert_val_o  = rw_pe_clr_i ? 1'b0 : 1'b1 ;
    
endmodule
`endif
