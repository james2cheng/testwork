/**************************************************************************
   Copyright (c) 2014 Realsil MicroElectronics Inc. 
   All rights reserved. Copying, compilation, modification, distribution or 
   any other use whatsover of this material is strictly prohibited except in 
   accordance with a softwart license agreement with Realsil MicroElectronics Inc.
 -----------------------------------------------------------------------------
   Project             : Unknown
   File                : tim_etr
   Author              : zj
   Version             : 1.0
   Modify Date         : Fri Feb 23 13:33:37 2018
   Description         : TBD
 **************************************************************************/
`ifndef TIM_ETR
`define TIM_ETR
module tim_etr
  (
   input       clk_i,
   input       rstn_i, 

   input       rw_etp_i, // polarity
   input       rw_ece_i, // ext clk enable
   input [1:0] rw_etps_i, // prescaler : 0:off/1:div2/2:dev4/3:div8
   input [3:0] rw_etf_i, // same as filter
   input [1:0] rw_ckd_i,

   input       etr_i,
   output      etrf_lv_o,
   output      etrf_ps_o
   );

    reg [2:0]           etr_sync ;
    reg [2:0]           cnt_edg ;
    reg [2:0]           per_edg ;
    reg                 etrp_lv_r ;
    wire                etrp_pos ;
    wire                etrp_neg ;
    wire                etrp_act ;
    wire                etrp_lv ;
    wire                etrp_ps ;
    // etr_i is at most 1/4 of clk_i
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            etr_sync <= 3'd0;
        end
        else begin
            etr_sync <= {etr_sync[1:0],etr_i};
        end
    end
    assign etrp_lv   = etr_sync[1] ^ rw_etp_i ; // level
    //assign etrp_pos  = etrp_lv & ~etrp_lv_r ;
    assign etrp_pos  = etr_sync[1] & ~etr_sync[2] ;
    assign etrp_neg  = ~etr_sync[1] & etr_sync[2] ;
    assign etrp_act  = rw_etp_i ? etrp_neg : etrp_pos ;
    assign etrp_ps   = etrp_act & (cnt_edg == per_edg) ;
    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            //etrp_lv_r <= 1'b0;
            cnt_edg   <= 3'd0;
        end
        else begin
            //etrp_lv_r <= etrp_lv;
            cnt_edg   <= etrp_ps ? 3'd0 : cnt_edg + {2'd0,etrp_act} ;
        end
    end
    always @(*) begin
        case(rw_etps_i)
            2'b00: per_edg  = 3'd0;
            2'b01: per_edg  = 3'd1;
            2'b10: per_edg  = 3'd3;
            2'b11: per_edg  = 3'd7;
        endcase // case (rw_icps_i)
    end
    // **************************************************************** 
    // **                           filter                           **
    // When filter , it must using level signal
    // When PS not /1 , filter mustn't use 
    // ****************************************************************
    // only level signal go through filter
    assign etrf_ps_o = etrp_ps ;
    /* tim_flt auto_template(
     .tif_o               (etrf_lv_o),
     .ti_i                (etrp_lv),
     .rw_icf_i            (rw_etf_i[3:0]),
     );*/
    tim_flt u_flt (/*autoinst*/
                   // Outputs
                   .tif_o               (etrf_lv_o),             // Templated
                   // Inputs
                   .clk_i               (clk_i),
                   .rstn_i              (rstn_i),
                   .rw_ckd_i            (rw_ckd_i[1:0]),
                   .rw_icf_i            (rw_etf_i[3:0]),         // Templated
                   .ti_i                (etrp_lv));               // Templated
endmodule
`endif
