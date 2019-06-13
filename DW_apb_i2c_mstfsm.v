// ------------------------------------------------------------------------
// --
// --                  (C) COPYRIGHT 2001-2010 SYNOPSYS, INC.
// --                            ALL RIGHTS RESERVED
// --
// --  This software and the associated documentation are confidential and
// --  proprietary to Synopsys, Inc.  Your use or disclosure of this
// --  software is subject to the terms and conditions of a written
// --  license agreement between you, or your company, and Synopsys, Inc.
// --
// --  The entire notice above must be reproduced on all authorized copies.
//
//
// File    : DW_apb_i2c_mstfsm.v
// Author  : Hani Saleh
// Created : Sep, 2002
// Abstract: I2C Master Control will be active when the I2C module is
//           configured for master mode of operation as defined by the
//           mode control bit.  This module will control: 
//           master-receiver or master-transmit functions in either
//           the 7-bit or 10-bit mode as defined by the ic_con 
///
//
// -------------------------------------------------------------------
// Revision: $Id: //dwh/DW_ocb/DW_apb_i2c/main/src/DW_apb_i2c_mstfsm.v#98 $
// -------------------------------------------------------------------

// -----------------------------------------------------------
// -- Macros
// -----------------------------------------------------------


module DW_apb_i2c_mstfsm
  (
   ic_rst_n,
   ic_clk,
  
   //Signals from pclk domain
   ic_enable_sync,
   ic_master_sync,
   ic_10bit_mst_sync,
   ic_hs_sync,
   tx_empty_sync,
   tx_empty_sync_hl,
   ic_rstrt_en_sync,
   ic_master_srst_sync,
   ic_srst_sync,
  
   //signals to the int_cntl
   mst_tx_abrt,
  
   //rx filter signals
   ic_bus_idle,
   arb_lost,
   ack_det,
  
   //Tx shift reg signals
   mst_tx_en,
   mst_rx_en,
   mst_tx_data_buf_in,
   start_en,
   re_start_en,
   split_start_en,
   mst_txfifo_ld_en,
   tx_fifo_data_buf,
   stop_en,
   mst_gen_ack_en,
   start_cmplt,   
   re_start_cmplt,   
   stop_cmplt,
   mst_tx_cmplt,
   byte_wait_scl,

   //clk_gen signals
   hs_mcode_en,
   min_hld_cmplt,
  
   //Rx shift reg signals
   mst_rxbyte_rdy,
   mst_push_rxfifo_en,
   mst_rx_cmplt,

   //signals from the reg file
   ic_hs_maddr,
   ic_tar,

   //fifo signals
   load_tx_shift,
   //slvfsm signals
   ic_rd_req,
   //misc signals
   mst_activity,

   //tx_abrt source indicators
   abrt_master_dis,//Access master while disabled
   abrt_sbyte_norstrt,//Send SBYTE while restart is disabled
   abrt_hs_norstrt,//Hisgh Speed mode while restart disabled
   abrt_hs_ackdet,//High Speed Master code was acknowledged
   abrt_sbyte_ackdet,//Start Byte was acknowleged
   abrt_gcall_read,//Try to read while sending a Gcall
   abrt_gcall_noack,//No slave acknowledged the G.CALL
   abrt_7b_addr_noack,//7bit 1address was not acknowledged
   abrt_txdata_noack,//Slave did not acknowledge sent data
   abrt_10addr1_noack,//10 bit 1address was not acknowledged
   abrt_10b_rd_norstrt,//10 bit read command while restart is disabled
   abrt_10addr2_noack,//10 bit 2address was not acknowledged

   //top level debug signals
   mst_debug_addr,
   mst_debug_data,
   mst_debug_cstate,
   /*autoarg*/
    // Outputs
    mst_fsm_wait_o, mst_set_sb_o, mst_set_addr_o, mst_set_add10_o,
    mst_set_btf_o, mst_set_af_o, mst_set_tra_o, mst_set_berr_o,
    mst_set_perr_o, mst_state_idle_o,
    // Inputs
    s_det_i, p_det_i, rw_dw_mode_sync_i, rw_start_sync_i,
    rw_stop_sync_i, rw_pec_sync_i, rw_ack_sync_i, rr_tra_sync_i,
    rw_dr_sync_i, rr_txe_sync_i, rr_rxne_sync_i, rr_btf_sync_i,
    rw_timeout_sync_i, rd_dr_sync_i, rr_msl_sync_i, rr_busy_sync_i,
    mst_go_addr_i, tx_bit_count, mst_rx_bit_count
    );

    // ------------------------------------------------------
    // -- Port declaration
    // ------------------------------------------------------
    // INPUTS
    input ic_clk;    // module clock: runs i2c module
    input ic_rst_n;  // asynchronous reset input active low
    
    input ic_enable_sync; // logic 1: enable i2c module
    input ic_master_sync; //logic 1: IC module is a Master; logic 0: slave
    input ic_10bit_mst_sync; // logic 1: IC 10-bit address transfer mode
    // logic 0: IC 7-bit address transfer mode
    input ic_hs_sync;  //logic 1: IC is in High Speed mode (3.4 Mb/s)
    input ic_bus_idle; //logic 1: IC bus is idle
    input tx_empty_sync; // tx fifo empty
    input tx_empty_sync_hl;//logic 1:high to low edge detection of tx_empty_sync
    input ic_rstrt_en_sync;//logic 1:Master can generate re-starts in general
    
    
    input arb_lost;   // logic 1: master lost arbitration
    input ack_det;    // logic 1: acknowledge detected
    input mst_rxbyte_rdy; //Indicates that a byte has been received
    input [`IC_HS_MADDR_RS-1:0] ic_hs_maddr;//the master address code register value
    input [`IC_TAR_RS-1:0]      ic_tar;//the target slave address register
    input [`IC_DATA_CMD_RS-1:0] tx_fifo_data_buf;//Buffer to hold data popped from tx fifo
    input                       start_cmplt;//logic 1:start condition has been generated               
    input                       re_start_cmplt;//logic 1: restart condition has been generated                
    input                       stop_cmplt;//logic 1:stop condition has been generated         
    input                       mst_tx_cmplt;//logic 1:master bit transmission is finished             
    input                       mst_rx_cmplt;//logic 1:master bit receiption is finished               
    input                       byte_wait_scl;//logic 1: wait for scl to go high before a restart, tx, rx or stop
    input                       ic_rd_req;//logic 1:Slave is waiting on data from the processor to tx
    input                       ic_master_srst_sync;//1:master soft reset
    input                       ic_srst_sync;//1:i2c soft reset
    input                       min_hld_cmplt;//Scl hasbeen pulled low and the
    // Minimum hold time to genearte 
    // start conditionhas elapsed
    //Outputs
    output [`IC_DATA_RS-1:0]    mst_tx_data_buf_in; // data to be transmitted on sda data out
    output                      start_en;   // Enable START condition
    output                      re_start_en;   // Enable RE-START condition 
    output                      split_start_en; // Enable Split start condition
    output                      mst_tx_en; // Enable tx shift register to transmit data
    output                      mst_rx_en; // Enable rx shift register to transmit data
    output                      mst_gen_ack_en; // Enable Ack gen. ckt
    output                      mst_tx_abrt;   // logic 1: master aborted TX transfer
    output                      load_tx_shift; // 
    output                      mst_txfifo_ld_en;// load tx_buffer from the tx fifo output
    output                      stop_en;   // Generate STOP condition
    output                      hs_mcode_en;//logic 1:master is in hs and transmitting the hs_mcode   
    output                      mst_push_rxfifo_en;//logic 1:push received data to the RX fifo
    output                      mst_activity;//logic 1: master is busy
    output                      mst_debug_addr;//logic 1:indicates master is transmitting the adress   
    output                      mst_debug_data;//logic 1:indicates master is transmitting the adress
    

    /////////////////////////////
    //tx_abrt source
    output                      abrt_master_dis;//Access master while disabled
    output                      abrt_sbyte_norstrt;//Send SBYTE while restart is disabled
    output                      abrt_hs_norstrt;//Hisgh Speed mode while restart disabled
    output                      abrt_hs_ackdet;//High Speed Master code was acknowledged
    output                      abrt_sbyte_ackdet;//Start Byte was acknowleged
    output                      abrt_gcall_read;//Try to read while sending a Gcall
    output                      abrt_gcall_noack;//No slave acknowledged the G.CALL
    output                      abrt_7b_addr_noack;//7bit 1address was not acknowledged
    output                      abrt_txdata_noack;//Slave did not acknowledge sent data
    output                      abrt_10addr1_noack;//10 bit 1address was not acknowledged
    output                      abrt_10b_rd_norstrt;//10 bit read command while restart is disabled
    output                      abrt_10addr2_noack;//10 bit 2address was not acknowledged
    //arb_lost ---> //Abort lost issue a tx abort as well

    output [4:0]                mst_debug_cstate;

    // new added
    input                       s_det_i;
    input                       p_det_i;
    input                       rw_dw_mode_sync_i;
    input                       rw_start_sync_i;
    input                       rw_stop_sync_i;
    input                       rw_pec_sync_i;
    input                       rw_ack_sync_i;
    input                       rr_tra_sync_i;  // todo
    input [7:0]                 rw_dr_sync_i;
    input                       rr_txe_sync_i;
    input                       rr_rxne_sync_i;
    input                       rr_btf_sync_i;
    input                       rw_timeout_sync_i;
    input                       rd_dr_sync_i;
    input                       rr_msl_sync_i;
    input                       rr_busy_sync_i;

    output                      mst_fsm_wait_o;
    input                       mst_go_addr_i;
    output                      mst_set_sb_o;
    output                      mst_set_addr_o; 
    output                      mst_set_add10_o;
    output                      mst_set_btf_o;
    output                      mst_set_af_o;   // ack fail set
    output                      mst_set_tra_o;
    output                      mst_set_berr_o;
    output                      mst_set_perr_o;
    output                      mst_state_idle_o;

    input [3:0]                 tx_bit_count;
    input [3:0]                 mst_rx_bit_count;

    reg                         mst_set_sb_o;
    //reg                         mst_set_addr_o;
    //reg                         mst_set_add10_o;
    reg                         mst_set_btf_o;
    reg                         mst_set_af_o;
    reg                         mst_set_perr_o;
    
    // ----------------------------------------------------------
    // -- local registers
    // ----------------------------------------------------------
    reg [4:0]                   mst_current_state;
    reg [4:0]                   mst_next_state;

    //non registers (wires) have to be defined as regs to be used in always block
    reg [`IC_DATA_RS-1:0]       mst_tx_data_buf_in;
    reg                         start_en_int;//gen start condition
    reg                         re_start_en_int;//gen re-start condition
    reg                         split_start_en_int; // split start
    reg                         mst_tx_en;//enable tx shifter
    reg                         mst_rx_en;//enable rx shifter
    reg                         mst_gen_ack_en_r, mst_gen_ack_en_s;//enable gen ACK in rx mode
    wire                        mst_gen_ack_en;
    reg                         mst_tx_abrt;//Master Tx aborted int.
    reg                         master_read;//logic 1: master is reading from the bus, 0: writing
    reg                         mst_txfifo_ld_en; //Load fifo data into TX buffer
    reg                         stop_en; //Generate stop condition
    reg                         delay_stop_en;
    reg                         addr_1byte_sent;//1st address byte has been sent
    reg                         addr_2byte_sent;//2nd address byte has been sent
    reg                         load_tx_shift; // 
    reg                         old_is_read; // Indicates if the previous transaction 
    // in a byte stream is read (1) or no transaction (0)
    // in a byte stream is write (1) or No Transaction (0)
    reg                         byte_waiting_q;//Indicates there is another byte to be processed for the RX_BYTE state
    reg                         mst_activity;//indicates that we can stop without performing
    //illegal action on I2C bus
    reg                         hs_mcode_en;//logic 1:master is in hs and transmitting the hs_mcode   
    reg                         mst_push_rxfifo_en;//logic 1:push received data to the RX fifo
    reg                         byte_waiting;//Indicates there is another byte to be processed for the RX_BYTE state
    reg                         mst_tx_flush;//logic 1: Master has flushed the tx fifo buffer
    reg                         tx_empty_hld;//Hold the value of tx_empty_sync_hl
    //   reg       byte_no1;//1: this is the 1st byte ever of the current transfer
    
    //tx_abrt source
    reg                         abrt_master_dis;//Access master while disabled
    reg                         abrt_sbyte_norstrt;//Send SBYTE while restart is disabled
    reg                         abrt_hs_norstrt;//Hisgh Speed mode while restart disabled
    reg                         abrt_hs_ackdet;//High Speed Master code was acknowledged
    reg                         abrt_sbyte_ackdet;//Start Byte was acknowleged
    reg                         abrt_gcall_read;//Try to read while sending a Gcall
    reg                         abrt_gcall_noack;//No slave acknowledged the G.CALL
    reg                         abrt_7b_addr_noack;//7bit 1address was not acknowledged
    reg                         abrt_txdata_noack;//Slave did not acknowledge sent data
    reg                         abrt_10addr1_noack;//10 bit 1address was not acknowledged
    reg                         abrt_10b_rd_norstrt;//10 bit read command while restart is disabled
    reg                         abrt_10addr2_noack;//10 bit 2address was not acknowledged
    //arb_lost ---> //Abort lost issue a tx abort as well

    
    // ----------------------------------------------------------
    // -- local wires
    // ----------------------------------------------------------
    wire                        start_en;//gen start condition
    wire                        re_start_en;//gen re-start condition
    wire                        split_start_en;

    // ----------------------------------------------------------
    // -- state variables (gray coded)
    // ----------------------------------------------------------
    parameter IDLE         = 5'b00000;//0
    parameter GEN_START    = 5'b00001;//1
    parameter TX_HS_MCODE  = 5'b00011;//3
    parameter POP_TX_DATA  = 5'b00010;//2
    parameter CHECK_IC_TAR = 5'b00110;//6
    parameter RX_BYTE      = 5'b00111;//7
    parameter GEN_STOP     = 5'b00101;//5
    parameter TX7_1ST_ADDR  = 5'b00100;//4
    parameter TX10_1ST_ADDR = 5'b01100;//c
    parameter TX10_2ND_ADDR = 5'b01101;//d
    parameter GEN_RSTRT_SBYTE = 5'b01110;//e
    parameter TX_BYTE   = 5'b01011;//b
    parameter GEN_RSTRT_10BIT = 5'b1010;//a
    parameter GEN_RSTRT_7BIT  = 5'b01001;//9
    parameter GEN_RSTRT_HS    = 5'b01000;//8
    parameter GEN_SPLIT_STOP  = 5'b01111;//f
    parameter GEN_SPLIT_START  = 5'b10101;//15
    // add new state
    parameter STM_WAIT_ADDR  = 5'b10000;//15
    parameter STM_WAIT_TRX  = 5'b10001;//15

    assign mst_fsm_wait_o  = (mst_current_state == STM_WAIT_ADDR) | (mst_current_state == STM_WAIT_TRX) ;
    assign mst_set_tra_o   = (mst_set_addr_o | mst_set_add10_o) & ~rw_dr_sync_i[0] ;  // 0:rx , 1:tx
    //assign mst_set_berr_o  = (p_det_i | s_det_i) & ((mst_current_state == TX_BYTE) | (mst_current_state == RX_BYTE)) ;
    assign mst_set_berr_o  = (p_det_i | s_det_i) & |{tx_bit_count,mst_rx_bit_count};
    assign mst_set_addr_o = (mst_next_state == STM_WAIT_TRX) & ack_det & 
                            ((mst_current_state == CHECK_IC_TAR) |
                             (mst_current_state == TX7_1ST_ADDR) |
                             (mst_current_state == TX10_1ST_ADDR) |
                             (mst_current_state == TX10_2ND_ADDR));
    assign mst_set_add10_o = (mst_next_state == STM_WAIT_ADDR) & (mst_current_state == TX10_1ST_ADDR) & ack_det ;
    
    //always @(posedge ic_clk or negedge ic_rst_n) begin
    //    if(ic_rst_n == 1'b0) begin
    //        mst_set_addr_o  <= 1'b0;            
    //        mst_set_add10_o <= 1'b0;
    //    end
    //    else begin
    //        mst_set_addr_o <= (mst_next_state == STM_WAIT_TRX) &
    //                           ((mst_current_state == CHECK_IC_TAR) |
    //                            (mst_current_state == TX7_1ST_ADDR) |
    //                            (mst_current_state == TX10_2ND_ADDR));
    //        mst_set_add10_o <= (mst_next_state != TX10_1ST_ADDR) & (mst_current_state == TX10_1ST_ADDR) ;
    //    end
    //end

    // ----------------------------------------------------------
    // -- Assigning outputs (get ride off a leda Warning: Reading from an output port)
    // ----------------------------------------------------------
    assign    start_en = start_en_int;
    assign    re_start_en = re_start_en_int;
    assign    split_start_en = split_start_en_int;
    
    // ----------------------------------------------------------
    // -- state assignment
    // ----------------------------------------------------------
    always @(posedge ic_clk or negedge ic_rst_n) begin : FSM_SEQ_PROC
        if(ic_rst_n == 1'b0) begin
            mst_current_state <= IDLE;
        end
        else if(~rw_dw_mode_sync_i) begin
            mst_current_state <= mst_next_state;
        end
        else begin
            if (
                ((mst_activity == 1'b0) && (ic_enable_sync  == 1'b0))
                || (ic_master_sync  == 1'b0)
                || (arb_lost == 1'b1)
                || (ic_master_srst_sync == 1'b1)
                || (ic_srst_sync == 1'b1)
                )  begin
                mst_current_state <= IDLE;
            end
            else if(rw_timeout_sync_i) begin
                mst_current_state <= GEN_STOP;
            end
            else begin
                mst_current_state <= mst_next_state;
            end
        end
    end

    // ----------------------------------------------------------
    // -- FSM Flags
    // ----------------------------------------------------------
    //start enable control flag   
    always @(posedge ic_clk or negedge ic_rst_n) begin : START_EN_INT_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              start_en_int <= 1'b0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 || (ic_srst_sync == 1'b1)
                 )
          begin
              start_en_int <= 1'b0;
          end
        else
          begin
              if ((mst_current_state == TX_HS_MCODE) 
                  || (mst_current_state == CHECK_IC_TAR)
                  || (mst_current_state == TX7_1ST_ADDR) 
                  || (mst_current_state == TX10_1ST_ADDR)
                  || (mst_current_state == IDLE)
                  || ((mst_next_state == GEN_SPLIT_START) 
                      && (min_hld_cmplt == 1'b1) 
                      && (start_cmplt == 1'b1))
                  )
                begin
                    start_en_int <= 1'b0;
                end

              else if (mst_next_state == GEN_START)
                begin
                    start_en_int <= 1'b1;
                end
              
              else if (mst_next_state == GEN_SPLIT_START)
                begin
                    start_en_int <= (start_en_int == 1'b0) ? ic_bus_idle:1'b1;
                end


          end // else: !if(ic_rst_n == 1'b0)
    end // block: START_EN_INT_FLAG_PROC
    
    
    //restart enable control flag
    always @(posedge ic_clk or negedge ic_rst_n) begin : RE_START_EN_INT_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              re_start_en_int <= 1'b0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 || (ic_srst_sync == 1'b1)
                 )
          begin
              re_start_en_int <= 1'b0;
          end
        else
          begin
              if ((mst_current_state == TX_BYTE) || (mst_current_state == RX_BYTE)
                  || (mst_current_state == CHECK_IC_TAR) ||(mst_current_state == TX7_1ST_ADDR) 
                  || (mst_current_state == TX10_1ST_ADDR)||(mst_current_state == IDLE)
                  || (mst_current_state == TX_HS_MCODE)
                  || (mst_current_state == STM_WAIT_ADDR)
                  )
                begin
                    re_start_en_int <= 1'b0;
                end

              else if ((mst_next_state == GEN_RSTRT_7BIT) || (mst_next_state == GEN_RSTRT_10BIT) 
                       || (mst_next_state == GEN_RSTRT_HS) || (mst_next_state == GEN_RSTRT_SBYTE) 
                       )
                begin
                    re_start_en_int <= ~byte_wait_scl;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: RE_START_EN_INT_FLAG_PROC

    always @(posedge ic_clk or negedge ic_rst_n) begin : SPLIT_START_EN_INT_PROC
        if(ic_rst_n==1'b0) begin
            split_start_en_int <= 1'd0;
        end else begin
            if(mst_next_state == GEN_SPLIT_START)
              split_start_en_int <= 1'd1;
            else
              split_start_en_int <= 1'd0;
        end
    end
    
    
    //previous transaction direction   
    always @(posedge ic_clk or negedge ic_rst_n) begin : OLD_IS_READ_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              old_is_read <= 1'b0;
          end
        else if ((ic_master_srst_sync == 1'b1)             
                 || (ic_srst_sync == 1'b1))
          begin
              old_is_read <= 0;
          end
        else
          begin
              if (mst_current_state == IDLE)             
                begin
                    old_is_read <= 0;
                end
              
              else if (mst_current_state == TX_BYTE) 
                begin
                    old_is_read <= 1'b0;
                end
              else  if (mst_current_state == RX_BYTE)
                begin
                    old_is_read <= 1'b1;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: OLD_IS_READ_FLAG_PROC
    
    //1st address byte sent flag   
    always @(posedge ic_clk or negedge ic_rst_n) begin : ADDR1_SENT_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              addr_1byte_sent <=1'b0;
          end
        else if((ic_master_srst_sync == 1'b1)
                || (ic_srst_sync == 1'b1)
                )
          begin
              addr_1byte_sent <= 1'b0;
          end
        else
          begin
              if ((mst_current_state == IDLE)
                  ||(mst_current_state == GEN_SPLIT_START)
                  )
                begin
                    addr_1byte_sent <= 1'b0;
                end
              
              else if ((mst_current_state == TX7_1ST_ADDR) 
                       || (mst_current_state == TX10_1ST_ADDR))
                begin
                    addr_1byte_sent <= 1'b1;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: ADDR1_SENT_FLAG_PROC
    
    
    //2nd address byte sent flag
    always @(posedge ic_clk or negedge ic_rst_n) begin : ADDR2_SENT_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              addr_2byte_sent <=1'b0;
          end
        else if((ic_master_srst_sync == 1'b1)
                || (ic_srst_sync == 1'b1)
                )
          begin
              addr_2byte_sent <= 1'b0;
          end
        else
          begin
              if ((mst_current_state == IDLE)
                  ||(mst_current_state == GEN_SPLIT_START)
                  )
                begin
                    addr_2byte_sent <= 1'b0;
                end
              
              else if (mst_current_state == TX10_2ND_ADDR)
                begin
                    addr_2byte_sent <= 1'b1;
                end
              
          end // else: !if(ic_rst_n == 1'b0)
    end // block: ADDR2_SENT_FLAG_PROC
    
    //RX_BYTE state byte is waiting flag
    always @(posedge ic_clk or negedge ic_rst_n) begin : BYTE_WAITING_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              byte_waiting_q <= 1'b0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 || (ic_srst_sync == 1'b1)
                 )
          begin
              byte_waiting_q <= 1'b0;
          end
        else
          begin
              if (((mst_next_state == RX_BYTE) && (byte_waiting == 1'b0)) 
                  || (mst_current_state == IDLE)
                  )
                begin
                    byte_waiting_q <= 1'b0;
                end
              
              else if (mst_next_state == RX_BYTE)
                begin
                    byte_waiting_q <= 1'b1;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: BYTE_WAITING_FLAG_PROC
    
    //master activity flag
    always @(posedge ic_clk or negedge ic_rst_n) begin : MST_ACTIVITY_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              mst_activity <= 1'b0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 && (ic_srst_sync == 1'b1)
                 )
            
          begin
              mst_activity <= 1'b0;
          end
        else
          begin
              if (mst_current_state != IDLE) 
                begin
                    mst_activity <= 1'b1;
                end
              else
                mst_activity <= 1'b0;
          end // else: !if(ic_rst_n == 1'b0)
    end // block: MST_ACTIVITY_FLAG_PROC
    
    //master flushed tx fifo buffer
    always @(posedge ic_clk or negedge ic_rst_n) begin : MST_TX_FLUSH_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              mst_tx_flush <= 0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 || (ic_srst_sync == 1'b1)
                 )
          begin
              mst_tx_flush <= 0;
          end
        else
          begin
              if ((mst_current_state == GEN_START) 
                  || (ic_enable_sync  == 1'b0) 
                  || (ic_rd_req == 1'b1)
                  )
                //                  else  if (mst_current_state == GEN_START)
                begin
                    mst_tx_flush <= 0;
                end
              
              else if((mst_tx_abrt == 1'b1) && (mst_current_state != IDLE))
                begin
                    mst_tx_flush <= 1'b1;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: MST_ACTIVITY_FLAG_PROC


    //master tx_empty_hld generation
    always @(posedge ic_clk or negedge ic_rst_n) begin : TX_EMPTY_FLAG_PROC
        if(ic_rst_n == 1'b0) 
          begin
              tx_empty_hld <= 0;
          end
        else if ((ic_master_srst_sync == 1'b1)
                 || (ic_srst_sync == 1'b1)
                 )
          begin
              tx_empty_hld <= 0;
          end
        else
          begin
              if ((mst_current_state == GEN_START) 
                  || (ic_enable_sync  == 1'b0) 
                  || (ic_rd_req == 1'b1)                    
                  )
                begin
                    tx_empty_hld <= 0;
                end
              else if((tx_empty_sync_hl == 1'b1) && (mst_current_state == IDLE))
                begin
                    tx_empty_hld <= 1'b1;
                end
          end // else: !if(ic_rst_n == 1'b0)
    end // block: MST_ACTIVITY_FLAG_PROC

    // ----------------------------------------------------------
    // -- This combinational process calculates the next state
    // -- and generate the outputs 
    // -- (Check RMM, 2nd Edition, Page 112)
    // ----------------------------------------------------------
    always @(*) begin: FSM_COMB_PROC
        //set default values
        mst_tx_abrt          = arb_lost;
        mst_tx_en            = 1'b0;
        mst_rx_en            = 1'b0;
        mst_gen_ack_en_s     = 1'b0;
        load_tx_shift        = 1'b0;
        mst_txfifo_ld_en     = 1'b0;
        stop_en              = 1'b0;
        master_read          = 1'b0;
        hs_mcode_en          = 1'b0;             
        mst_push_rxfifo_en   = 1'b0;
        mst_tx_data_buf_in   = {`IC_DATA_RS{1'b1}};
        mst_next_state       = IDLE;
        byte_waiting         = 1'b0;                   

        abrt_master_dis      = 1'b0;//Access master while disabled
        abrt_sbyte_norstrt   = 1'b0;//Send SBYTE while restart is disabled
        abrt_hs_norstrt      = 1'b0;//Hisgh Speed mode while restart disabled
        abrt_hs_ackdet       = 1'b0;//High Speed Master code was acknowledged
        abrt_sbyte_ackdet    = 1'b0;//Start Byte was acknowleged
        abrt_gcall_read      = 1'b0;//Try to read while sending a Gcall
        abrt_gcall_noack     = 1'b0;//No slave acknowledged the G.CALL
        abrt_7b_addr_noack   = 1'b0;//7bit 1address was not acknowledged
        abrt_txdata_noack    = 1'b0;//Slave did not acknowledge sent data
        abrt_10addr1_noack   = 1'b0;//10 bit 1address was not acknowledged
        abrt_10b_rd_norstrt  = 1'b0;//10 bit read command while restart is disabled
        abrt_10addr2_noack   = 1'b0;//10 bit 2address was not acknowledged

        mst_set_sb_o         = 1'b0;
        mst_set_af_o         = 1'b0;
        mst_set_btf_o        = 1'b0;
        mst_set_perr_o       = 1'b0;
        
        case (mst_current_state)
            IDLE :
              begin
                  //Control signals initialization
                  mst_tx_abrt         = 1'b0;
                  mst_tx_en           = 1'b0;
                  mst_rx_en           = 1'b0;
                  mst_gen_ack_en_s    = 1'b0;
                  load_tx_shift       = 1'b0;
                  mst_txfifo_ld_en    = 1'b0;
                  stop_en             = 1'b0;
                  hs_mcode_en         = 1'b0;             
                  mst_push_rxfifo_en  = rd_dr_sync_i & ~(rr_busy_sync_i & ~rr_msl_sync_i);
                  mst_tx_data_buf_in  = {`IC_DATA_RS{1'b1}};
                  byte_waiting        = 1'b0;             

                  if (~rw_dw_mode_sync_i & rw_start_sync_i & (ic_bus_idle == 1'b1)) begin
                      mst_next_state  = GEN_START;
                  end
                  else if (
                           (((mst_tx_flush == 1'b0)&&(tx_empty_sync   == 1'b0)) || // TX FIFO has data in it
                             ((mst_tx_flush == 1'b1)&&(tx_empty_hld == 1'b1))) &&  // TX FIFO has data in it                
                           (ic_bus_idle == 1'b1) // The bus is free
                           ) 
                    begin
                        //
                        if(
                           ((ic_rstrt_en_sync == 1'b0) &&//re_start is disabled and 
                            ((ic_hs_sync == 1'b1) //High speed transfer 
                             || (ic_tar[11:10] == 2'b11)))//send a start byte
                           || (ic_master_sync  == 1'b0)//master is disabled
                           ) 
                          begin
                              mst_tx_abrt = 1'b1;//Abrt invalid transaction while re_start is disabled

                              if(ic_master_sync  == 1'b0) abrt_master_dis  = 1'b1;
                              if(ic_tar[11:10] == 2'b11)  abrt_sbyte_norstrt = 1'b1;
                              if(ic_hs_sync == 1'b1)      abrt_hs_norstrt = 1'b1;

                              mst_next_state = IDLE;//Camp on Idle
                              
                          end
                        else
                          mst_next_state = GEN_START;//Generate a start condition and go ahead with the transfer
                        //
                    end 
                  else 
                    begin
                        mst_next_state = IDLE;//Remain in the idle state
                    end
              end // case: IDLE
            STM_WAIT_ADDR: begin
                if(rw_stop_sync_i) begin
                    // if no slave ack , AF assert , sw may set stop bit 
                    mst_next_state    = GEN_STOP ;
                end
                else if(mst_go_addr_i) begin
                    // addr DR have written
                    mst_txfifo_ld_en  = 1'b1 ;     // load tx_pop_data (DR) to txfifo_buf for furture use
                    if(rw_dr_sync_i == 8'h0) begin // GENCALL
                        mst_next_state = CHECK_IC_TAR;   // ic_tar[11:10] is remaped
                    end
                    else if(ic_10bit_mst_sync) begin
                        if(~addr_1byte_sent)
                          mst_next_state  = TX10_1ST_ADDR;
                        else
                          mst_next_state  = TX10_2ND_ADDR;
                    end
                    else begin
                        mst_next_state    = TX7_1ST_ADDR ;
                    end
                end
                else begin
                    mst_next_state    = STM_WAIT_ADDR ;
                end
            end
            STM_WAIT_TRX: begin
                //master_read       = tx_fifo_data_buf[8];
                master_read       = ~rr_tra_sync_i ;
                if(rw_stop_sync_i) begin
                    // if data nack ,AF assert, sw may set stop bit
                    mst_next_state    = GEN_STOP ;
                end
                else if(rw_start_sync_i) begin
                    // change direction
                    mst_next_state = ic_10bit_mst_sync ? GEN_RSTRT_10BIT : GEN_RSTRT_7BIT ;
                end
                else if(master_read & ~rr_rxne_sync_i) begin
                    mst_push_rxfifo_en  = rr_btf_sync_i ;
                    mst_next_state      = RX_BYTE  ;
                end
                else if(~master_read & ~rr_txe_sync_i) begin
                    mst_txfifo_ld_en  = 1'b1;
                    mst_next_state    = TX_BYTE  ;
                end
                else begin
                    mst_next_state    = STM_WAIT_TRX ;
                end
            end
            // =========================================================================================
            // When "start_cmplt" is asserted, this Master will have satisfied the minimum time for SDA
            // to stay LOW and move on to pulling SCL to LOW. This is t[HD,STA] requirement.
            // Meanwhile "min_hld_cmplt" waits to be asserted whenever SDA is pulled LOW, potentially
            // by *other* Masters.
            // Thus, if "start_cmplt" is HIGH and "min_hld_cmplt" is HIGH as well, then this Master have
            // already discovered that, after waiting for t[HD;STA], the I2C bus requires arbitration.
            // =========================================================================================

            GEN_START: begin //This state generates a Start Condition on the I2C bus

                if(ic_hs_sync == 1'b1)
                  hs_mcode_en   = 1'b1;//Use FS timing if in High Speed Mode

                if(start_cmplt == 1'b1) begin
                    if(~rw_dw_mode_sync_i) begin
                        mst_next_state  = STM_WAIT_ADDR;
                        mst_set_sb_o    = 1'b1;
                    end
                    else begin  //We still own the bus (Has a start condition been detected?)
                        
                        //Start detected go on
                        
                        //LK--ToBeDeleted if(min_hld_cmplt == 1'b1) //Another master is on the bus
                        //LK--ToBeDeleted                           // and The minimum hold time to generate
                        //LK--ToBeDeleted                           // a start has elapsed
                        //LK--ToBeDeleted                           // So park on the idle state until
                        //LK--ToBeDeleted                           // the bus is idle again
                        //LK--ToBeDeleted   mst_next_state = IDLE;

                        //LK--ToBeDeleted else if(ic_tar[11] == 1'b1)     //if ic_tar[11]=1 then  We are sending General Call or Start byte
                        if(ic_tar[11] == 1'b1)     //if ic_tar[11]=1 then  We are sending General Call or Start byte
                          mst_next_state = CHECK_IC_TAR;//Decide if we are sending General Call or start byte
                        
                        else  if (ic_hs_sync == 1'b1) begin //Are we in High speed mode?
                            mst_next_state = TX_HS_MCODE;     // We are in HS mode so send the HS Master Code
                        end
                        
                        else begin                      //if (ic_tar[11] = 1'b0) we are sending normal data
                            mst_txfifo_ld_en = 1'b1;      // load the FIFO data into the tx buf
                            // (pop data from tx fifo)
                            mst_next_state = POP_TX_DATA; //Complete Pop data process and process the data to be send
                        end
                    end
                end
                else begin
                    mst_next_state = GEN_START; // Wait for the Start condition to be detected,
                    // so loopback to GEN_START
                end
            end // case: GEN_START
            

            TX_HS_MCODE://This state transmits the HS Master Code on the I2C bus
              begin
                  mst_tx_data_buf_in = {`IC_HS_CODE,ic_hs_maddr}; //Fill the data buf with the correct value
                  hs_mcode_en = 1'b1;//State that we are sending the HS mode Master Code (MCODE)
                  
                  mst_tx_en = ~byte_wait_scl; // Enable Transmission of HS Master Code
                  
                  if (mst_tx_cmplt == 1'b1)
                    begin
                        if(ack_det == 1'b1) begin //MCODE should not be acknowledged (something is wrong)
                            mst_tx_en = 1'b0; // Stop TX of the data
                            hs_mcode_en = 1'b0;//We finished sending the MCODE
                            mst_tx_abrt = 1'b1;//Master aborted Transmission
                            
                            abrt_hs_ackdet = 1'b1;
                            
                            mst_next_state = GEN_STOP;//Generate a stop condition and quit the bus
                            
                        end
                        else
                          begin //We still own the bus and no SLAVE 
                              // acknowledged the  MCODE (correct behavior)
                              hs_mcode_en = 1'b0;//We finished transmitting the MCODE
                              mst_tx_en = 1'b0; // Disable Transmitter
                              mst_next_state = GEN_RSTRT_HS;
                          end 
                    end
                  else 
                    begin//We are still waiting for a start or arb lost or ack or not ack signals
                        mst_next_state = TX_HS_MCODE;
                    end
                  
              end // case: TX_HS_MCODE
            
            
            CHECK_IC_TAR://This State sends general call or start byte to the I2C bus 
              begin

                  if(~rw_dw_mode_sync_i | (ic_tar[11:10] == 2'b10)) begin //Are we sending a general call address?
                      mst_tx_data_buf_in  = 8'h00;// Load Tx data buffer with a general Call "00h"

                      hs_mcode_en         = ic_hs_sync;//1'b1;//Use FS timing if in High Speed Mode

                      mst_tx_en           = ~byte_wait_scl; // Enable transmitter to send the Gen. Call if we are not in byte waiting mode
                      
                      
                      if (mst_tx_cmplt == 1'b1) begin
                          if(~rw_dw_mode_sync_i) begin
                              // no matter ack or nack
                              mst_next_state    = STM_WAIT_TRX;
                              mst_set_af_o      = ~ack_det ;
                          end
                          else if (ack_det == 1'b1)  begin //We still own the bus, has a slave acknowldged the Gen Call
                              mst_tx_en = 1'b0;//We have an acknowledge so procede to next byte
                              mst_txfifo_ld_en = 1'b1; // Load the FIFO data into the tx buf
                          end else
                            begin 
                                //No slave ackwnoledged the General call so abort transfer
                                mst_tx_abrt = 1'b1;//Master aborted transfer
                                abrt_gcall_noack =  1'b1;//No slave acknowledged the G.CALL
                                mst_next_state = GEN_STOP;//Generate a stop condition
                            end 
                      end
                      else begin
                          mst_next_state = CHECK_IC_TAR; // Wait for an ACK signal
                      end
                  end
                  
                  else //if(ic_tar[11:10] == 2'b11) //Tx Start Byte
                  begin
                      // never coming here when STM mode
                      hs_mcode_en = ic_hs_sync;//Use FS timing if in High Speed Mode
                      
                      mst_tx_data_buf_in   = 8'h01; //Set the Start Byte data
                      
                      mst_tx_en = ~byte_wait_scl; //Enable the transmitter

                      if (mst_tx_cmplt == 1'b1) //Start byte should not be acknwledged (correct behavior)
                        begin
                            if (ack_det == 1'b1)
                              begin //something is wrong on the bus
                                  mst_tx_abrt = 1'b1;
                                  abrt_sbyte_ackdet = 1'b1;
                                  
                                  mst_next_state = GEN_STOP;
                              end 
                            else
                              begin
                                  mst_tx_en = 1'b0;
                                  mst_next_state = GEN_RSTRT_SBYTE;
                              end
                        end
                      else
                        mst_next_state = CHECK_IC_TAR;
                  end // if (ic_tar[11:10] = 2'b11)
              end // case: CHECK_IC_TAR
            
            
            
            POP_TX_DATA:
              begin
                  mst_txfifo_ld_en = 1'b0; //Latch the Fifo data into the TX buffer
                  master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                  if((mst_tx_cmplt == 1'b1) || (mst_rx_cmplt == 1'b1))
                    begin
                        mst_next_state = POP_TX_DATA;
                    end
                  ////----> Case 1: we are in General call processing

                  else if(ic_tar[11:10] == 2'b10) begin //IC is sending general call data
                      /////
                      if(master_read == 1'b0) //Master is writing
                        begin
                            mst_next_state = TX_BYTE;
                        end
                      else //Master is reading (not allowed)
                        begin
                            mst_tx_abrt = 1'b1;
                            abrt_gcall_read = 1'b1;
                            
                            mst_next_state = GEN_STOP;
                        end        
                      /////
                        end // if (ic_tar[11:10] == 2'b10)
                  
                  
                  ////----> Case 2: we are in 7 bit address mode
                  else
                    begin
                        if(ic_10bit_mst_sync == 1'b0) begin //IC is in 7 bit address mode
                            /////
                            
                            if ((master_read != old_is_read) && (addr_1byte_sent == 1'b1))//are we changing direction?
                              begin //gen re-start condition to change the direction of the transaction
                                  if(ic_rstrt_en_sync == 1'b1)
                                    mst_next_state = GEN_RSTRT_7BIT;
                                  else
                                    mst_next_state = GEN_SPLIT_STOP;
                              end
                            
                            else if(addr_1byte_sent == 1'b0) 
                              begin
                                  mst_next_state = TX7_1ST_ADDR;
                              end 
                            else 
                              begin
                                  mst_next_state = TX_BYTE;
                              end        
                            /////
                            ////----> Case 3: we are in 10 bit address mode
                        end else 
                          begin // IC is in 10 bit address mode
                              
                              if ((master_read != old_is_read) && (addr_2byte_sent == 1'b1))//change dir of transfer
                                begin //gen re-start condition to change the direction of the transaction
                                    if(ic_rstrt_en_sync == 1'b1)
                                      mst_next_state = GEN_RSTRT_10BIT;
                                    else
                                      mst_next_state = GEN_SPLIT_STOP;
                                end
                              
                              else if(addr_1byte_sent == 1'b0)
                                begin
                                    mst_next_state = TX10_1ST_ADDR;
                                end 
                              else
                                begin
                                    mst_next_state = TX_BYTE;
                                end   
                              
                          end // else: !if(ic_10bit_mst_sync == 1'b0)
                    end // else: !if(ic_tar[11:10] == 2'b10)
                  
              end // case: POP_TX_DATA

            TX7_1ST_ADDR:
              begin
                  //master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                  //mst_tx_data_buf_in   = {ic_tar[6:0],master_read}; //Set the Start Byte data
                  master_read = rw_dr_sync_i[0];
                  mst_tx_data_buf_in   = rw_dr_sync_i;
                  mst_tx_en = ~byte_wait_scl ; //Enable the transmitter
                  
                  if (mst_tx_cmplt == 1'b1)
                    begin
                        if(~rw_dw_mode_sync_i) begin
                            // no matter ack or nack
                            mst_next_state  = STM_WAIT_TRX ;
                            mst_set_af_o    = ~ack_det ;
                        end
                        else if (ack_det == 1'b1)
                          begin //slave acknowledged the address
                              mst_tx_en = 1'b0;//disable the transmitter

                              if(master_read ==1'b1)
                                begin
                                    if(tx_empty_sync == 1'b0) //We have more data in tx fifo
                                      begin
                                          mst_txfifo_ld_en = 1'b1; //Load the fifo data to the tx buffer
                                          byte_waiting = 1'b1;
                                      end
                                    else
                                      begin
                                          byte_waiting = 1'b0;
                                      end
                                    mst_next_state = RX_BYTE;
                                end
                              else begin
                                  mst_next_state  = TX_BYTE ;
                              end
                          end
                        
                        else
                          begin
                              mst_tx_abrt = 1'b1;
                              abrt_7b_addr_noack = 1'b1;
                              
                              mst_tx_en = 1'b0; //disable the transmitter
                              mst_next_state = GEN_STOP;
                          end 
                    end
                  else
                    mst_next_state = TX7_1ST_ADDR; // Wait for an ACK signal

              end // case: TX_1ST_ADDR
            
            
            TX_BYTE:   
              begin

                  if(ic_tar[11:10] == 2'b10) 
                    begin
                        //IC is sending general call data
                        hs_mcode_en = ic_hs_sync;//1'b1;//Use FS timing if in High Speed Mode
                    end
                  else
                    hs_mcode_en       = 1'b0;
                  
                  
                  mst_tx_data_buf_in  = tx_fifo_data_buf[7:0];
                  mst_tx_en           = ~byte_wait_scl;

                  if(mst_tx_cmplt == 1'b1) begin
                      if(~rw_dw_mode_sync_i) begin
                          mst_tx_en       = 1'b0;//disable the transmitter
                          mst_next_state  = rw_stop_sync_i ? GEN_STOP :
                                            rw_start_sync_i ? (ic_10bit_mst_sync ? GEN_RSTRT_10BIT : GEN_RSTRT_7BIT) :
                                            ~rr_txe_sync_i ? TX_BYTE :  STM_WAIT_TRX ;
                          mst_txfifo_ld_en  = ~rw_stop_sync_i & ~rr_txe_sync_i ;
                          mst_set_btf_o     = rr_txe_sync_i ;
                          mst_set_af_o      = ~ack_det ;
                      end
                      else if (ack_det == 1'b1) 
                        begin //slave acknowledged the data bytes
                            if(tx_empty_sync == 1'b0) //We have more data in tx fifo
                              begin
                                  mst_txfifo_ld_en = 1'b1; //Load the fifo data to the tx buffer
                                  mst_next_state = POP_TX_DATA;
                              end
                            else //No more data to process
                              begin
                                  mst_next_state = GEN_STOP;
                              end
                            //end
                            //else begin
                            //    mst_next_state = TX_BYTE ;
                            //    //mst_next_state = rw_stop_i ? GEN_STOP : TX_BYTE ;
                            //end
                        end
                      else
                        begin
                            mst_tx_abrt = 1'b1;
                            abrt_txdata_noack = 1'b1;
                            
                            mst_tx_en = 1'b0;//disable the transmitter
                            mst_next_state = GEN_STOP;
                            
                        end
                  end
                  else // else: !if(ack_det == 1'b1)
                      
                    mst_next_state = TX_BYTE; // Wait for an ACK signal
                  
              end // case: TX_BYTE
            
            // =========================================================================
            //
            // =========================================================================
            RX_BYTE:   begin
                // byte wait not used for STM mode , for it goto stop when ack bit cleared
                if(byte_waiting_q ==1) begin
                    mst_txfifo_ld_en = 1'b0; //Latch the fifo data to the tx buffer if it has been loaded before
                    master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                    mst_gen_ack_en_s = master_read;//(master_read == 1'b1) ? 1'b1 : 1'b0;
                    //if next byte is RX then gen ack
                end else  begin
                    mst_gen_ack_en_s = 1'b0;
                end
                if(~rw_dw_mode_sync_i) begin
                    // must quick enough before bit 8 : gen ACK/NACK
                    // sync or delayed outside this module
                    mst_gen_ack_en_s = rw_ack_sync_i ; 
                end
                mst_push_rxfifo_en = 1'b0;             
                mst_rx_en = ~byte_wait_scl;

                if (mst_rxbyte_rdy == 1'b1) begin //We recevied the data byte
                    mst_rx_en = 1'b0;
                    mst_gen_ack_en_s = 1'b0;
                    
                    mst_push_rxfifo_en = 1'b1;
                    if(~rw_dw_mode_sync_i) begin
                        // no need fifo ld_en
                        mst_next_state  = rw_stop_sync_i ? GEN_STOP :
                                          rw_start_sync_i ? (ic_10bit_mst_sync ? GEN_RSTRT_10BIT : GEN_RSTRT_7BIT) :
                                          //~rr_rxne_sync_i ? RX_BYTE :  STM_WAIT_TRX ;
                                          STM_WAIT_TRX ;
                        //mst_set_btf_o       = rr_rxne_sync_i | ~rw_ack_sync_i ;
                        mst_set_btf_o       = rr_rxne_sync_i ;
                        mst_set_perr_o      = rw_pec_sync_i ;
                        mst_push_rxfifo_en  = ~rr_rxne_sync_i ; // don't push if not empty
                    end
                    else if(byte_waiting_q == 1'b1) begin
                        if(master_read == 1'b1) begin //byte waiting is read
                            if(tx_empty_sync == 1'b0) begin //We have more data in tx fifo
                                mst_txfifo_ld_en = 1'b1; //Load the fifo data to the tx buffer
                                byte_waiting = 1'b1;
                            end else begin
                                byte_waiting = 1'b0;
                            end

                            mst_gen_ack_en_s = 1'b0;
                            mst_next_state = RX_BYTE;
                        end else begin //Byte waiting is write (change direction)
                            if(ic_rstrt_en_sync == 1'b0)
                              mst_next_state = GEN_SPLIT_STOP;
                            else
                              mst_next_state = (ic_10bit_mst_sync == 1'b0) ? GEN_RSTRT_7BIT
                                               : GEN_RSTRT_10BIT;
                            
                        end // else master_read
                    end else begin //no more data to process
                        mst_rx_en = 1'b0;
                        mst_next_state = GEN_STOP;
                    end
                end // if (mst_rxbyte_rdy == 1'b1)
                else begin
                    byte_waiting = (byte_waiting_q == 1'b1) ? 1'b1 : 1'b0;//preserve the value of the byte waiting flag
                    mst_next_state = RX_BYTE;
                end // else mst_rxbyte_rdy
                
            end // case: RX_BYTE


            TX10_1ST_ADDR:
              begin
                  //master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                  //if ((master_read != old_is_read) && (addr_2byte_sent == 1'b1))//change direction of transfer
                  //  //Set the 1st ADDR Byte data + Read, we are switching dir.
                  //  mst_tx_data_buf_in   = {`IC_SLV_ADDR_10BIT,ic_tar[9:8],master_read}; 
                  //else
                  //  //Set the 1st ADDR Byte data + Write it is a normal addr phase
                  //  mst_tx_data_buf_in   = {`IC_SLV_ADDR_10BIT,ic_tar[9:8],1'b0}; 
                  master_read         = rw_dr_sync_i[0]; // 0: Master is writing, 1: Master is reading
                  mst_tx_data_buf_in  = rw_dr_sync_i;
                  
                  mst_tx_en           = ~byte_wait_scl; //Enable the transmitter

                  if (mst_tx_cmplt == 1'b1)
                    begin
                        if(~rw_dw_mode_sync_i) begin
                            // no matter ack or nack
                            // always to wait_trx, then determine to TX or RX by master_read
                            mst_next_state  = ~addr_2byte_sent ? STM_WAIT_ADDR : STM_WAIT_TRX ; 
                            mst_set_af_o    = ~ack_det ;
                        end
                        else if (ack_det == 1'b1)
                          begin //slave acknowledged the address
                              mst_tx_en = 1'b0;
                              if ((master_read != old_is_read)&&(addr_2byte_sent == 1'b1)&&(master_read == 1'b1))// we are changing the transfer direction
                                begin
                                    if(tx_empty_sync == 1'b0) //We have more data in tx fifo
                                      begin
                                          mst_txfifo_ld_en = 1'b1; //Load the fifo data to the tx buffer
                                          byte_waiting = 1'b1;
                                      end
                                    else
                                      byte_waiting = 1'b0;
                                    
                                    mst_next_state = RX_BYTE;
                                end
                              else
                                mst_next_state = TX10_2ND_ADDR;
                          end
                        
                        else //No Slave acknowledged the address
                          begin
                              mst_tx_abrt = 1'b1;
                              abrt_10addr1_noack = 1'b1;
                              mst_tx_en = 1'b0; //disable the transmitter
                              mst_next_state = GEN_STOP;
                          end 
                    end 
                  else
                    mst_next_state = TX10_1ST_ADDR; // Wait for an ACK signal
              end // case: TX10_1ST_ADDR
            
            TX10_2ND_ADDR:
              begin
                  //master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                  //mst_tx_data_buf_in   = ic_tar[7:0]; //Set the 2nd ADDR Byte data
                  master_read         = rw_dr_sync_i[0];
                  mst_tx_data_buf_in  = rw_dr_sync_i;

                  if((ic_rstrt_en_sync == 1'b0) && (master_read == 1'b1))//if re_start is disabled then you cant read in 10 bit mode
                    begin
                        mst_tx_abrt = 1'b1;
                        abrt_10b_rd_norstrt = 1'b1;
                        
                        mst_next_state = GEN_STOP;//Master is reading from slave  
                    end
                  
                  else
                    begin
                        mst_tx_en = ~byte_wait_scl; //Enable the transmitter
                        
                        if (mst_tx_cmplt == 1'b1)//ack detected
                          begin
                              if(~rw_dw_mode_sync_i) begin
                                  // no matter ack or nack
                                  // always to wait_trx, then determine to TX or RX by master_read
                                  mst_next_state  = STM_WAIT_TRX ; 
                                  mst_set_af_o    = ~ack_det ;
                              end
                              else if (ack_det == 1'b1)//ack detected
                                begin //slave acknowledged the address
                                    mst_tx_en = 1'b0; //disable the transmitter
                                    if(master_read == 1'b0) //Master is writing to slave
                                      mst_next_state = TX_BYTE;
                                    else
                                      begin 
                                          mst_next_state =  GEN_RSTRT_10BIT;//Master is reading from slave
                                      end
                                end
                              
                              else
                                begin
                                    mst_tx_en = 1'b0;
                                    mst_tx_abrt = 1'b1;
                                    abrt_10addr2_noack = 1'b1;
                                    mst_next_state = GEN_STOP;
                                end 
                          end
                        else
                          mst_next_state = TX10_2ND_ADDR; // Wait for an ACK signal
                    end
              end // case: TX10_2ND_ADDR
            
            GEN_RSTRT_HS:
              begin //gen re-start condition to change the direction of the transaction
                  //if(ic_hs_sync == 1'b1)
                  //hs_mcode_en = 1'b1;//Use FS timing if in High Speed Mode
                  
                  if (re_start_cmplt == 1'b1)
                    begin //Re-Start detected go on
                        mst_txfifo_ld_en = 1'b1;// Load the FIFO data into the tx buf
                        mst_next_state = POP_TX_DATA;//Complete the POP TX FIFO process
                    end
                  
                  else begin
                      mst_next_state = GEN_RSTRT_HS;
                  end
              end//case: GEN_RSTRT_HS;

            GEN_RSTRT_SBYTE:
              begin //gen re-start condition to change the direction of the transaction
                  hs_mcode_en = ic_hs_sync;//Use FS timing if in High Speed Mode
                  
                  if (re_start_cmplt == 1'b1)
                    begin //Re-Start detected go on
                        if(ic_hs_sync == 1'b1)
                          mst_next_state = TX_HS_MCODE;//Complete the POP TX FIFO process
                        else
                          begin
                              mst_txfifo_ld_en = 1'b1;// Load the FIFO data into the tx buf
                              mst_next_state = POP_TX_DATA;//Complete the POP TX FIFO process
                          end
                    end
                  
                  else 
                    begin
                        mst_next_state = GEN_RSTRT_SBYTE;
                    end
              end//case: GEN_RSTRT_SBYTE;


            GEN_RSTRT_7BIT:
              begin //gen re-start condition to change the direction of the transaction
                  if (re_start_cmplt == 1'b1)
                    begin //Re-Start detected go on
                        mst_set_sb_o    = 1'b1;
                        //mst_next_state  = TX7_1ST_ADDR;
                        mst_next_state  = ~rw_dw_mode_sync_i ? STM_WAIT_ADDR : TX7_1ST_ADDR;//TX10_1ST_ADDR_RD;
                    end
                  else begin
                      mst_next_state = GEN_RSTRT_7BIT;
                  end
              end//case: GEN_RSTRT_7BIT;

            GEN_RSTRT_10BIT:
              begin //gen re-start condition to change the direction of the transaction
                  if (re_start_cmplt == 1'b1)
                    begin //Re-Start detected go on
                        mst_set_sb_o    = 1'b1;
                        mst_next_state  = ~rw_dw_mode_sync_i ? STM_WAIT_ADDR : TX10_1ST_ADDR;//TX10_1ST_ADDR_RD;
                    end
                  else begin
                      mst_next_state = GEN_RSTRT_10BIT;
                  end
              end//case: GEN_RSTRT_10BIT;

            GEN_SPLIT_STOP:   //used only in SS or FS Mode
              begin
                  stop_en = ~byte_wait_scl; //Enable generating stop condition
                  
                  if (stop_cmplt == 1'b1)
                    begin
                        mst_next_state = ~rw_dw_mode_sync_i ? STM_WAIT_TRX : GEN_SPLIT_START;
                    end
                  else
                    mst_next_state = GEN_SPLIT_STOP;
                  
              end // case: GEN_STOP

            GEN_SPLIT_START://This is only used in SS or FS Mode
              begin //gen stop-start condition to split a combined transaction when re_start is disabled
                  master_read = tx_fifo_data_buf[8]; // 0: Master is writing, 1: Master is reading
                  
                  if (start_cmplt == 1'b1)
                    begin //Start detected, go on

                        if (min_hld_cmplt == 1'b1)//Another master is on the bus 
                          // and The minimum hold time to generate
                          // a start has elapsed
                          // So park on the gen_split_start 
                          // state until 
                          // the bus is idle again
                          mst_next_state = GEN_SPLIT_START;

                        else if(ic_10bit_mst_sync == 1'b0)
                          mst_next_state = TX7_1ST_ADDR;
                        else
                          mst_next_state = TX10_1ST_ADDR; //<HS>Might be redundent
                    end // if (start_cmplt == 1'b1)
                  
                  else
                    begin
                        mst_next_state = GEN_SPLIT_START;
                    end
                  
              end // case: GEN_SPLIT_START
            
            

            GEN_STOP:   
              begin
                  //mst_push_rxfifo_en  = rd_dr_sync_i;
                  if(ic_hs_sync == 1'b1)
                    hs_mcode_en = 1'b1;//Use FS timing if in High Speed Mode
                  
                  if(delay_stop_en)
                    stop_en = 1'd0;
                  else
                    stop_en = ~byte_wait_scl; //Enable generating stop condition
                  
                  if (stop_cmplt == 1'b1) begin
                      mst_next_state = IDLE; // We lost the bus
                  end else
                    mst_next_state = GEN_STOP;
                  
              end // case: GEN_STOP
            
            
            default :
              mst_next_state = IDLE;
            
            
        endcase // case(mst_current_state)
        
    end // block: FSM_COMB_PROC
    
    // =======================================================================
    // Generate "delay_stop_en".
    // This is necessary to ensure that the "stop_hi" signal, inside the TxShift
    // module, does NOT glitch prior to the "stop_lo" signal being asserted.
    // Subsequently, this removes the problem where "ic_data_oe" changes together
    // with "ic_clkc_oe" during Master-Tx.
    // =======================================================================
    always @(posedge ic_clk or negedge ic_rst_n) begin
        if(!ic_rst_n) begin
            delay_stop_en <= 1'd0;
        end else begin
            if(mst_current_state == TX_BYTE      ||
               mst_current_state == TX_HS_MCODE  ||
               mst_current_state == TX7_1ST_ADDR ||
               mst_current_state == TX10_2ND_ADDR ||
               mst_current_state == TX10_1ST_ADDR) 
              delay_stop_en <= 1'd1;
            else
              delay_stop_en <= 1'd0;
        end // else ic_rst_n
    end

    // =======================================================================
    // Generate "mst_gen_ack_en"
    // Forced the original behaviour of the signal to pulse for TWO clock
    // cycles, INSTEAD of the previous ONE.
    // This ensures that the second transition due to the ACK bit is made 1
    // clock cycle after "ic_clk_oe".
    // =======================================================================
    always @(posedge ic_clk or negedge ic_rst_n) begin
        if(!ic_rst_n) begin
            mst_gen_ack_en_r <= 1'd0;
        end else begin
            mst_gen_ack_en_r <= mst_gen_ack_en_s;
        end
    end // always

    assign mst_gen_ack_en = mst_gen_ack_en_r & mst_gen_ack_en_s;

    // ----------------------------------
    // : generate debug signals
    // ----------------------------------
    assign mst_debug_addr = ((mst_current_state == TX7_1ST_ADDR)
                             ||(mst_current_state == TX10_1ST_ADDR)
                             ||(mst_current_state == TX10_2ND_ADDR)
                             ||(mst_current_state == TX_HS_MCODE)
                             ||(mst_current_state == CHECK_IC_TAR));
    
    assign mst_debug_data = ((mst_current_state == TX_BYTE)
                             ||(mst_current_state == RX_BYTE));

    assign mst_debug_cstate = mst_current_state;
    
endmodule // DW_apb_i2c_mstfsm
