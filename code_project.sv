// Code your testbench here
// or browse Examples

`include "zeroriscy_config.sv"

`include "zeroriscy_defines.sv"
`timescale 1ns/1ps

 parameter RV32M      = 1;
parameter OPCODE_SYSTEM    = 7'h73;
parameter OPCODE_FENCE     = 7'h0f;
parameter OPCODE_OP        = 7'h33;
parameter OPCODE_OPIMM     = 7'h13;
parameter OPCODE_STORE     = 7'h23;
parameter OPCODE_LOAD      = 7'h03;
parameter OPCODE_BRANCH    = 7'h63;
parameter OPCODE_JALR      = 7'h67;
parameter OPCODE_JAL       = 7'h6f;
parameter OPCODE_AUIPC     = 7'h17;
parameter OPCODE_LUI       = 7'h37;
parameter OPCODE_MMULT     = 7'h0b;

// those opcodes are now used for PULP custom instructions
// parameter OPCODE_CUST0     = 7'h0b
// parameter OPCODE_CUST1     = 7'h2b

// PULP custom
parameter OPCODE_LOAD_POST  = 7'h0b;
parameter OPCODE_STORE_POST = 7'h2b;
parameter OPCODE_PULP_OP    = 7'h5b;
parameter OPCODE_VECOP      = 7'h57;
parameter OPCODE_HWLOOP     = 7'h7b;

parameter REGC_S1   = 2'b10;
parameter REGC_RD   = 2'b01;
parameter REGC_ZERO = 2'b11;


//////////////////////////////////////////////////////////////////////////////
//      _    _    _   _    ___                       _   _                  //
//     / \  | |  | | | |  / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___  //
//    / _ \ | |  | | | | | | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __| //
//   / ___ \| |__| |_| | | |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \ //
//  /_/   \_\_____\___/   \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/ //
//                             |_|                                          //
//////////////////////////////////////////////////////////////////////////////

parameter ALU_OP_WIDTH = 6;

parameter ALU_ADD   = 6'b011000;
parameter ALU_SUB   = 6'b011001;
parameter ALU_ADDU  = 6'b011010;
parameter ALU_SUBU  = 6'b011011;
parameter ALU_ADDR  = 6'b011100;
parameter ALU_SUBR  = 6'b011101;
parameter ALU_ADDUR = 6'b011110;
parameter ALU_SUBUR = 6'b011111;

parameter ALU_XOR   = 6'b101111;
parameter ALU_OR    = 6'b101110;
parameter ALU_AND   = 6'b010101;

// Shifts
parameter ALU_SRA   = 6'b100100;
parameter ALU_SRL   = 6'b100101;
parameter ALU_ROR   = 6'b100110;
parameter ALU_SLL   = 6'b100111;

// bit manipulation
parameter ALU_BEXT  = 6'b101000;
parameter ALU_BEXTU = 6'b101001;
parameter ALU_BINS  = 6'b101010;
parameter ALU_BCLR  = 6'b101011;
parameter ALU_BSET  = 6'b101100;

// Bit counting
parameter ALU_FF1   = 6'b110110;
parameter ALU_FL1   = 6'b110111;
parameter ALU_CNT   = 6'b110100;
parameter ALU_CLB   = 6'b110101;

// Sign-/zero-extensions
parameter ALU_EXTS  = 6'b111110;
parameter ALU_EXT   = 6'b111111;

// Comparisons
parameter ALU_LTS   = 6'b000000;
parameter ALU_LTU   = 6'b000001;
parameter ALU_LES   = 6'b000100;
parameter ALU_LEU   = 6'b000101;
parameter ALU_GTS   = 6'b001000;
parameter ALU_GTU   = 6'b001001;
parameter ALU_GES   = 6'b001010;
parameter ALU_GEU   = 6'b001011;
parameter ALU_EQ    = 6'b001100;
parameter ALU_NE    = 6'b001101;

// Set Lower Than operations
parameter ALU_SLTS  = 6'b000010;
parameter ALU_SLTU  = 6'b000011;
parameter ALU_SLETS = 6'b000110;
parameter ALU_SLETU = 6'b000111;

// Absolute value
parameter ALU_ABS   = 6'b010100;
parameter ALU_CLIP  = 6'b010110;
parameter ALU_CLIPU = 6'b010111;

// Insert/extract
parameter ALU_INS   = 6'b101101;

// min/max
parameter ALU_MIN   = 6'b010000;
parameter ALU_MINU  = 6'b010001;
parameter ALU_MAX   = 6'b010010;
parameter ALU_MAXU  = 6'b010011;

// div/rem
parameter ALU_DIVU  = 6'b110000; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_DIV   = 6'b110001; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REMU  = 6'b110010; // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REM   = 6'b110011; // bit 0 is used for signed mode, bit 1 is used for remdiv

parameter ALU_SHUF  = 6'b111010;
parameter ALU_SHUF2 = 6'b111011;
parameter ALU_PCKLO = 6'b111000;
parameter ALU_PCKHI = 6'b111001;


parameter MD_OP_MULL  = 2'b00;
parameter MD_OP_MULH  = 2'b01;
parameter MD_OP_DIV   = 2'b10;
parameter MD_OP_REM   = 2'b11;

// vector modes
parameter VEC_MODE32 = 2'b00;
parameter VEC_MODE16 = 2'b10;
parameter VEC_MODE8  = 2'b11;


/////////////////////////////////////////////////////////
//    ____ ____    ____            _     _             //
//   / ___/ ___|  |  _ \ ___  __ _(_)___| |_ ___ _ __  //
//  | |   \___ \  | |_) / _ \/ _` | / __| __/ _ \ '__| //
//  | |___ ___) | |  _ <  __/ (_| | \__ \ ||  __/ |    //
//   \____|____/  |_| \_\___|\__, |_|___/\__\___|_|    //
//                           |___/                     //
/////////////////////////////////////////////////////////

// CSR operations
parameter CSR_OP_NONE  = 2'b00;
parameter CSR_OP_WRITE = 2'b01;
parameter CSR_OP_SET   = 2'b10;
parameter CSR_OP_CLEAR = 2'b11;


// SPR for debugger, not accessible by CPU
parameter SP_DVR0       = 16'h3000;
parameter SP_DCR0       = 16'h3008;
parameter SP_DMR1       = 16'h3010;
parameter SP_DMR2       = 16'h3011;

parameter SP_DVR_MSB = 8'h00;
parameter SP_DCR_MSB = 8'h01;
parameter SP_DMR_MSB = 8'h02;
parameter SP_DSR_MSB = 8'h04;

// Privileged mode
typedef enum logic[1:0] {
  PRIV_LVL_M = 2'b11,
  PRIV_LVL_H = 2'b10,
  PRIV_LVL_S = 2'b01,
  PRIV_LVL_U = 2'b00
} PrivLvl_t;

///////////////////////////////////////////////
//   ___ ____    ____  _                     //
//  |_ _|  _ \  / ___|| |_ __ _  __ _  ___   //
//   | || | | | \___ \| __/ _` |/ _` |/ _ \  //
//   | || |_| |  ___) | || (_| | (_| |  __/  //
//  |___|____/  |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// forwarding operand mux
parameter SEL_REGFILE      = 2'b00;
parameter SEL_FW_EX        = 2'b01;
parameter SEL_FW_WB        = 2'b10;
parameter SEL_MISALIGNED   = 2'b11;

// operand a selection
parameter OP_A_REGA_OR_FWD = 3'b000;
parameter OP_A_CURRPC      = 3'b001;
parameter OP_A_IMM         = 3'b010;
parameter OP_A_REGB_OR_FWD = 3'b011;
parameter OP_A_REGC_OR_FWD = 3'b100;

// immediate a selection
parameter IMMA_Z      = 1'b0;
parameter IMMA_ZERO   = 1'b1;

// operand b selection
parameter OP_B_REGB_OR_FWD = 3'b000;
parameter OP_B_REGC_OR_FWD = 3'b001;
parameter OP_B_IMM         = 3'b010;
parameter OP_B_REGA_OR_FWD = 3'b011;
parameter OP_B_BMASK       = 3'b100;
parameter OP_B_ZERO        = 3'b101;

// immediate b selection
parameter IMMB_I      = 4'b0000;
parameter IMMB_S      = 4'b0001;
parameter IMMB_U      = 4'b0010;
parameter IMMB_PCINCR = 4'b0011;
parameter IMMB_S2     = 4'b0100;
parameter IMMB_S3     = 4'b0101;
parameter IMMB_VS     = 4'b0110;
parameter IMMB_VU     = 4'b0111;
parameter IMMB_SHUF   = 4'b1000;
parameter IMMB_CLIP   = 4'b1001;
parameter IMMB_BI     = 4'b1011;
parameter IMMB_UJ	  = 4'b1100;
parameter IMMB_SB	  = 4'b1101;

// bit mask selection
parameter BMASK_A_ZERO = 1'b0;
parameter BMASK_A_S3   = 1'b1;

parameter BMASK_B_S2   = 2'b00;
parameter BMASK_B_S3   = 2'b01;
parameter BMASK_B_ZERO = 2'b10;
parameter BMASK_B_ONE  = 2'b11;

parameter BMASK_A_REG  = 1'b0;
parameter BMASK_A_IMM  = 1'b1;
parameter BMASK_B_REG  = 1'b0;
parameter BMASK_B_IMM  = 1'b1;



///////////////////////////////////////////////
//   ___ _____   ____  _                     //
//  |_ _|  ___| / ___|| |_ __ _  __ _  ___   //
//   | || |_    \___ \| __/ _` |/ _` |/ _ \  //
//   | ||  _|    ___) | || (_| | (_| |  __/  //
//  |___|_|     |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// PC mux selector defines
parameter PC_BOOT          = 3'b000;
parameter PC_JUMP          = 3'b010;
parameter PC_EXCEPTION     = 3'b100;
parameter PC_ERET          = 3'b101;
parameter PC_DBG_NPC       = 3'b111;

// Exception PC mux selector defines
parameter EXC_PC_ILLINSN   = 2'b00;
parameter EXC_PC_ECALL     = 2'b01;
parameter EXC_PC_LOAD      = 2'b10;
parameter EXC_PC_STORE     = 2'b10;
parameter EXC_PC_IRQ       = 2'b11;

// Exception Cause
parameter EXC_CAUSE_ILLEGAL_INSN = 6'h02;
parameter EXC_CAUSE_BREAKPOINT   = 6'h03;
parameter EXC_CAUSE_ECALL        = 6'h08;

// Exceptions offsets
// target address = {boot_addr[31:8], EXC_OFF} (boot_addr must be 32 BYTE aligned!)
// offset 00 to 7e is used for external interrupts
parameter EXC_OFF_RST      = 8'h80;
parameter EXC_OFF_ILLINSN  = 8'h84;
parameter EXC_OFF_ECALL    = 8'h88;
parameter EXC_OFF_LSUERR   = 8'h8c;


// Debug module
parameter DBG_SETS_W = 6;

parameter DBG_SETS_IRQ    = 5;
parameter DBG_SETS_ECALL  = 4;
parameter DBG_SETS_EILL   = 3;
parameter DBG_SETS_ELSU   = 2;
parameter DBG_SETS_EBRK   = 1;
parameter DBG_SETS_SSTE   = 0;

parameter DBG_CAUSE_HALT   = 6'h1F;

interface decoder_ports;
	logic        deassert_we_i;           // deassert we, we are stalled or not active
    logic        branch_mux_i;
    logic        jump_mux_i;
   logic        illegal_insn_o;          // illegal instruction encountered
   logic        ebrk_insn_o;             // trap instruction encountered
   logic        mret_insn_o;             // return from exception instruction encountered
   logic        ecall_insn_o;            // environment call (syscall) instruction encountered
   logic        pipe_flush_o;            // pipeline flush is requested

  // from IF/ID pipeline
    logic [31:0] instr_rdata_i;           // instruction read from instr memory/cache
    logic        illegal_c_insn_i;       // compressed instruction decode failed
    logic        raw_ra_i;                // RA==WB Wreg
    logic        raw_rb_i;                // RB==WB Wreg
   logic        raw_stall_o;

  // ALU signals
   logic [ALU_OP_WIDTH-1:0] alu_operator_o; // ALU operation selection
   logic [2:0]  alu_op_a_mux_sel_o;      // operand a selection: reg value, PC, immediate or zero
   logic [2:0]  alu_op_b_mux_sel_o;      // oNOperand b selection: reg value or immediate

   logic [0:0]  imm_a_mux_sel_o;         // immediate selection for operand a
   logic [3:0]  imm_b_mux_sel_o;         // immediate selection for operand b

  // MUL, DIV related control signals
   logic        mult_int_en_o;          // perform integer multiplication
   logic        div_int_en_o;           // perform integer division or reminder
   logic [1:0]  multdiv_operator_o;
   logic [1:0]  multdiv_signed_mode_o;

  // MMULT related control signals
   logic        mmult_en_o;             // perform MMULT
   logic [2:0]  mmult_operator_o;
   logic [6:0]  mmult_param_o;

  // register file related signals
   logic        regfile_we_o;           // write enable for regfile

  // CSR manipulation
   logic        csr_access_o;            // access to CSR
   logic [1:0]  csr_op_o;                // operation to perform on CSR
   logic        csr_status_o;           // access to xstatus CSR

  // LD/ST unit signals
   logic        data_req_o;              // start transaction to data memory
   logic        data_we_o;               // data memory write enable
   logic [1:0]  data_type_o;             // data type on data memory: byte, half word or word
   logic        data_sign_extension_o;   // sign extension on read data from data memory
   logic [1:0]  data_reg_offset_o;       // offset in byte inside register for stores
   logic        data_load_event_o;       // data request is in the special event range


  // jump/branches
   logic        jump_in_id_o;            // jump is being calculated in ALU
   logic        branch_in_id_o;
  
  covergroup cover_grp ();
    option.per_instance = 1;
      deassert : coverpoint deassert_we_i;
      branch :	 coverpoint   branch_mux_i;
	jump :	 coverpoint  jump_mux_i;
	instr_rdata : coverpoint instr_rdata_i;
      illegal_ins : coverpoint illegal_c_insn_i;       // compressed instruction decode failed
     raw_ra : coverpoint  raw_ra_i;                
     raw_rb : coverpoint  raw_rb_i;  
      

  endgroup
  
 cover_grp cgrp1 = new();
endinterface

class transaction;

      randc bit deassert_we_i;           // deassert we, we are stalled or not active
  randc bit branch_mux_i;
  randc bit jump_mux_i;
  randc bit [31:0] instr_rdata_i;           // instruction read from instr memory/cache
  randc bit illegal_c_insn_i;        // compressed instruction decode failed
  randc bit raw_ra_i;                // RA==WB Wreg
  randc bit raw_rb_i;    


  bit        illegal_insn_o;          // illegal instruction encountered
  bit        ebrk_insn_o;             // trap instruction encountered
  bit        mret_insn_o;             // return from exception instruction encountered
  bit        ecall_insn_o;            // environment call (syscall) instruction encountered
  bit        pipe_flush_o;            // pipeline flush is requested


bit        raw_stall_o;

  // ALU signals
  bit [ALU_OP_WIDTH-1:0] alu_operator_o; // ALU operation selection
  bit [2:0]  alu_op_a_mux_sel_o;      // operand a selection: reg value; PC; immediate or zero
  bit [2:0]  alu_op_b_mux_sel_o;      // oNOperand b selection: reg value or immediate

  bit [0:0]  imm_a_mux_sel_o;         // immediate selection for operand a
  bit [3:0]  imm_b_mux_sel_o;         // immediate selection for operand b

  // MUL; DIV related control signals
  bit        mult_int_en_o;          // perform integer multiplication
  bit        div_int_en_o;           // perform integer division or reminder
  bit [1:0]  multdiv_operator_o;
  bit [1:0]  multdiv_signed_mode_o;

  // MMULT related control signals
  bit        mmult_en_o;             // perform MMULT
  bit [2:0]  mmult_operator_o;
  bit [6:0]  mmult_param_o;

  // register file related signals
  bit        regfile_we_o;            // write enable for regfile

  // CSR manipulation
  bit        csr_access_o;            // access to CSR
  bit [1:0]  csr_op_o;                // operation to perform on CSR
  bit        csr_status_o;            // access to xstatus CSR

  // LD/ST unit signals
  bit        data_req_o;              // start transaction to data memory
  bit        data_we_o;               // data memory write enable
  bit [1:0]  data_type_o;             // data type on data memory: byte; half word or word
  bit        data_sign_extension_o;   // sign extension on read data from data memory
  bit [1:0]  data_reg_offset_o;       // offset in byte inside register for stores
  bit        data_load_event_o;       // data request is in the special event range


  // jump/branches
  bit        jump_in_id_o;            // jump is being calculated in ALU
  bit        branch_in_id_o;



  
  function void display(input string layer);
    $display("transaction");    
  endfunction
  
  function transaction copy();
    copy = new();
   copy.deassert_we_i = this.deassert_we_i;
copy.branch_mux_i = this.branch_mux_i;
copy.jump_mux_i = this.jump_mux_i;
copy.illegal_insn_o = this.illegal_insn_o;
copy.ebrk_insn_o = this.ebrk_insn_o;
copy.mret_insn_o = this.mret_insn_o;
copy.ecall_insn_o = this.ecall_insn_o;
copy.pipe_flush_o = this.pipe_flush_o;
copy.instr_rdata_i = this.instr_rdata_i;
copy.illegal_c_insn_i = this.illegal_c_insn_i;
copy.raw_ra_i = this.raw_ra_i;
copy.raw_rb_i = this.raw_rb_i;
copy.raw_stall_o = this.raw_stall_o;
copy.alu_operator_o = this.alu_operator_o;
copy.alu_op_a_mux_sel_o = this.alu_op_a_mux_sel_o;
copy.alu_op_b_mux_sel_o = this.alu_op_b_mux_sel_o;
copy.imm_a_mux_sel_o = this.imm_a_mux_sel_o;
copy.imm_b_mux_sel_o = this.imm_b_mux_sel_o;
copy.mult_int_en_o = this.mult_int_en_o;
copy.div_int_en_o = this.div_int_en_o;
copy.multdiv_operator_o = this.multdiv_operator_o;
copy.multdiv_signed_mode_o = this.multdiv_signed_mode_o;
copy.mmult_en_o = this.mmult_en_o;
copy.mmult_operator_o = this.mmult_operator_o;
copy.mmult_param_o = this.mmult_param_o;
copy.regfile_we_o = this.regfile_we_o;
copy.csr_access_o = this.csr_access_o;
copy.csr_op_o = this.csr_op_o;
copy.csr_status_o = this.csr_status_o;
copy.data_req_o = this.data_req_o;
copy.data_we_o = this.data_we_o;
copy.data_type_o = this.data_type_o;
copy.data_sign_extension_o = this.data_sign_extension_o;
copy.data_reg_offset_o = this.data_reg_offset_o;
copy.data_load_event_o = this.data_load_event_o;
copy.jump_in_id_o = this.jump_in_id_o;
copy.branch_in_id_o = this.branch_in_id_o;



  endfunction
endclass







class driver;
  mailbox #(transaction) mbx;
  transaction datac;
  virtual decoder_ports ports_intr;
  
  function new(mailbox #(transaction) mbx);
  	this.mbx = mbx;
  endfunction
  
  /* task reset();
    ports_intr.A <= 0;
    ports_intr.B <= 0;
    ports_intr.Control <= 0;
    
    #10;
  endtask */
  
  task run();
    forever begin
      mbx.get(datac);
    //  datac.display("DRIVER");

ports_intr.deassert_we_i <= datac.deassert_we_i;
ports_intr.branch_mux_i <= datac.branch_mux_i;
ports_intr.jump_mux_i <= datac.jump_mux_i;
ports_intr.instr_rdata_i <= datac.instr_rdata_i;
ports_intr.illegal_c_insn_i <= datac.illegal_c_insn_i;
ports_intr.raw_ra_i <= datac.raw_ra_i;
ports_intr.raw_rb_i <= datac.raw_rb_i;

//      #10;
    end
  endtask
endclass


class generator;
  transaction trans_obj;
  mailbox #(transaction) mbx;
  event cov;
  
  int count = 200;
  event tr_new, done;
  
  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    trans_obj = new();
  endfunction
  
  task run();
    repeat(count) begin
      assert(trans_obj.randomize()) else $error("Unsuccessful Randomization");
      mbx.put(trans_obj.copy);
      -> cov;
      trans_obj.display("GENERATOR");
      #10;
      @tr_new;
    end
    ->done;
  endtask
endclass






class monitor;
  mailbox #(transaction) mbx;
  transaction trans_obj;
  virtual decoder_ports ports_intr;

  
  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
  endfunction
  
  task run();
    trans_obj = new();
    forever begin
      #10;
   trans_obj.deassert_we_i = ports_intr.deassert_we_i;
trans_obj.branch_mux_i = ports_intr.branch_mux_i;
trans_obj.jump_mux_i = ports_intr.jump_mux_i;
trans_obj.illegal_insn_o = ports_intr.illegal_insn_o;
trans_obj.ebrk_insn_o = ports_intr.ebrk_insn_o;
trans_obj.mret_insn_o = ports_intr.mret_insn_o;
trans_obj.ecall_insn_o = ports_intr.ecall_insn_o;
trans_obj.pipe_flush_o = ports_intr.pipe_flush_o;
trans_obj.instr_rdata_i = ports_intr.instr_rdata_i;
trans_obj.illegal_c_insn_i = ports_intr.illegal_c_insn_i;
trans_obj.raw_ra_i = ports_intr.raw_ra_i;
trans_obj.raw_rb_i = ports_intr.raw_rb_i;
trans_obj.raw_stall_o = ports_intr.raw_stall_o;
trans_obj.alu_operator_o = ports_intr.alu_operator_o;
trans_obj.alu_op_a_mux_sel_o = ports_intr.alu_op_a_mux_sel_o;
trans_obj.alu_op_b_mux_sel_o = ports_intr.alu_op_b_mux_sel_o;
trans_obj.imm_a_mux_sel_o = ports_intr.imm_a_mux_sel_o;
trans_obj.imm_b_mux_sel_o = ports_intr.imm_b_mux_sel_o;
trans_obj.mult_int_en_o = ports_intr.mult_int_en_o;
trans_obj.div_int_en_o = ports_intr.div_int_en_o;
trans_obj.multdiv_operator_o = ports_intr.multdiv_operator_o;
trans_obj.multdiv_signed_mode_o = ports_intr.multdiv_signed_mode_o;
trans_obj.mmult_en_o = ports_intr.mmult_en_o;
trans_obj.mmult_operator_o = ports_intr.mmult_operator_o;
trans_obj.mmult_param_o = ports_intr.mmult_param_o;
trans_obj.regfile_we_o = ports_intr.regfile_we_o;
trans_obj.csr_access_o = ports_intr.csr_access_o;
trans_obj.csr_op_o = ports_intr.csr_op_o;
trans_obj.csr_status_o = ports_intr.csr_status_o;
trans_obj.data_req_o = ports_intr.data_req_o;
trans_obj.data_we_o = ports_intr.data_we_o;
trans_obj.data_type_o = ports_intr.data_type_o;
trans_obj.data_sign_extension_o = ports_intr.data_sign_extension_o;
trans_obj.data_reg_offset_o = ports_intr.data_reg_offset_o;
trans_obj.data_load_event_o = ports_intr.data_load_event_o;
trans_obj.jump_in_id_o = ports_intr.jump_in_id_o;
trans_obj.branch_in_id_o = ports_intr.branch_in_id_o;


      mbx.put(trans_obj);
      trans_obj.display("MONITOR");
    end
  endtask
  
endclass


class scoreboard;
  mailbox #(transaction) mbx;
  transaction trans_obj;
  event tr_new;
  // virtual decoder_ports ports_intr;

bit        illegal_insn_o;          // illegal instruction encountered
  bit        ebrk_insn_o;             // trap instruction encountered
  bit        mret_insn_o;             // return from exception instruction encountered
  bit        ecall_insn_o;            // environment call (syscall) instruction encountered
  bit        pipe_flush_o;            // pipeline flush is requested


bit        raw_stall_o;

  // ALU signals
  bit [ALU_OP_WIDTH-1:0] alu_operator_o; // ALU operation selection
  bit [2:0]  alu_op_a_mux_sel_o;      // operand a selection: reg value; PC; immediate or zero
  bit [2:0]  alu_op_b_mux_sel_o;      // oNOperand b selection: reg value or immediate

  bit [0:0]  imm_a_mux_sel_o;         // immediate selection for operand a
  bit [3:0]  imm_b_mux_sel_o;         // immediate selection for operand b

  // MUL; DIV related control signals
  bit        mult_int_en_o;          // perform integer multiplication
  bit        div_int_en_o;           // perform integer division or reminder
  bit [1:0]  multdiv_operator_o;
  bit [1:0]  multdiv_signed_mode_o;

  // MMULT related control signals
  bit        mmult_en_o;             // perform MMULT
  bit [2:0]  mmult_operator_o;
  bit [6:0]  mmult_param_o;

  // register file related signals
  bit        regfile_we_o;            // write enable for regfile

  // CSR manipulation
  bit        csr_access_o;            // access to CSR
  bit [1:0]  csr_op_o;                // operation to perform on CSR
  bit        csr_status_o;            // access to xstatus CSR

  // LD/ST unit signals
  bit        data_req_o;              // start transaction to data memory
  bit        data_we_o;               // data memory write enable
  bit [1:0]  data_type_o;             // data type on data memory: byte; half word or word
  bit        data_sign_extension_o;   // sign extension on read data from data memory
  bit [1:0]  data_reg_offset_o;       // offset in byte inside register for stores
  bit        data_load_event_o;       // data request is in the special event range


  // jump/branches
  bit        jump_in_id_o;            // jump is being calculated in ALU
  bit        branch_in_id_o;


  logic       regfile_we;
  logic       data_req;

  logic       mult_int_en;
  logic       div_int_en;
  logic       mmult_en;
  logic       branch_in_id;
  logic       jump_in_id;

  logic [1:0] csr_op;
  logic       csr_illegal;
   logic  deassert_we;
  
    
task func

(
  // singals running to/from controller
  input  logic        deassert_we_i,           // deassert we, we are stalled or not active
  input  logic        branch_mux_i,
  input  logic        jump_mux_i,
  
  // from IF/ID pipeline
  input  logic [31:0] instr_rdata_i,           // instruction read from instr memory/cache
  input  logic        illegal_c_insn_i,        // compressed instruction decode failed
  input  logic        raw_ra_i,                // RA==WB Wreg
  input  logic        raw_rb_i                // RB==WB Wreg

  // ALU signals
  );

  // write enable/request control


  /////////////////////////////////////////////
  //   ____                     _            //
  //  |  _ \  ___  ___ ___   __| | ___ _ __  //
  //  | | | |/ _ \/ __/ _ \ / _` |/ _ \ '__| //
  //  | |_| |  __/ (_| (_) | (_| |  __/ |    //
  //  |____/ \___|\___\___/ \__,_|\___|_|    //
  //                                         //
  /////////////////////////////////////////////

  
  begin
    jump_in_id                  = 1'b0;
    branch_in_id                = 1'b0;
    alu_operator_o              = ALU_SLTU;
    alu_op_a_mux_sel_o          = OP_A_REGA_OR_FWD;
    alu_op_b_mux_sel_o          = OP_B_REGB_OR_FWD;

    imm_a_mux_sel_o             = IMMA_ZERO;
    imm_b_mux_sel_o             = IMMB_I;

    mult_int_en                 = 1'b0;
    div_int_en                  = 1'b0;
    multdiv_operator_o          = MD_OP_MULL;
    multdiv_signed_mode_o       = 2'b00;

    mmult_en                    = 1'b0;
    mmult_operator_o            = 3'b000;
    mmult_param_o               = 7'b0000000;

    regfile_we                  = 1'b0;

    csr_access_o                = 1'b0;
    csr_status_o                = 1'b0;
    csr_illegal                 = 1'b0;
    csr_op                      = CSR_OP_NONE;

    data_we_o                   = 1'b0;
    data_type_o                 = 2'b00;
    data_sign_extension_o       = 1'b0;
    data_reg_offset_o           = 2'b00;
    data_req                    = 1'b0;
    data_load_event_o           = 1'b0;

    illegal_insn_o              = 1'b0;
    ebrk_insn_o                 = 1'b0;
    mret_insn_o                 = 1'b0;
    ecall_insn_o                = 1'b0;
    pipe_flush_o                = 1'b0;

    unique case (instr_rdata_i[6:0])

      //////////////////////////////////////
      //      _ _   _ __  __ ____  ____   //
      //     | | | | |  \/  |  _ \/ ___|  //
      //  _  | | | | | |\/| | |_) \___ \  //
      // | |_| | |_| | |  | |  __/ ___) | //
      //  \___/ \___/|_|  |_|_|   |____/  //
      //                                  //
      //////////////////////////////////////

      OPCODE_JAL: begin   // Jump and Link
        jump_in_id            = 1'b1;
        if(jump_mux_i) begin
          // Calculate jump target
          alu_op_a_mux_sel_o  = OP_A_CURRPC;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_b_mux_sel_o     = IMMB_UJ;
          alu_operator_o      = ALU_ADD;
          regfile_we          = 1'b0;
        end else begin
          // Calculate and store PC+4
          alu_op_a_mux_sel_o  = OP_A_CURRPC;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_b_mux_sel_o     = IMMB_PCINCR;
          alu_operator_o      = ALU_ADD;
          regfile_we          = 1'b1;
        end
      end

      OPCODE_JALR: begin  // Jump and Link Register
        jump_in_id            = 1'b1;
        if(jump_mux_i) begin
          // Calculate jump target
          alu_op_a_mux_sel_o  = OP_A_REGA_OR_FWD;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_b_mux_sel_o     = IMMB_I;
          alu_operator_o      = ALU_ADD;
          regfile_we          = 1'b0;
        end else begin
          // Calculate and store PC+4
          alu_op_a_mux_sel_o  = OP_A_CURRPC;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_b_mux_sel_o     = IMMB_PCINCR;
          alu_operator_o      = ALU_ADD;
          regfile_we          = 1'b1;
        end
        if (instr_rdata_i[14:12] != 3'b0) begin
          jump_in_id       = 1'b0;
          regfile_we       = 1'b0;
          illegal_insn_o   = 1'b1;
        end

      end

      OPCODE_BRANCH: begin // Branch

        branch_in_id          = 1'b1;

        if (branch_mux_i)
        begin
          unique case (instr_rdata_i[14:12])
            3'b000: alu_operator_o = ALU_EQ;
            3'b001: alu_operator_o = ALU_NE;
            3'b100: alu_operator_o = ALU_LTS;
            3'b101: alu_operator_o = ALU_GES;
            3'b110: alu_operator_o = ALU_LTU;
            3'b111: alu_operator_o = ALU_GEU;
            default: begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end
        else begin
          // Calculate jump target in EX
          alu_op_a_mux_sel_o  = OP_A_CURRPC;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_b_mux_sel_o     = IMMB_SB;
          alu_operator_o      = ALU_ADD;
          regfile_we          = 1'b0;
        end
        
      end


      //////////////////////////////////
      //  _     ____    ______ _____  //
      // | |   |  _ \  / / ___|_   _| //
      // | |   | | | |/ /\___ \ | |   //
      // | |___| |_| / /  ___) || |   //
      // |_____|____/_/  |____/ |_|   //
      //                              //
      //////////////////////////////////

      OPCODE_STORE: begin
        data_req       = 1'b1;
        data_we_o      = 1'b1;
        alu_operator_o = ALU_ADD;

        if (instr_rdata_i[14] == 1'b0) begin
          // offset from immediate
          imm_b_mux_sel_o     = IMMB_S;
          alu_op_b_mux_sel_o  = OP_B_IMM;
        end
        // Register offset is illegal since no register c available
        else begin
          data_req       = 1'b0;
          data_we_o      = 1'b0;
          illegal_insn_o = 1'b1;
        end

        // store size
        unique case (instr_rdata_i[13:12])
          2'b00: data_type_o = 2'b10; // SB
          2'b01: data_type_o = 2'b01; // SH
          2'b10: data_type_o = 2'b00; // SW
          default: begin
            data_req       = 1'b0;
            data_we_o      = 1'b0;
            illegal_insn_o = 1'b1;
          end
        endcase
      end

      OPCODE_LOAD: begin
        data_req        = 1'b1;
        regfile_we      = 1'b1;
        data_type_o     = 2'b00;

        // offset from immediate
        alu_operator_o      = ALU_ADD;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_I;


        // sign/zero extension
        data_sign_extension_o = ~instr_rdata_i[14];

        // load size
        unique case (instr_rdata_i[13:12])
          2'b00:   data_type_o = 2'b10; // LB
          2'b01:   data_type_o = 2'b01; // LH
          2'b10:   data_type_o = 2'b00; // LW
          default: data_type_o = 2'b00; // illegal or reg-reg
        endcase

        // reg-reg load (different encoding)
        if (instr_rdata_i[14:12] == 3'b111) begin
          // offset from RS2
          alu_op_b_mux_sel_o = OP_B_REGB_OR_FWD;

          // sign/zero extension
          data_sign_extension_o = ~instr_rdata_i[30];

          // load size
          unique case (instr_rdata_i[31:25])
            7'b0000_000,
            7'b0100_000: data_type_o = 2'b10; // LB, LBU
            7'b0001_000,
            7'b0101_000: data_type_o = 2'b01; // LH, LHU
            7'b0010_000: data_type_o = 2'b00; // LW
            default: begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end

        // special p.elw (event load)
        if (instr_rdata_i[14:12] == 3'b110)
          data_load_event_o = 1'b1;

        if (instr_rdata_i[14:12] == 3'b011) begin
          // LD -> RV64 only
          illegal_insn_o = 1'b1;
        end
      end


      //////////////////////////
      //     _    _    _   _  //
      //    / \  | |  | | | | //
      //   / _ \ | |  | | | | //
      //  / ___ \| |__| |_| | //
      // /_/   \_\_____\___/  //
      //                      //
      //////////////////////////

      OPCODE_LUI: begin  // Load Upper Immediate
        alu_op_a_mux_sel_o  = OP_A_IMM;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_a_mux_sel_o     = IMMA_ZERO;
        imm_b_mux_sel_o     = IMMB_U;
        alu_operator_o      = ALU_ADD;
        regfile_we          = 1'b1;
      end

      OPCODE_AUIPC: begin  // Add Upper Immediate to PC
        alu_op_a_mux_sel_o  = OP_A_CURRPC;
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_U;
        alu_operator_o      = ALU_ADD;
        regfile_we          = 1'b1;
      end

      OPCODE_OPIMM: begin // Register-Immediate ALU Operations
        alu_op_b_mux_sel_o  = OP_B_IMM;
        imm_b_mux_sel_o     = IMMB_I;
        regfile_we          = 1'b1;

        unique case (instr_rdata_i[14:12])
          3'b000: alu_operator_o = ALU_ADD;  // Add Immediate
          3'b010: alu_operator_o = ALU_SLTS; // Set to one if Lower Than Immediate
          3'b011: alu_operator_o = ALU_SLTU; // Set to one if Lower Than Immediate Unsigned
          3'b100: alu_operator_o = ALU_XOR;  // Exclusive Or with Immediate
          3'b110: alu_operator_o = ALU_OR;   // Or with Immediate
          3'b111: alu_operator_o = ALU_AND;  // And with Immediate

          3'b001: begin
            alu_operator_o = ALU_SLL;  // Shift Left Logical by Immediate
            if (instr_rdata_i[31:25] != 7'b0)
              illegal_insn_o = 1'b1;
          end

          3'b101: begin
            if (instr_rdata_i[31:25] == 7'b0)
              alu_operator_o = ALU_SRL;  // Shift Right Logical by Immediate
            else if (instr_rdata_i[31:25] == 7'b010_0000)
              alu_operator_o = ALU_SRA;  // Shift Right Arithmetically by Immediate
            else
              illegal_insn_o = 1'b1;
          end

          default: illegal_insn_o = 1'b1;
        endcase
      end

      OPCODE_OP: begin  // Register-Register ALU operation
        regfile_we     = 1'b1;

        if (instr_rdata_i[31]) begin
          illegal_insn_o = 1'b1;
        end
        else
        begin // non bit-manipulation instructions

          if (~instr_rdata_i[28])

          unique case ({instr_rdata_i[30:25], instr_rdata_i[14:12]})
            // RV32I ALU operations
            {6'b00_0000, 3'b000}: alu_operator_o = ALU_ADD;   // Add
            {6'b10_0000, 3'b000}: alu_operator_o = ALU_SUB;   // Sub
            {6'b00_0000, 3'b010}: alu_operator_o = ALU_SLTS;  // Set Lower Than
            {6'b00_0000, 3'b011}: alu_operator_o = ALU_SLTU;  // Set Lower Than Unsigned
            {6'b00_0000, 3'b100}: alu_operator_o = ALU_XOR;   // Xor
            {6'b00_0000, 3'b110}: alu_operator_o = ALU_OR;    // Or
            {6'b00_0000, 3'b111}: alu_operator_o = ALU_AND;   // And
            {6'b00_0000, 3'b001}: alu_operator_o = ALU_SLL;   // Shift Left Logical
            {6'b00_0000, 3'b101}: alu_operator_o = ALU_SRL;   // Shift Right Logical
            {6'b10_0000, 3'b101}: alu_operator_o = ALU_SRA;   // Shift Right Arithmetic

            // supported RV32M instructions
            {6'b00_0001, 3'b000}: begin // mul
                alu_operator_o        = ALU_ADD;
                multdiv_operator_o    = MD_OP_MULL;
                mult_int_en           = 1'b1;
                multdiv_signed_mode_o = 2'b00;
                illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b001}: begin // mulh
                alu_operator_o        = ALU_ADD;
                multdiv_operator_o    = MD_OP_MULH;
                mult_int_en           = 1'b1;
                multdiv_signed_mode_o = 2'b11;
                illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b010}: begin // mulhsu
                alu_operator_o        = ALU_ADD;
                multdiv_operator_o    = MD_OP_MULH;
                mult_int_en           = 1'b1;
                multdiv_signed_mode_o = 2'b01;
                illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b011}: begin // mulhu
                alu_operator_o        = ALU_ADD;
                multdiv_operator_o    = MD_OP_MULH;
                mult_int_en           = 1'b1;
                multdiv_signed_mode_o = 2'b00;
                illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b100}: begin // div
              alu_operator_o        = ALU_ADD;
              multdiv_operator_o    = MD_OP_DIV;
              div_int_en            = 1'b1;
              multdiv_signed_mode_o = 2'b11;
              illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b101}: begin // divu
              alu_operator_o        = ALU_ADD;
              multdiv_operator_o    = MD_OP_DIV;
              div_int_en            = 1'b1;
              multdiv_signed_mode_o = 2'b00;
              illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b110}: begin // rem
              alu_operator_o        = ALU_ADD;
              multdiv_operator_o    = MD_OP_REM;
              div_int_en            = 1'b1;
              multdiv_signed_mode_o = 2'b11;
              illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            {6'b00_0001, 3'b111}: begin // remu
              alu_operator_o        = ALU_ADD;
              multdiv_operator_o    = MD_OP_REM;
              div_int_en            = 1'b1;
              multdiv_signed_mode_o = 2'b00;
              illegal_insn_o        = RV32M ? 1'b0 : 1'b1;
            end
            default: begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end
      end




      ////////////////////////////////////////////////
      //  ____  ____  _____ ____ ___    _    _      //
      // / ___||  _ \| ____/ ___|_ _|  / \  | |     //
      // \___ \| |_) |  _|| |    | |  / _ \ | |     //
      //  ___) |  __/| |__| |___ | | / ___ \| |___  //
      // |____/|_|   |_____\____|___/_/   \_\_____| //
      //                                            //
      ////////////////////////////////////////////////

      OPCODE_SYSTEM: begin
        if (instr_rdata_i[14:12] == 3'b000)
        begin
          // non CSR related SYSTEM instructions
          unique case (instr_rdata_i[31:20])
            12'h000:  // ECALL
            begin
              // environment (system) call
              ecall_insn_o = 1'b1;
            end

            12'h001:  // ebreak
            begin
              // debugger trap
              ebrk_insn_o = 1'b1;
            end

            12'h302:  // mret
            begin
              mret_insn_o = 1'b1;
            end

            12'h105:  // wfi
            begin
              // flush pipeline
              pipe_flush_o = 1'b1;
            end

            default:
            begin
              illegal_insn_o = 1'b1;
            end
          endcase
        end
        else
        begin
          // instruction to read/modify CSR
          csr_access_o        = 1'b1;
          regfile_we          = 1'b1;
          alu_op_b_mux_sel_o  = OP_B_IMM;
          imm_a_mux_sel_o     = IMMA_Z;
          imm_b_mux_sel_o     = IMMB_I;    // CSR address is encoded in I imm

          if (instr_rdata_i[14] == 1'b1) begin
            // rs1 field is used as immediate
            alu_op_a_mux_sel_o = OP_A_IMM;
          end else begin
            alu_op_a_mux_sel_o = OP_A_REGA_OR_FWD;
          end

          unique case (instr_rdata_i[13:12])
            2'b01:   csr_op   = CSR_OP_WRITE;
            2'b10:   csr_op   = CSR_OP_SET;
            2'b11:   csr_op   = CSR_OP_CLEAR;
            default: csr_illegal = 1'b1;
          endcase

          if(~csr_illegal)
            if (instr_rdata_i[31:20] == 12'h300)
              //access to mstatus
              csr_status_o = 1'b1;

          illegal_insn_o = csr_illegal;

        end

      end
      OPCODE_FENCE: begin
         unique case (instr_rdata_i[14:12])
           3'b000:  // FENCE
             begin
             end

           3'b001:  // FENCEI
             begin
                // flush pipeline
                pipe_flush_o = 1'b1;
             end

           default:
             begin
                illegal_insn_o = 1'b1;
             end
         endcase
      end

      //////////////////////////////////////
      //   __  __ __  __ _   _ _   _____  //
      //  |  \/  |  \/  | | | | | |_   _| //
      //  | |\/| | |\/| | | | | |   | |   //
      //  | |  | | |  | | |_| | |___| |   //
      //  |_|  |_|_|  |_|\___/|_____|_|   //
      //                                  //
      //////////////////////////////////////
      OPCODE_MMULT: begin  // MMULT operation
         unique case (instr_rdata_i[14:12])
           3'b101:begin // mmult32
              mmult_en = 1'b1;
              mmult_operator_o = 3'b101;
              mmult_param_o = instr_rdata_i[31:25];
              regfile_we = 1'b1;
           end
           default: begin
              illegal_insn_o = 1'b1;
           end
         endcase
      end
      default: begin
        illegal_insn_o = 1'b1;
      end
    endcase

    // make sure invalid compressed instruction causes an exception
    if (illegal_c_insn_i) begin
      illegal_insn_o = 1'b1;
    end
  




   raw_stall_o <= (raw_ra_i & (alu_op_a_mux_sel_o==OP_A_REGA_OR_FWD)|
                        raw_rb_i & (alu_op_b_mux_sel_o==OP_B_REGB_OR_FWD)|
                        raw_rb_i & (data_we_o&data_req)                  );

   deassert_we <= deassert_we_i | raw_stall_o;

  // deassert we signals (in case of stalls)
   regfile_we_o      <= (deassert_we) ? 1'b0          : regfile_we;
   mult_int_en_o     <= RV32M ? ((deassert_we) ? 1'b0 : mult_int_en) : 1'b0;
   div_int_en_o      <= RV32M ? ((deassert_we) ? 1'b0 : div_int_en ) : 1'b0;
   mmult_en_o        <= (deassert_we) ? 1'b0          : mmult_en;
   data_req_o       <= (deassert_we) ? 1'b0          : data_req;
   csr_op_o          <= (deassert_we) ? CSR_OP_NONE   : csr_op;
   jump_in_id_o      <= (deassert_we) ? 1'b0          : jump_in_id;
   branch_in_id_o    <= (deassert_we) ? 1'b0          : branch_in_id;
end
    
  endtask
  
  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
  endfunction
  
  task run();
    forever begin
      mbx.get(trans_obj);
      trans_obj.display("SCOREBOARD");
      
      func(.deassert_we_i(trans_obj.deassert_we_i),
.branch_mux_i(trans_obj.branch_mux_i),
.jump_mux_i(trans_obj.jump_mux_i),
.instr_rdata_i(trans_obj.instr_rdata_i),
.illegal_c_insn_i(trans_obj.illegal_c_insn_i),
.raw_ra_i(trans_obj.raw_ra_i),
.raw_rb_i(trans_obj.raw_rb_i));


      
      if(
trans_obj.illegal_insn_o != illegal_insn_o |
trans_obj.ebrk_insn_o != ebrk_insn_o |
trans_obj.mret_insn_o != mret_insn_o |
trans_obj.ecall_insn_o != ecall_insn_o |
trans_obj.pipe_flush_o != pipe_flush_o |
trans_obj.raw_stall_o != raw_stall_o |
trans_obj.alu_operator_o != alu_operator_o |
trans_obj.alu_op_a_mux_sel_o != alu_op_a_mux_sel_o |
trans_obj.alu_op_b_mux_sel_o != alu_op_b_mux_sel_o |
trans_obj.imm_a_mux_sel_o != imm_a_mux_sel_o |
trans_obj.imm_b_mux_sel_o != imm_b_mux_sel_o |
trans_obj.mult_int_en_o != mult_int_en_o |
trans_obj.div_int_en_o != div_int_en_o |
trans_obj.multdiv_operator_o != multdiv_operator_o |
trans_obj.multdiv_signed_mode_o != multdiv_signed_mode_o |
trans_obj.mmult_en_o != mmult_en_o |
trans_obj.mmult_operator_o != mmult_operator_o |
trans_obj.mmult_param_o != mmult_param_o |
trans_obj.regfile_we_o != regfile_we_o |
trans_obj.csr_access_o != csr_access_o |
trans_obj.csr_op_o != csr_op_o |
trans_obj.csr_status_o != csr_status_o |
trans_obj.data_req_o != data_req_o |
trans_obj.data_we_o != data_we_o |
trans_obj.data_type_o != data_type_o |
trans_obj.data_sign_extension_o != data_sign_extension_o |
trans_obj.data_reg_offset_o != data_reg_offset_o |
trans_obj.data_load_event_o != data_load_event_o |
trans_obj.jump_in_id_o != jump_in_id_o |
trans_obj.branch_in_id_o != branch_in_id_o 


)
        begin
          $error("ERROR @ %0t", $time);

        end
      
      else begin
        $display("PASSED");
      end
      ->tr_new;
    end
  endtask
endclass



class environment;
  generator gen;
  driver drv;
  mailbox #(transaction) gdmbx;
  
  monitor mon;
  scoreboard sco;
  mailbox #(transaction) msmbx;
  
  virtual decoder_ports ports_intr;
  
  event tr_new_pass;
  
  function new(virtual decoder_ports ports_intr);
    gdmbx = new();
    gen = new(gdmbx);
    drv = new(gdmbx);
    
    msmbx = new();
    mon = new(msmbx);
    sco = new(msmbx);
    
    this.ports_intr = ports_intr;
    
    drv.ports_intr = ports_intr;
    mon.ports_intr = ports_intr;
    
    gen.tr_new = tr_new_pass;
    sco.tr_new = tr_new_pass;
  endfunction
  
 // task test_rst();
   // drv.reset();
 // endtask
  
  task test();
    fork
      gen.run();
      mon.run();
      sco.run();
      drv.run();
      
    begin
     @gen.cov ;
     ports_intr.cgrp1.sample();
     $display("Sample Collected");
      end
    
    join_any
  endtask
  
  task end_result();
    wait(gen.done.triggered);
    $display("Coverage obtained : %0.2f ", ports_intr.cgrp1.get_inst_coverage());
    #10;
    $finish();
  endtask
  
  
  task run();
  //  test_rst();
    test();
   end_result();
  endtask
endclass


module tb();
  environment env;
  decoder_ports ports_intr();
  
  zeroriscy_decoder dut(.deassert_we_i(ports_intr.deassert_we_i),
.branch_mux_i(ports_intr.branch_mux_i),
.jump_mux_i(ports_intr.jump_mux_i),
.illegal_insn_o(ports_intr.illegal_insn_o),
.ebrk_insn_o(ports_intr.ebrk_insn_o),
.mret_insn_o(ports_intr.mret_insn_o),
.ecall_insn_o(ports_intr.ecall_insn_o),
.pipe_flush_o(ports_intr.pipe_flush_o),
.instr_rdata_i(ports_intr.instr_rdata_i),
.illegal_c_insn_i(ports_intr.illegal_c_insn_i),
.raw_ra_i(ports_intr.raw_ra_i),
.raw_rb_i(ports_intr.raw_rb_i),
.raw_stall_o(ports_intr.raw_stall_o),
.alu_operator_o(ports_intr.alu_operator_o),
.alu_op_a_mux_sel_o(ports_intr.alu_op_a_mux_sel_o),
.alu_op_b_mux_sel_o(ports_intr.alu_op_b_mux_sel_o),
.imm_a_mux_sel_o(ports_intr.imm_a_mux_sel_o),
.imm_b_mux_sel_o(ports_intr.imm_b_mux_sel_o),
.mult_int_en_o(ports_intr.mult_int_en_o),
.div_int_en_o(ports_intr.div_int_en_o),
.multdiv_operator_o(ports_intr.multdiv_operator_o),
.multdiv_signed_mode_o(ports_intr.multdiv_signed_mode_o),
.mmult_en_o(ports_intr.mmult_en_o),
.mmult_operator_o(ports_intr.mmult_operator_o),
.mmult_param_o(ports_intr.mmult_param_o),
.regfile_we_o(ports_intr.regfile_we_o),
.csr_access_o(ports_intr.csr_access_o),
.csr_op_o(ports_intr.csr_op_o),
.csr_status_o(ports_intr.csr_status_o),
.data_req_o(ports_intr.data_req_o),
.data_we_o(ports_intr.data_we_o),
.data_type_o(ports_intr.data_type_o),
.data_sign_extension_o(ports_intr.data_sign_extension_o),
.data_reg_offset_o(ports_intr.data_reg_offset_o),
.data_load_event_o(ports_intr.data_load_event_o),
.jump_in_id_o(ports_intr.jump_in_id_o),
.branch_in_id_o(ports_intr.branch_in_id_o)
);

  initial begin
  //  ports_intr.coverage_trig = 0;
    env = new(ports_intr);
    env.gen.count = 40000;
    env.run();
  end

 // always #5 ports_intr.coverage_trig <= ~ports_intr.coverage_trig;

endmodule