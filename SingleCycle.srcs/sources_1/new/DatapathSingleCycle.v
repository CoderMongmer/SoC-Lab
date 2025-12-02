`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31

// RV opcodes are 7 bits
`define OPCODE_SIZE 6

// Don't forget your previous ALUs
//`include "divider_unsigned.v"
//`include "cla.v"

module RegFile (
    // Write Port
    input                   clk,
    input                   rst,
    input                   we,          // Write Enable
    input      [        4:0] rd,          // Destination register address
    input      [`REG_SIZE:0] rd_data,     // Data to write
    input      [        4:0] rs1,         // Source register 1 address
    output     [`REG_SIZE:0] rs1_data,    // Data from rs1
    input      [        4:0] rs2,         // Source register 2 address
    output     [`REG_SIZE:0] rs2_data     // Data from rs2

);
    reg [`REG_SIZE:0] regs[31:0];
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            // On reset, clear all registers to zero.
            for (i = 0; i < 32; i = i + 1) begin
                regs[i] <= 32'b0;
            end
        end else begin
            // On a normal clock edge, check if a write is enabled.
            // The 'rd != 0' check ensures we don't write to the zero register.
            if (we && rd != 5'b0) begin
                regs[rd] <= rd_data;
            end
        end
    end

    assign rs1_data = (rs1 == 5'b0) ? 32'b0 : regs[rs1];
    assign rs2_data = (rs2 == 5'b0) ? 32'b0 : regs[rs2];

endmodule

module DatapathSingleCycle (
    input                    clk,
    input                    rst,
    output reg               halt,
    output      [`REG_SIZE:0] pc_to_imem,
    input       [`REG_SIZE:0] inst_from_imem,
    output reg  [`REG_SIZE:0] addr_to_dmem,
    input       [`REG_SIZE:0] load_data_from_dmem,
    output reg  [`REG_SIZE:0] store_data_to_dmem,
    output reg  [        3:0] store_we_to_dmem
);

  // Instruction Decoding
  wire [6:0] inst_funct7;
  wire [4:0] inst_rs2, inst_rs1, inst_rd;
  wire [2:0] inst_funct3;
  wire [`OPCODE_SIZE:0] inst_opcode;
  assign {inst_funct7, inst_rs2, inst_rs1, inst_funct3, inst_rd, inst_opcode} = inst_from_imem;

  // Immediates
  wire [11:0] imm_i = inst_from_imem[31:20];
  wire [11:0] imm_s = {inst_funct7, inst_rd};
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:1], imm_b[11], imm_b[0]} = {inst_funct7, inst_rd, 1'b0};
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {inst_from_imem[31:12], 1'b0};
  wire [19:0] imm_u = inst_from_imem[31:12];

  // Sign Extension
  wire [`REG_SIZE:0] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE:0] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE:0] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE:0] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // Opcode Parameters
  localparam [`OPCODE_SIZE:0] OpLoad=7'b0000011, OpStore=7'b0100011, OpBranch=7'b1100011, 
                              OpJalr=7'b1100111, OpMiscMem=7'b0001111, OpJal=7'b1101111, 
                              OpRegImm=7'b0010011, OpRegReg=7'b0110011, OpEnviron=7'b1110011, 
                              OpAuipc=7'b0010111, OpLui=7'b0110111;

  // Flags
  wire inst_jal = (inst_opcode == OpJal);
  wire inst_jalr = (inst_opcode == OpJalr);
  wire inst_beq  = (inst_opcode == OpBranch) & (inst_funct3 == 3'b000);
  wire inst_bne  = (inst_opcode == OpBranch) & (inst_funct3 == 3'b001);
  wire inst_blt  = (inst_opcode == OpBranch) & (inst_funct3 == 3'b100);
  wire inst_bge  = (inst_opcode == OpBranch) & (inst_funct3 == 3'b101);
  wire inst_bltu = (inst_opcode == OpBranch) & (inst_funct3 == 3'b110);
  wire inst_bgeu = (inst_opcode == OpBranch) & (inst_funct3 == 3'b111);
  wire inst_ecall = (inst_opcode == OpEnviron) & (inst_from_imem[31:7] == 25'd0);

  // PC Logic
  reg [`REG_SIZE:0] pcCurrent, pcNext;
  always @(posedge clk) begin
    if (rst) pcCurrent <= 32'd0;
    else pcCurrent <= pcNext;
  end
  assign pc_to_imem = pcCurrent;

  // Register File
  wire [`REG_SIZE:0] rs1_data, rs2_data;
  wire reg_write_enable;
  wire [`REG_SIZE:0] reg_write_data;
  RegFile rf (.clk(clk), .rst(rst), .we(reg_write_enable), .rd(inst_rd), .rd_data(reg_write_data), .rs1(inst_rs1), .rs2(inst_rs2), .rs1_data(rs1_data), .rs2_data(rs2_data));

  // CLA
  reg [`REG_SIZE:0] alu_op1, alu_op2;
  reg [`REG_SIZE:0] alu_result;
  wire is_sub = (inst_opcode == OpRegReg && inst_funct3 == 3'b000 && inst_funct7[5]) || (inst_opcode == OpBranch);
  wire [`REG_SIZE:0] cla_b_operand = is_sub ? ~alu_op2 : alu_op2;
  wire cla_cin = is_sub;
  wire [`REG_SIZE:0] cla_sum;
  cla cla_instance (.a(alu_op1), .b(cla_b_operand), .cin(cla_cin), .sum(cla_sum));

  // M-Extension
  wire signed_mul_op1 = (inst_funct3 == 3'b001) || (inst_funct3 == 3'b010);
  wire signed_mul_op2 = (inst_funct3 == 3'b001);
  wire signed [63:0] mul_op1_ext = signed_mul_op1 ? {{32{rs1_data[31]}}, rs1_data} : {32'b0, rs1_data};
  wire signed [63:0] mul_op2_ext = signed_mul_op2 ? {{32{rs2_data[31]}}, rs2_data} : {32'b0, rs2_data};
  wire [63:0] mul_full_result = mul_op1_ext * mul_op2_ext;

  wire is_signed_div = (inst_funct3 == 3'b100) || (inst_funct3 == 3'b110);
  wire [31:0] div_abs_op1 = (is_signed_div && rs1_data[31]) ? (~rs1_data + 1) : rs1_data;
  wire [31:0] div_abs_op2 = (is_signed_div && rs2_data[31]) ? (~rs2_data + 1) : rs2_data;
  wire [31:0] div_quotient_raw, div_remainder_raw;
  divider_unsigned u_div (.i_dividend(div_abs_op1), .i_divisor(div_abs_op2), .o_quotient(div_quotient_raw), .o_remainder(div_remainder_raw));
  
  wire quotient_is_neg  = is_signed_div && (rs1_data[31] != rs2_data[31]);
  wire remainder_is_neg = is_signed_div && rs1_data[31];
  wire [31:0] div_quotient_final  = quotient_is_neg  ? (~div_quotient_raw + 1)  : div_quotient_raw;
  wire [31:0] div_remainder_final = remainder_is_neg ? (~div_remainder_raw + 1) : div_remainder_raw;

  // Control & ALU
  wire effective_funct7_5;
  assign effective_funct7_5 = (inst_opcode == OpRegImm) ? (((inst_funct3 == 3'b001) || (inst_funct3 == 3'b101)) ? inst_funct7[5] : 1'b0) : inst_funct7[5];

  reg illegal_inst;
  reg reg_write_enable_reg;
  reg [`REG_SIZE:0] reg_write_data_reg;
  assign reg_write_enable = reg_write_enable_reg;
  assign reg_write_data = reg_write_data_reg;
  reg [`REG_SIZE:0] load_val_shifted;

  always @(*) begin
    alu_op1 = rs1_data;
    alu_op2 = rs2_data;
    reg_write_enable_reg = 1'b0;
    illegal_inst = 1'b0;
    halt = 1'b0;
    store_we_to_dmem = 4'b0000;
    store_data_to_dmem = 32'b0;

    case (inst_opcode)
        OpLui:    reg_write_enable_reg = 1'b1;
        OpAuipc:  begin reg_write_enable_reg = 1'b1; alu_op1 = pcCurrent; alu_op2 = imm_u << 12; end
        OpRegImm: begin reg_write_enable_reg = 1'b1; alu_op2 = imm_i_sext; end
        OpRegReg: reg_write_enable_reg = 1'b1;
        OpLoad:   begin reg_write_enable_reg = 1'b1; alu_op2 = imm_i_sext; end
        OpStore:  alu_op2 = imm_s_sext;
        OpJal, OpJalr: reg_write_enable_reg = 1'b1;
    endcase

    alu_result = 32'b0;
    if (inst_opcode == OpLui) alu_result = {imm_u, 12'b0};
    else if (inst_opcode == OpLoad || inst_opcode == OpStore) alu_result = cla_sum;
    else if (inst_opcode == OpJal || inst_opcode == OpJalr) alu_result = pcCurrent + 4;
    else if (inst_opcode == OpRegReg && inst_funct7 == 7'b0000001) begin
        case (inst_funct3)
           3'b000: alu_result = mul_full_result[31:0];
           3'b001: alu_result = mul_full_result[63:32];
           3'b010: alu_result = mul_full_result[63:32];
           3'b011: alu_result = mul_full_result[63:32];
           3'b100: alu_result = div_quotient_final;
           3'b101: alu_result = div_quotient_final;
           3'b110: alu_result = div_remainder_final;
           3'b111: alu_result = div_remainder_final;
        endcase
    end
    else if (inst_opcode == OpRegReg || inst_opcode == OpRegImm) begin
        case ({effective_funct7_5, inst_funct3})
          {1'b0, 3'b000}: alu_result = cla_sum;
          {1'b1, 3'b000}: alu_result = cla_sum;
          {1'b0, 3'b001}: alu_result = alu_op1 << alu_op2[4:0];
          {1'b0, 3'b101}: alu_result = alu_op1 >> alu_op2[4:0];
          {1'b1, 3'b101}: alu_result = $signed(alu_op1) >>> alu_op2[4:0];
          {1'b0, 3'b111}: alu_result = alu_op1 & alu_op2;
          {1'b0, 3'b110}: alu_result = alu_op1 | alu_op2;
          {1'b0, 3'b100}: alu_result = alu_op1 ^ alu_op2;
          // FIX: Explicitly ensure unsigned comparison for SLTIU/SLTU
          {1'b0, 3'b010}: alu_result = $signed(alu_op1) < $signed(alu_op2); 
          {1'b0, 3'b011}: alu_result = alu_op1 < alu_op2; // Verilog treats reg as unsigned by default
          default:        alu_result = 32'h0;
        endcase
    end

    reg_write_data_reg = alu_result;
    addr_to_dmem       = alu_result;

    if (inst_opcode == OpLoad) begin
        load_val_shifted = load_data_from_dmem >> (alu_result[1:0] * 8);
        case (inst_funct3)
           3'b000: reg_write_data_reg = {{24{load_val_shifted[7]}}, load_val_shifted[7:0]};
           3'b001: reg_write_data_reg = {{16{load_val_shifted[15]}}, load_val_shifted[15:0]};
           3'b010: reg_write_data_reg = load_data_from_dmem;
           3'b100: reg_write_data_reg = {24'b0, load_val_shifted[7:0]};
           3'b101: reg_write_data_reg = {16'b0, load_val_shifted[15:0]};
        endcase
    end

    if (inst_opcode == OpStore) begin
        store_data_to_dmem = rs2_data << (alu_result[1:0] * 8);
        case (inst_funct3)
           3'b000: store_we_to_dmem = 4'b0001 << alu_result[1:0];
           3'b001: store_we_to_dmem = 4'b0011 << alu_result[1:0];
           3'b010: store_we_to_dmem = 4'b1111;
        endcase
    end

    if (inst_ecall) halt = 1'b1;
  end

  wire take_branch = (inst_beq & (rs1_data == rs2_data)) |
                     (inst_bne & (rs1_data != rs2_data)) |
                     (inst_blt & ($signed(rs1_data) < $signed(rs2_data))) |
                     (inst_bge & ($signed(rs1_data) >= $signed(rs2_data))) |
                     (inst_bltu & (rs1_data < rs2_data)) |
                     (inst_bgeu & (rs1_data >= rs2_data));
                      
  wire [`REG_SIZE:0] pc_plus_4        = pcCurrent + 4;
  wire [`REG_SIZE:0] pc_branch_target = pcCurrent + imm_b_sext;
  wire [`REG_SIZE:0] pc_jal_target    = pcCurrent + imm_j_sext;
  wire [`REG_SIZE:0] pc_jalr_target   = (rs1_data + imm_i_sext) & ~1;

  always @(*) begin
      if (inst_jal)        pcNext = pc_jal_target;
      else if (inst_jalr)  pcNext = pc_jalr_target;
      else if (take_branch) pcNext = pc_branch_target;
      else                 pcNext = pc_plus_4;
  end

endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
  input                    rst,                 // rst for both imem and dmem
  input                    clock_mem,           // clock for both imem and dmem
  input      [`REG_SIZE:0] pc_to_imem,          // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] inst_from_imem,      // the value at memory location pc_to_imem
  input      [`REG_SIZE:0] addr_to_dmem,        // must always be aligned to a 4B boundary
  output reg [`REG_SIZE:0] load_data_from_dmem, // the value at memory location addr_to_dmem
  input      [`REG_SIZE:0] store_data_to_dmem,  // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
  // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
  // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
  input      [        3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  // preload instructions to mem_array
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(posedge clock_mem) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clock_mem) begin
   if (store_we_to_dmem[0]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
   end
   if (store_we_to_dmem[1]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
   end
   if (store_we_to_dmem[2]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
   end
   if (store_we_to_dmem[3]) begin
     mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
   end
   // dmem is "read-first": read returns value before the write
   load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  clock_proc,
    input  clock_mem,
    input  rst,
    output halt
);

  wire [`REG_SIZE:0] pc_to_imem, inst_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [        3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clock_mem           (clock_mem),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathSingleCycle datapath (
    .clk                 (clock_proc),
    .rst                 (rst),
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    .addr_to_dmem        (mem_data_addr),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we),
    .load_data_from_dmem (mem_data_loaded_value),
    .halt                (halt)
  );

endmodule