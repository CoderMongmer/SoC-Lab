`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31

// inst. are 32 bits in RV32IM
`define INST_SIZE 31

// RV opcodes are 7 bits
`define OPCODE_SIZE 6

`define DIVIDER_STAGES 8

// Don't forget your old codes
//`include "cla.v"
//`include "DividerUnsignedPipelined.v"

module RegFile (
  input      [        4:0] rd,
  input      [`REG_SIZE:0] rd_data,
  input      [        4:0] rs1,
  output reg [`REG_SIZE:0] rs1_data,
  input      [        4:0] rs2,
  output reg [`REG_SIZE:0] rs2_data,
  input                    clk,
  input                    we,
  input                    rst
);
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs[0:NumRegs-1];
  integer i;

  // 1. Write Logic (Synchronous)
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) regs[i] <= 0;
    end else if (we && (rd != 0)) begin
      regs[rd] <= rd_data;
    end
  end

  // 2. Read Logic with WD Bypass (Combinational)
  // If we read a register that is being written in the same cycle, forward the new value.
  always @(*) begin
    // RS1 Logic
    if (rs1 == 0) 
      rs1_data = 0;
    else if ((rs1 == rd) && we) // WD Bypass
      rs1_data = rd_data;
    else 
      rs1_data = regs[rs1];

    // RS2 Logic
    if (rs2 == 0) 
      rs2_data = 0;
    else if ((rs2 == rd) && we) // WD Bypass
      rs2_data = rd_data;
    else 
      rs2_data = regs[rs2];
  end

endmodule



module DatapathPipelined (
  input                     clk,
  input                     rst,
  output      [ `REG_SIZE:0] pc_to_imem,
  input       [`INST_SIZE:0] inst_from_imem,
  // dmem is read/write
  output reg [ `REG_SIZE:0] addr_to_dmem,
  input       [ `REG_SIZE:0] load_data_from_dmem,
  output reg [ `REG_SIZE:0] store_data_to_dmem,
  output reg [         3:0] store_we_to_dmem,
  output reg                halt,
  // The PC of the inst currently in Writeback. 0 if not a valid inst.
  output reg [ `REG_SIZE:0] trace_writeback_pc,
  // The bits of the inst currently in Writeback. 0 if not a valid inst.
  output reg [`INST_SIZE:0] trace_writeback_inst
);

  // opcodes
  localparam [`OPCODE_SIZE:0] OpcodeLoad    = 7'b00_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeStore   = 7'b01_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeBranch  = 7'b11_000_11;
  localparam [`OPCODE_SIZE:0] OpcodeJalr    = 7'b11_001_11;
  localparam [`OPCODE_SIZE:0] OpcodeMiscMem = 7'b00_011_11;
  localparam [`OPCODE_SIZE:0] OpcodeJal     = 7'b11_011_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegImm  = 7'b00_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeRegReg  = 7'b01_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeEnviron = 7'b11_100_11;
  localparam [`OPCODE_SIZE:0] OpcodeAuipc   = 7'b00_101_11;
  localparam [`OPCODE_SIZE:0] OpcodeLui     = 7'b01_101_11;

  // cycle counter
  reg [`REG_SIZE:0] cycles_current;
  always @(posedge clk) begin
    if (rst) cycles_current <= 0;
    else cycles_current <= cycles_current + 1;
  end

  // ========================================================================
  // PIPELINE REGISTERS
  // ========================================================================
  // IF/ID Registers
  reg [`REG_SIZE:0] d_pc;
  reg [`INST_SIZE:0] d_inst;

  // ID/EX Registers
  reg [`REG_SIZE:0] x_pc, x_rs1_data, x_rs2_data, x_imm;
  reg [6:0] x_opcode, x_funct7;
  reg [2:0] x_funct3;
  reg [4:0] x_rd, x_rs1, x_rs2;
  reg [`INST_SIZE:0] x_inst_debug;

  // EX/MEM Registers
  reg [`REG_SIZE:0] m_pc, m_alu_result, m_rs2_data;
  reg [4:0] m_rd;
  reg [6:0] m_opcode;
  reg [2:0] m_funct3;
  reg [`INST_SIZE:0] m_inst_debug;

  // MEM/WB Registers
  reg [`REG_SIZE:0] w_pc, w_alu_result, w_mem_data;
  reg [4:0] w_rd;
  reg [6:0] w_opcode;
  reg [2:0] w_funct3; // Added w_funct3 to support Load logic in WB
  reg [`INST_SIZE:0] w_inst_debug;

  // Control Signals (Wires)
  wire w_we;
  wire [`REG_SIZE:0] w_write_data;
  wire branch_taken;
  wire [`REG_SIZE:0] branch_target;
  wire stall; 

  // ========================================================================
  // 1. FETCH STAGE (F)
  // ========================================================================
  reg  [`REG_SIZE:0] f_pc_current;
  wire [`REG_SIZE:0] f_pc_next;

  assign f_pc_next = branch_taken ? branch_target : (f_pc_current + 4);

  always @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
    end else if (!stall) begin 
      f_pc_current <= f_pc_next;
    end
  end

  assign pc_to_imem = f_pc_current;

  // IF -> ID Pipeline Register
  always @(posedge clk) begin
    if (rst || branch_taken) begin // Flush on Branch
      d_pc <= 0;
      d_inst <= 32'h00000013; // NOP
    end else if (!stall) begin // Freeze Decode if stalled
      d_pc <= f_pc_current;
      d_inst <= inst_from_imem;
    end
  end

  // ========================================================================
  // 2. DECODE STAGE (D)
  // ========================================================================
  wire [4:0] d_rs1 = d_inst[19:15];
  wire [4:0] d_rs2 = d_inst[24:20];
  wire [4:0] d_rd  = d_inst[11:7];
  wire [6:0] d_opcode = d_inst[6:0];
  wire [2:0] d_funct3 = d_inst[14:12];
  wire [6:0] d_funct7 = d_inst[31:25];

  // Immediate Generation
  wire [`REG_SIZE:0] d_imm_i = {{20{d_inst[31]}}, d_inst[31:20]};
  wire [`REG_SIZE:0] d_imm_s = {{20{d_inst[31]}}, d_inst[31:25], d_inst[11:7]};
  wire [`REG_SIZE:0] d_imm_b = {{19{d_inst[31]}}, d_inst[31], d_inst[7], d_inst[30:25], d_inst[11:8], 1'b0};
  wire [`REG_SIZE:0] d_imm_u = {d_inst[31:12], 12'b0};
  wire [`REG_SIZE:0] d_imm_j = {{11{d_inst[31]}}, d_inst[31], d_inst[19:12], d_inst[20], d_inst[30:21], 1'b0};

  reg [`REG_SIZE:0] d_imm;
  always @(*) begin
    case(d_opcode)
      OpcodeStore:  d_imm = d_imm_s;
      OpcodeBranch: d_imm = d_imm_b;
      OpcodeLui, OpcodeAuipc: d_imm = d_imm_u;
      OpcodeJal:    d_imm = d_imm_j;
      default:      d_imm = d_imm_i;
    endcase
  end

  wire [`REG_SIZE:0] d_rs1_data, d_rs2_data;
  RegFile rf (
    .clk(clk), .rst(rst), .we(w_we), .rd(w_rd), .rd_data(w_write_data),
    .rs1(d_rs1), .rs1_data(d_rs1_data), .rs2(d_rs2), .rs2_data(d_rs2_data)
  );

  // --- HAZARD DETECTION UNIT ---
  wire x_is_load = (x_opcode == OpcodeLoad);
   
  // Identify DIV instructions in X stage (M-Extension bit 0 set, Funct3 4-7)
  wire x_is_div = (x_opcode == OpcodeRegReg) && (x_funct7 == 7'b0000001) && (x_funct3[2] == 1'b1);
   
  // Identify DIV instructions in D stage
  wire d_is_div = (d_opcode == OpcodeRegReg) && (d_funct7 == 7'b0000001) && (d_funct3[2] == 1'b1);

  // Divider Tracking Signals
  reg [4:0] div_pipe_rd     [0:7];
  reg       div_pipe_valid [0:7];
  integer h;
   
  reg div_busy;
  always @(*) begin
      div_busy = 0;
      for(h=0; h<8; h=h+1) begin
          if (div_pipe_valid[h]) div_busy = 1;
      end
  end
   
  // 2. Divider RAW Hazard
  reg div_raw_hazard;
  always @(*) begin
      div_raw_hazard = 0;
      for(h=0; h<8; h=h+1) begin
          if (div_pipe_valid[h] && (div_pipe_rd[h] != 0)) begin
             if (div_pipe_rd[h] == d_rs1 || div_pipe_rd[h] == d_rs2) div_raw_hazard = 1;
          end
      end
      if (x_is_div && (x_rd != 0)) begin
         if (x_rd == d_rs1 || x_rd == d_rs2) div_raw_hazard = 1;
      end
      if ((m_opcode == OpcodeRegReg) && (m_rd != 0) && (m_rd == d_rs1 || m_rd == d_rs2)) begin
          // Conservative stall for dependent instructions
      end
  end

  // IN-ORDER STALL LOGIC:
  wire div_in_progress = div_busy || (x_is_div && !rst);
  wire stall_for_ordering = div_in_progress && !d_is_div;

  // Combined Stall Logic
  assign stall = (x_is_load && (x_rd != 0) && ((x_rd == d_rs1) || (x_rd == d_rs2))) || // Load-Use
                 div_raw_hazard ||          
                 stall_for_ordering;

  // ID -> EX Pipeline Register
  always @(posedge clk) begin
    if (rst || branch_taken || stall) begin
      x_pc <= 0;
      x_opcode <= 32'h00000013; // NOP
      x_rd <= 0; x_rs1 <= 0; x_rs2 <= 0;
      x_funct3 <= 0; x_funct7 <= 0; x_imm <= 0; 
      x_rs1_data <= 0; x_rs2_data <= 0;
      x_inst_debug <= 32'h00000013;
    end else begin
      x_pc <= d_pc;
      x_opcode <= d_opcode;
      x_rd <= d_rd;
      x_rs1 <= d_rs1;
      x_rs2 <= d_rs2;
      x_funct3 <= d_funct3;
      x_funct7 <= d_funct7;
      x_imm <= d_imm;
      x_rs1_data <= d_rs1_data;
      x_rs2_data <= d_rs2_data;
      x_inst_debug <= d_inst;
    end
  end

  // ========================================================================
  // 3. EXECUTE STAGE (X)
  // ========================================================================

  // --- FORWARDING LOGIC ---
  reg [`REG_SIZE:0] alu_op1_input, alu_op2_input;
   
  wire m_we_forward = (m_opcode == OpcodeRegImm) || (m_opcode == OpcodeRegReg) || (m_opcode == OpcodeLui) || (m_opcode == OpcodeAuipc) || (m_opcode == OpcodeLoad) || (m_opcode == OpcodeJal) || (m_opcode == OpcodeJalr);
  wire w_we_forward = (w_opcode == OpcodeRegImm) || (w_opcode == OpcodeRegReg) || (w_opcode == OpcodeLui) || (w_opcode == OpcodeAuipc) || (w_opcode == OpcodeLoad) || (w_opcode == OpcodeJal) || (w_opcode == OpcodeJalr);

  always @(*) begin
    // RS1 Forwarding
    if (m_we_forward && (m_rd != 0) && (m_rd == x_rs1)) 
       alu_op1_input = m_alu_result; // MX Bypass
    else if (w_we_forward && (w_rd != 0) && (w_rd == x_rs1)) 
       alu_op1_input = w_write_data; // WX Bypass
    else 
       alu_op1_input = x_rs1_data;

    // RS2 Forwarding
    if (m_we_forward && (m_rd != 0) && (m_rd == x_rs2)) 
       alu_op2_input = m_alu_result; // MX Bypass
    else if (w_we_forward && (w_rd != 0) && (w_rd == x_rs2)) 
       alu_op2_input = w_write_data; // WX Bypass
    else 
       alu_op2_input = x_rs2_data;
  end

  // --- DIVIDER INTEGRATION ---
  wire [31:0] w_div_quotient, w_div_remainder;

  Divider_Unsigned_Pipelined div_unit (
      .clk(clk), .rst(rst),
      .i_dividend(alu_op1_input), 
      .i_divisor(alu_op2_input),
      .o_quotient(w_div_quotient),
      .o_remainder(w_div_remainder)
  );

  reg [2:0] div_pipe_funct3 [0:7];
  integer k;
   
  always @(posedge clk) begin
    if (rst) begin
      for(k=0; k<8; k=k+1) begin
         div_pipe_valid[k]  <= 0;
         div_pipe_rd[k]     <= 0;
         div_pipe_funct3[k] <= 0;
      end
    end else begin
      for(k=1; k<8; k=k+1) begin
         div_pipe_valid[k]  <= div_pipe_valid[k-1];
         div_pipe_rd[k]     <= div_pipe_rd[k-1];
         div_pipe_funct3[k] <= div_pipe_funct3[k-1];
      end
      if (x_is_div && !rst) begin
         div_pipe_valid[0]  <= 1;
         div_pipe_rd[0]     <= x_rd;
         div_pipe_funct3[0] <= x_funct3;
      end else begin
         div_pipe_valid[0]  <= 0;
         div_pipe_rd[0]     <= 0;
         div_pipe_funct3[0] <= 0;
      end
    end
  end

  // --- ALU LOGIC ---
  reg [`REG_SIZE:0] alu_op2_final;
  reg [`REG_SIZE:0] x_alu_result;
  wire [63:0] mul_res_full; 

  wire signed [32:0] mul_op1 = (x_funct3 == 3'b011) ? {1'b0, alu_op1_input} : {alu_op1_input[31], alu_op1_input};
  wire signed [32:0] mul_op2 = (x_funct3 == 3'b011 || x_funct3 == 3'b010) ? {1'b0, alu_op2_input} : {alu_op2_input[31], alu_op2_input};
   
  assign mul_res_full = $signed(mul_op1) * $signed(mul_op2);

  wire eff_funct7_5 = (x_opcode == OpcodeRegImm) ? (((x_funct3 == 3'b001) || (x_funct3 == 3'b101)) ? x_funct7[5] : 1'b0) : x_funct7[5];

  always @(*) begin
    if ((x_opcode == OpcodeRegImm) || (x_opcode == OpcodeLoad) || (x_opcode == OpcodeStore) || (x_opcode == OpcodeJalr) || (x_opcode == OpcodeAuipc) || (x_opcode == OpcodeLui))
        alu_op2_final = x_imm;
    else
        alu_op2_final = alu_op2_input;

    if (x_is_div) begin
        x_alu_result = 0; 
    end else begin
        case (x_opcode)
            OpcodeLui: x_alu_result = x_imm;
            OpcodeAuipc: x_alu_result = x_pc + x_imm;
            OpcodeJal, OpcodeJalr: x_alu_result = x_pc + 4;
            OpcodeRegImm, OpcodeRegReg, OpcodeLoad, OpcodeStore: begin
                if (x_opcode == OpcodeRegReg && x_funct7 == 7'b0000001 && x_funct3 < 3'b100) begin
                    case (x_funct3)
                        3'b000: x_alu_result = mul_res_full[31:0];  // MUL
                        3'b001: x_alu_result = mul_res_full[63:32]; // MULH
                        3'b010: x_alu_result = mul_res_full[63:32]; // MULHSU
                        3'b011: x_alu_result = mul_res_full[63:32]; // MULHU
                        default: x_alu_result = 0;
                    endcase
                end else if (x_opcode == OpcodeLoad || x_opcode == OpcodeStore) begin
                    x_alu_result = alu_op1_input + alu_op2_final;
                end else begin
                    case ({eff_funct7_5, x_funct3})
                        {1'b0, 3'b000}: x_alu_result = alu_op1_input + alu_op2_final;
                        {1'b1, 3'b000}: x_alu_result = alu_op1_input - alu_op2_final;
                        {1'b0, 3'b001}: x_alu_result = alu_op1_input << alu_op2_final[4:0];
                        {1'b0, 3'b010}: x_alu_result = ($signed(alu_op1_input) < $signed(alu_op2_final)) ? 1 : 0;
                        {1'b0, 3'b011}: x_alu_result = (alu_op1_input < alu_op2_final) ? 1 : 0;
                        {1'b0, 3'b100}: x_alu_result = alu_op1_input ^ alu_op2_final;
                        {1'b0, 3'b101}: x_alu_result = alu_op1_input >> alu_op2_final[4:0];
                        {1'b1, 3'b101}: x_alu_result = $signed(alu_op1_input) >>> alu_op2_final[4:0];
                        {1'b0, 3'b110}: x_alu_result = alu_op1_input | alu_op2_final;
                        {1'b0, 3'b111}: x_alu_result = alu_op1_input & alu_op2_final;
                        default: x_alu_result = 0;
                    endcase
                end
            end
            default: x_alu_result = 0;
        endcase
    end
  end

  // Branch Logic
  assign branch_target = (x_opcode == OpcodeJalr) ? (alu_op1_input + x_imm) : (x_pc + x_imm);
  reg branch_condition_met;
  always @(*) begin
    case (x_funct3)
        3'b000: branch_condition_met = (alu_op1_input == alu_op2_input);
        3'b001: branch_condition_met = (alu_op1_input != alu_op2_input);
        3'b100: branch_condition_met = ($signed(alu_op1_input) < $signed(alu_op2_input));
        3'b101: branch_condition_met = ($signed(alu_op1_input) >= $signed(alu_op2_input));
        3'b110: branch_condition_met = (alu_op1_input < alu_op2_input);
        3'b111: branch_condition_met = (alu_op1_input >= alu_op2_input);
        default: branch_condition_met = 0;
    endcase
  end
  assign branch_taken = ((x_opcode == OpcodeBranch) && branch_condition_met) || (x_opcode == OpcodeJal) || (x_opcode == OpcodeJalr);

  // EX -> MEM Pipeline Register
  wire div_finishing = div_pipe_valid[7]; 

  always @(posedge clk) begin
    if (rst) begin
      m_pc <= 0; m_opcode <= 0; m_rd <= 0; m_inst_debug <= 32'h00000013;
      m_funct3 <= 0; m_alu_result <= 0; m_rs2_data <= 0;
    end else if (div_finishing) begin
      // --- PRIORITY: DIVIDER RESULT ---
      m_pc <= 0; 
      m_rd <= div_pipe_rd[7];
      m_opcode <= OpcodeRegReg; 
      m_inst_debug <= 32'h00000033;
      
      case (div_pipe_funct3[7])
         3'b100, 3'b101: m_alu_result <= w_div_quotient;
         3'b110, 3'b111: m_alu_result <= w_div_remainder;
         default:        m_alu_result <= w_div_quotient;
      endcase
      m_rs2_data <= 0; 
      m_funct3 <= 0;
      
    end else if (!stall) begin 
      // --- STANDARD ALU PATH ---
      m_pc <= x_pc;
      m_alu_result <= x_alu_result;
      m_rs2_data <= alu_op2_input;
      
      if (x_is_div) begin
         m_rd <= 0;
         m_opcode <= OpcodeRegImm; 
      end else begin
         m_rd <= x_rd;
         m_opcode <= x_opcode;
      end

      m_funct3 <= x_funct3;
      m_inst_debug <= x_inst_debug;
    end
  end

  // ========================================================================
  // 4. MEMORY STAGE (M)
  // ========================================================================
  // UPDATED: Logic for shifting/aligning store data
  always @(*) begin
    addr_to_dmem = m_alu_result;
    
    // Shift the data to the correct byte/half-word position
    store_data_to_dmem = m_rs2_data << (m_alu_result[1:0] * 8);
    
    store_we_to_dmem = 0;
    if (m_opcode == OpcodeStore) begin
        case(m_funct3)
            3'b000: store_we_to_dmem = 4'b0001 << m_alu_result[1:0]; // SB
            3'b001: store_we_to_dmem = 4'b0011 << m_alu_result[1:0]; // SH
            3'b010: store_we_to_dmem = 4'b1111;                      // SW
            default: store_we_to_dmem = 4'b0000; 
        endcase
    end
  end

  // MEM -> WB Pipeline Register
  always @(posedge clk) begin
    if (rst) begin
      w_pc <= 0; w_opcode <= 0; w_rd <= 0; w_inst_debug <= 32'h00000013;
      w_funct3 <= 0; // Reset new register
    end else begin
      w_pc <= m_pc;
      w_alu_result <= m_alu_result;
      w_mem_data <= load_data_from_dmem;
      w_rd <= m_rd;
      w_opcode <= m_opcode;
      w_funct3 <= m_funct3; // Pass funct3 to WB
      w_inst_debug <= m_inst_debug;
    end
  end

  // ========================================================================
  // 5. WRITEBACK STAGE (W)
  // ========================================================================
  assign w_we = (w_opcode == OpcodeRegImm) || (w_opcode == OpcodeRegReg) || (w_opcode == OpcodeLui) || (w_opcode == OpcodeAuipc) || (w_opcode == OpcodeLoad) || (w_opcode == OpcodeJal) || (w_opcode == OpcodeJalr);
   
  // UPDATED: Logic for Load Data alignment and sign-extension
  reg [31:0] w_load_val_shifted;
  reg [31:0] w_load_data_processed;
  
  always @(*) begin
    // Shift data to LSB
    w_load_val_shifted = w_mem_data >> (w_alu_result[1:0] * 8);
    
    case (w_funct3)
        3'b000: w_load_data_processed = {{24{w_load_val_shifted[7]}}, w_load_val_shifted[7:0]};   // LB
        3'b001: w_load_data_processed = {{16{w_load_val_shifted[15]}}, w_load_val_shifted[15:0]}; // LH
        3'b010: w_load_data_processed = w_mem_data;                                               // LW
        3'b100: w_load_data_processed = {24'b0, w_load_val_shifted[7:0]};                         // LBU
        3'b101: w_load_data_processed = {16'b0, w_load_val_shifted[15:0]};                        // LHU
        default: w_load_data_processed = w_mem_data;
    endcase
  end

  // Select loaded data or ALU result
  assign w_write_data = (w_opcode == OpcodeLoad) ? w_load_data_processed : w_alu_result;

  always @(*) begin
    trace_writeback_pc = w_pc;
    trace_writeback_inst = w_inst_debug;
    halt = (w_opcode == OpcodeEnviron);
  end

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
    input                     rst,                  
    input                     clk,                  
    input       [`REG_SIZE:0] pc_to_imem,           
    output reg [`REG_SIZE:0] inst_from_imem,       
    input       [`REG_SIZE:0] addr_to_dmem,         
    output reg [`REG_SIZE:0] load_data_from_dmem, 
    input       [`REG_SIZE:0] store_data_to_dmem,   
    input       [        3:0] store_we_to_dmem
);

  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end

  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(negedge clk) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clk) begin
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
    load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
  end
endmodule


module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
    input                    rst,                 // rst for both imem and dmem
    input                    clk,                 // clock for both imem and dmem
	                                              // The memory reads/writes on @(negedge clk)
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

  always @(negedge clk) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end

  always @(negedge clk) begin
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

/* This design has just one clock for both processor and memory. */
module Processor (
    input                 clk,
    input                 rst,
    output                halt,
    output [ `REG_SIZE:0] trace_writeback_pc,
    output [`INST_SIZE:0] trace_writeback_inst
);

  wire [`INST_SIZE:0] inst_from_imem;
  wire [ `REG_SIZE:0] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [         3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
    .rst                 (rst),
    .clk                 (clk),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathPipelined datapath (
    .clk                  (clk),
    .rst                  (rst),
    .pc_to_imem           (pc_to_imem),
    .inst_from_imem       (inst_from_imem),
    .addr_to_dmem         (mem_data_addr),
    .store_data_to_dmem   (mem_data_to_write),
    .store_we_to_dmem     (mem_data_we),
    .load_data_from_dmem  (mem_data_loaded_value),
    .halt                 (halt),
    .trace_writeback_pc   (trace_writeback_pc),
    .trace_writeback_inst (trace_writeback_inst)
  );

endmodule
