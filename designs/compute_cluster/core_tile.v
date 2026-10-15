// ==============================================================================
// VLSI SoC Studio — Apple-Class High-Performance 64-bit Compute Core Tile
// Synthesizable RISC-V / Apple-Style Compute Core with 5-Stage Pipeline
// ==============================================================================

`timescale 1ns / 1ps

module core_tile (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sleep_ctrl,       // Power gating sleep control
    input  wire [31:0] inst_in,          // Instruction stream
    input  wire        inst_valid,
    input  wire [63:0] mem_rdata,        // Memory read data
    input  wire        mem_ready,
    output reg  [63:0] inst_addr,        // Program counter
    output reg         inst_req,
    output reg  [63:0] mem_addr,         // Data memory address
    output reg  [63:0] mem_wdata,        // Data memory write data
    output reg         mem_write,
    output reg         mem_valid,
    output reg  [63:0] perf_counter      // Retired instruction cycle counter
);

    // --------------------------------------------------------------------------
    // 32 x 64-bit General Purpose Architectural Register File
    // --------------------------------------------------------------------------
    reg [63:0] regfile [0:31];
    integer i;

    // --------------------------------------------------------------------------
    // Pipeline Registers
    // --------------------------------------------------------------------------
    // 1. IF/ID Stage
    reg [63:0] if_id_pc;
    reg [31:0] if_id_inst;

    // 2. ID/EX Stage
    reg [63:0] id_ex_pc;
    reg [63:0] id_ex_rs1_val;
    reg [63:0] id_ex_rs2_val;
    reg [63:0] id_ex_imm;
    reg [4:0]  id_ex_rd;
    reg [3:0]  id_ex_alu_op;
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        id_ex_reg_write;

    // 3. EX/MEM Stage
    reg [63:0] ex_mem_alu_res;
    reg [63:0] ex_mem_wdata;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_reg_write;

    // 4. MEM/WB Stage
    reg [63:0] mem_wb_data;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_reg_write;

    // --------------------------------------------------------------------------
    // Decode Wires
    // --------------------------------------------------------------------------
    wire [4:0]  dec_rs1 = if_id_inst[19:15];
    wire [4:0]  dec_rs2 = if_id_inst[24:20];
    wire [4:0]  dec_rd  = if_id_inst[11:7];
    wire [6:0]  dec_op  = if_id_inst[6:0];
    wire [2:0]  dec_f3  = if_id_inst[14:12];
    wire [6:0]  dec_f7  = if_id_inst[31:25];
    wire [63:0] dec_imm = {{52{if_id_inst[31]}}, if_id_inst[31:20]};

    // --------------------------------------------------------------------------
    // ALU Execution
    // --------------------------------------------------------------------------
    reg [63:0] alu_result;
    always @(*) begin
        case (id_ex_alu_op)
            4'b0000: alu_result = id_ex_rs1_val + id_ex_rs2_val;                   // ADD
            4'b0001: alu_result = id_ex_rs1_val - id_ex_rs2_val;                   // SUB
            4'b0010: alu_result = id_ex_rs1_val & id_ex_rs2_val;                   // AND
            4'b0011: alu_result = id_ex_rs1_val | id_ex_rs2_val;                   // OR
            4'b0100: alu_result = id_ex_rs1_val ^ id_ex_rs2_val;                   // XOR
            4'b0101: alu_result = id_ex_rs1_val << id_ex_rs2_val[5:0];             // SLL
            4'b0110: alu_result = id_ex_rs1_val >> id_ex_rs2_val[5:0];             // SRL
            4'b0111: alu_result = $signed(id_ex_rs1_val) >>> id_ex_rs2_val[5:0];   // SRA
            4'b1000: alu_result = ($signed(id_ex_rs1_val) < $signed(id_ex_rs2_val)) ? 64'd1 : 64'd0; // SLT
            default: alu_result = id_ex_rs1_val + id_ex_imm;
        endcase
    end

    // --------------------------------------------------------------------------
    // Sequential Pipeline Clocking
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inst_addr       <= 64'h0000_0000_1000_0000;
            inst_req        <= 1'b0;
            mem_addr        <= 64'd0;
            mem_wdata       <= 64'd0;
            mem_write       <= 1'b0;
            mem_valid       <= 1'b0;
            perf_counter    <= 64'd0;

            if_id_pc        <= 64'd0;
            if_id_inst      <= 32'd0;

            id_ex_pc        <= 64'd0;
            id_ex_rs1_val   <= 64'd0;
            id_ex_rs2_val   <= 64'd0;
            id_ex_imm       <= 64'd0;
            id_ex_rd        <= 5'd0;
            id_ex_alu_op    <= 4'd0;
            id_ex_mem_read  <= 1'b0;
            id_ex_mem_write <= 1'b0;
            id_ex_reg_write <= 1'b0;

            ex_mem_alu_res  <= 64'd0;
            ex_mem_wdata    <= 64'd0;
            ex_mem_rd       <= 5'd0;
            ex_mem_mem_read <= 1'b0;
            ex_mem_mem_write<= 1'b0;
            ex_mem_reg_write<= 1'b0;

            mem_wb_data     <= 64'd0;
            mem_wb_rd       <= 5'd0;
            mem_wb_reg_write<= 1'b0;

            for (i = 0; i < 32; i = i + 1) begin
                regfile[i] <= 64'd0;
            end
        end else if (!sleep_ctrl) begin
            perf_counter <= perf_counter + 64'd1;

            // Stage 1: Instruction Fetch (IF)
            inst_req  <= 1'b1;
            inst_addr <= inst_addr + 64'd4;
            if (inst_valid) begin
                if_id_pc   <= inst_addr;
                if_id_inst <= inst_in;
            end

            // Stage 2: Instruction Decode (ID)
            id_ex_pc        <= if_id_pc;
            id_ex_rs1_val   <= (dec_rs1 != 5'd0) ? regfile[dec_rs1] : 64'd0;
            id_ex_rs2_val   <= (dec_rs2 != 5'd0) ? regfile[dec_rs2] : 64'd0;
            id_ex_imm       <= dec_imm;
            id_ex_rd        <= dec_rd;
            id_ex_alu_op    <= {dec_f7[5], dec_f3};
            id_ex_mem_read  <= (dec_op == 7'b0000011);
            id_ex_mem_write <= (dec_op == 7'b0100011);
            id_ex_reg_write <= (dec_rd != 5'd0);

            // Stage 3: Execution (EX)
            ex_mem_alu_res  <= alu_result;
            ex_mem_wdata    <= id_ex_rs2_val;
            ex_mem_rd       <= id_ex_rd;
            ex_mem_mem_read <= id_ex_mem_read;
            ex_mem_mem_write<= id_ex_mem_write;
            ex_mem_reg_write<= id_ex_reg_write;

            // Stage 4: Memory Access (MEM)
            mem_addr  <= ex_mem_alu_res;
            mem_wdata <= ex_mem_wdata;
            mem_write <= ex_mem_mem_write;
            mem_valid <= ex_mem_mem_read | ex_mem_mem_write;

            mem_wb_data     <= ex_mem_mem_read ? mem_rdata : ex_mem_alu_res;
            mem_wb_rd       <= ex_mem_rd;
            mem_wb_reg_write<= ex_mem_reg_write;

            // Stage 5: Writeback (WB)
            if (mem_wb_reg_write && mem_wb_rd != 5'd0) begin
                regfile[mem_wb_rd] <= mem_wb_data;
            end
        end
    end

endmodule
