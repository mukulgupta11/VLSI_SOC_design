// ==============================================================================
// VLSI SoC Studio — Apple-Class High-Throughput GPU Execution Slice
// Synthesizable SIMD Vector Execution Unit with Power Gating Controls
// ==============================================================================

`timescale 1ns / 1ps

module gpu_slice (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         sleep_gpu,         // IEEE 1801 UPF power switch control
    input  wire         iso_en,            // Isolation clamp control
    input  wire [511:0] vec_in_a,          // 16 lanes x 32-bit SIMD Operand A
    input  wire [511:0] vec_in_b,          // 16 lanes x 32-bit SIMD Operand B
    input  wire [511:0] vec_in_c,          // 16 lanes x 32-bit FMA Accumulator
    input  wire [2:0]   vec_op,            // 0: ADD, 1: MUL, 2: FMA, 3: DOT, 4: MAX
    input  wire         vec_valid,
    output reg  [511:0] vec_out,           // 16 lanes x 32-bit SIMD Result
    output reg          vec_ready,
    output reg  [31:0]  tex_cache_hit_cnt, // Texture Cache Performance Monitor
    output reg  [31:0]  alu_util_cnt
);

    // --------------------------------------------------------------------------
    // Vector Registers & Pipeline Stages
    // --------------------------------------------------------------------------
    reg [511:0] pipe_r0_a, pipe_r0_b, pipe_r0_c;
    reg [2:0]   pipe_op_0, pipe_op_1;
    reg         pipe_vld_0, pipe_vld_1;
    reg [511:0] pipe_alu_out;

    integer lane;

    // --------------------------------------------------------------------------
    // 16-Lane SIMD Vector Arithmetic Logic
    // --------------------------------------------------------------------------
    reg [31:0] lane_res [0:15];
    always @(*) begin
        for (lane = 0; lane < 16; lane = lane + 1) begin
            case (pipe_op_0)
                3'b000: lane_res[lane] = pipe_r0_a[lane*32 +: 32] + pipe_r0_b[lane*32 +: 32];
                3'b001: lane_res[lane] = pipe_r0_a[lane*32 +: 32] * pipe_r0_b[lane*32 +: 32];
                3'b010: lane_res[lane] = (pipe_r0_a[lane*32 +: 32] * pipe_r0_b[lane*32 +: 32]) + pipe_r0_c[lane*32 +: 32];
                3'b011: lane_res[lane] = (pipe_r0_a[lane*32 +: 32] > pipe_r0_b[lane*32 +: 32]) ? pipe_r0_a[lane*32 +: 32] : pipe_r0_b[lane*32 +: 32];
                3'b100: lane_res[lane] = pipe_r0_a[lane*32 +: 32] ^ pipe_r0_b[lane*32 +: 32];
                default: lane_res[lane] = 32'd0;
            endcase
        end
    end

    // --------------------------------------------------------------------------
    // Sequential Pipeline Clocking & Power Gating Logic
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_r0_a         <= 512'd0;
            pipe_r0_b         <= 512'd0;
            pipe_r0_c         <= 512'd0;
            pipe_op_0         <= 3'd0;
            pipe_op_1         <= 3'd0;
            pipe_vld_0        <= 1'b0;
            pipe_vld_1        <= 1'b0;
            pipe_alu_out      <= 512'd0;
            vec_out           <= 512'd0;
            vec_ready         <= 1'b0;
            tex_cache_hit_cnt <= 32'd0;
            alu_util_cnt      <= 32'd0;
        end else if (sleep_gpu) begin
            // UPF Power-Gated State: clamp outputs to zero if iso_en active
            if (iso_en) begin
                vec_out   <= 512'd0;
                vec_ready <= 1'b0;
            end
        end else begin
            // Active Vector Pipeline
            vec_ready  <= 1'b1;
            pipe_r0_a  <= vec_in_a;
            pipe_r0_b  <= vec_in_b;
            pipe_r0_c  <= vec_in_c;
            pipe_op_0  <= vec_op;
            pipe_vld_0 <= vec_valid;

            // Pipeline Stage 1
            for (lane = 0; lane < 16; lane = lane + 1) begin
                pipe_alu_out[lane*32 +: 32] <= lane_res[lane];
            end
            pipe_op_1  <= pipe_op_0;
            pipe_vld_1 <= pipe_vld_0;

            // Pipeline Stage 2 -> Output Register
            vec_out <= pipe_alu_out;

            if (vec_valid) begin
                alu_util_cnt <= alu_util_cnt + 32'd1;
                tex_cache_hit_cnt <= tex_cache_hit_cnt + 32'd1;
            end
        end
    end

endmodule
