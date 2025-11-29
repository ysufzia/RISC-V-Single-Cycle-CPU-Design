`default_nettype none

module processor(
    input         clk,
    input         reset,
    output [31:0] PC,
    input  [31:0] instruction,
    output        WE,
    output [31:0] address_to_mem,
    output [31:0] data_to_mem,
    input  [31:0] data_from_mem
);

    // --- Internal Wires ---
    wire [31:0] pc_out_wire, pc_next_wire, pc_wire, decode_wire;
    wire [31:0] read_data1, read_data_wire, WB_data_wire;
    wire [31:0] branch_target, immgen_wire, muxtoAlu;
    wire [31:0] alu_out_wire, wb_mux_out_1;

    wire RegWrite, ALUSrc, MemWrite, MemToReg;
    wire Branchbeq, Branchjal, Branchjalr, Zero;

    wire [3:0] ALUcontrol_wire;
    wire [2:0] immcontrol_wire;

    wire Branchjalx;
    wire branch_outcome;

    assign Branchjalx = Branchjal | Branchjalr;

    wire is_beq_op;
    wire is_blt_op;
    wire beq_success;
    wire blt_success;

    assign is_beq_op = (ALUcontrol_wire == 4'd1); // 1 = SUB (for beq)
    assign is_blt_op = (ALUcontrol_wire == 4'd4); // 4 = SLT (for blt)

    assign beq_success = is_beq_op & Zero;
    assign blt_success = is_blt_op & ~Zero;

    assign branch_outcome = (Branchbeq & (beq_success | blt_success)) | Branchjalx;

    pcounter pc1(
        .clk(clk),
        .rst(reset),
        .next_pc(pc_next_wire),
        .outpc(pc_out_wire)
    );

    pcadder pcadd1(
        .pc(pc_out_wire),
        .out(pc_wire)
    );

    mux_2to1pc #(.width(32)) pc_mux1(
        .pc_plus4(pc_wire),
        .branch_target(branch_target),
        .branch_outcome(branch_outcome),
        .pc_in(pc_next_wire)
    );

    GPR gpr1(
        .a1(instruction[19:15]),
        .a2(instruction[24:20]),
        .a3(instruction[11:7]),
        .wdata3(WB_data_wire),
        .clk(clk),
        .WEnable3(RegWrite),
        .rd1(read_data1),
        .rd2(read_data_wire)
    );

    imm_decode imm1(
        .immcontrol(immcontrol_wire),
        .instruction(instruction),
        .Immop(immgen_wire)
    );

    mux_2to1 #(.width(32)) alu_src_mux1(
        .in0(read_data_wire),
        .in1(immgen_wire),
        .select(ALUSrc),
        .out(muxtoAlu)
    );

    alu alu1(
        .srcA(read_data1),
        .srcB(muxtoAlu),
        .ALU_control(ALUcontrol_wire),
        .ALU_Out(alu_out_wire),
        .zero(Zero)
    );

    mux_2to1_aluout_pcplus4 #(.width(32)) wb_mux1(
        .alu_out(alu_out_wire),
        .pc_plus4(pc_wire),
        .Branchjalx(Branchjalx),
        .aluout_pcplus4(wb_mux_out_1)
    );

    mux_result #(.width(32)) mux_result1(
        .read_data(data_from_mem),
        .mux_2to1_aluout_pcplus4(wb_mux_out_1),
        .memtoreg(MemToReg),
        .result(WB_data_wire)
    );

    ctrl_unit ctrl1(
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .ALU_Src(ALUSrc),
        .Reg_Write(RegWrite),
        .Mem_Write(MemWrite),
        .Branchbeq(Branchbeq),
        .Branchjal(Branchjal),
        .Branchjalr(Branchjalr),
        .memtoreg(MemToReg),
        .ALU_Control(ALUcontrol_wire),
        .immcontrol(immcontrol_wire)
    );

    adder_pc_immop #(.width(32)) add_pc_imm_jal(
        .pc(pc_out_wire),
        .immop(immgen_wire),
        .out_pc_immop(decode_wire)
    );

    mux_2to1aluout_adderpcimmop #(.width(32)) mux_branch_target1(
        .aluout(alu_out_wire),
        .pc_immop(decode_wire),
        .Branchjalr(Branchjalr),
        .Branch_target(branch_target)
    );

    assign PC = pc_out_wire;
    assign WE = MemWrite;
    assign address_to_mem = alu_out_wire;
    assign data_to_mem = read_data_wire;

endmodule


module top(
    input clk,
    input reset,
    output [31:0] data_to_mem_out,
    output [31:0] address_to_mem_out,
    output        write_enable_out
);

    wire [31:0] pc_out;
    wire [31:0] instruction;
    wire [31:0] data_from_mem;
    wire [31:0] data_to_mem;
    wire [31:0] address_to_mem;
    wire        mem_write_enable;

    inst_mem imem(
        .address(pc_out[7:2]),
        .rd(instruction)
    );

    data_mem dmem(
        .clk(clk),
        .we(mem_write_enable),
        .address(address_to_mem),
        .wd(data_to_mem),
        .rd(data_from_mem)
    );

    processor cpu(
        .clk(clk),
        .reset(reset),
        .PC(pc_out),
        .instruction(instruction),
        .WE(mem_write_enable),
        .address_to_mem(address_to_mem),
        .data_to_mem(data_to_mem),
        .data_from_mem(data_from_mem)
    );

    assign data_to_mem_out    = data_to_mem;
    assign address_to_mem_out = address_to_mem;
    assign write_enable_out   = mem_write_enable;

endmodule


module adder #(parameter width = 32)(
    input  [width-1:0] in0,
    input  [width-1:0] in1,
    output [width-1:0] out
);
    assign out = in0 + in1;
endmodule

module pcounter(
    input clk,
    input rst,
    input  [31:0] next_pc,
    output reg [31:0] outpc
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            outpc <= 32'h00000000; 
        else
            outpc <= next_pc;
    end
endmodule


module pcadder #(parameter width = 32)(
    input  [width-1:0] pc,
    output [width-1:0] out
);
    assign out = pc + 32'd4;
endmodule


module mux_2to1 #(parameter width = 32)(
    input  [width-1:0] in0,
    input  [width-1:0] in1,
    input              select,
    output reg [width-1:0] out
);
    always @(*) begin
        if (!select)
            out = in0;
        else
            out = in1;
    end
endmodule


module GPR #(parameter width = 32)(
    input  [4:0] a1,
    input  [4:0] a2,
    input  [4:0] a3,
    input  [width-1:0] wdata3,
    input clk,
    input WEnable3,
    output [width-1:0] rd1,
    output [width-1:0] rd2
);
    reg [width-1:0] rf[31:0];

    assign rd1 = (a1 == 5'b0) ? 32'b0 : rf[a1];
    assign rd2 = (a2 == 5'b0) ? 32'b0 : rf[a2];

    always @(posedge clk) begin
        if (WEnable3 && (a3 != 5'b0))
            rf[a3] <= wdata3;
    end
endmodule


module adder_pc_immop #(parameter width = 32)(
    input  [31:0] pc,
    input  [31:0] immop,
    output [31:0] out_pc_immop
);
    assign out_pc_immop = pc + immop;
endmodule


module mux_2to1aluout_adderpcimmop #(parameter width = 32)(
    input  [31:0] aluout,
    input  [31:0] pc_immop,
    input         Branchjalr,
    output reg [31:0] Branch_target
);
    always @(*) begin
        if (!Branchjalr)
            Branch_target = pc_immop;
        else
            Branch_target = aluout;
    end
endmodule


module mux_2to1_aluout_pcplus4 #(parameter width = 32)(
    input  [31:0] alu_out,
    input  [31:0] pc_plus4,
    input         Branchjalx,
    output reg [31:0] aluout_pcplus4
);
    always @(*) begin
        if (!Branchjalx)
            aluout_pcplus4 = alu_out;
        else
            aluout_pcplus4 = pc_plus4;
    end
endmodule


module mux_2to1pc #(parameter width = 32)(
    input  [31:0] pc_plus4,
    input  [31:0] branch_target,
    input         branch_outcome,
    output [31:0] pc_in
);
    assign pc_in = branch_outcome ? branch_target : pc_plus4;
endmodule


module mux_result #(parameter width = 32)(
    input  [31:0] read_data,
    input  [31:0] mux_2to1_aluout_pcplus4,
    input         memtoreg,
    output reg [31:0] result
);
    always @(*) begin
        if (!memtoreg)
            result = mux_2to1_aluout_pcplus4;
        else
            result = read_data;
    end
endmodule


module alu #(parameter width = 32)(
    input  [width-1:0] srcA,
    input  [width-1:0] srcB,
    input  [3:0] ALU_control,
    output reg [width-1:0] ALU_Out,
    output reg zero
);

    reg [8:0] sum3, sum2, sum1, sum0;

    always @(*) begin
        case (ALU_control)
            4'd0: ALU_Out = srcA + srcB; // ADD
            4'd1: ALU_Out = srcA - srcB; // SUB
            4'd2: ALU_Out = srcA & srcB; // AND
            4'd3: ALU_Out = srcA >> srcB[4:0]; // SRL
            4'd4: ALU_Out = ($signed(srcA) < $signed(srcB)) ? 32'd1 : 32'd0; // SLT

            // --- IMPLEMENTED add_v ---
            4'd5: ALU_Out =
                { (srcA[31:24] + srcB[31:24]),
                  (srcA[23:16] + srcB[23:16]),
                  (srcA[15:8]  + srcB[15:8]),
                  (srcA[7:0]   + srcB[7:0]) };

            // --- IMPLEMENTED avg_v ---
            4'd6: begin
                sum3 = {1'b0, srcA[31:24]} + {1'b0, srcB[31:24]};
                sum2 = {1'b0, srcA[23:16]} + {1'b0, srcB[23:16]};
                sum1 = {1'b0, srcA[15:8]}  + {1'b0, srcB[15:8]};
                sum0 = {1'b0, srcA[7:0]}   + {1'b0, srcB[7:0]};
                ALU_Out = { sum3[8:1], sum2[8:1], sum1[8:1], sum0[8:1] };
            end

            default: ALU_Out = 32'b0;
        endcase

        zero = (ALU_Out == 32'b0);
    end
endmodule


module ctrl_unit(
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg ALU_Src,
    output reg Reg_Write,
    output reg Mem_Write,
    output reg Branchbeq,
    output reg Branchjal,
    output reg Branchjalr,
    output reg memtoreg,
    output reg [3:0] ALU_Control,
    output reg [2:0] immcontrol
);

    localparam OP_RTYPE  = 7'b0110011;
    localparam OP_CUSTOM = 7'b0001011;
    localparam OP_ITYPE  = 7'b0010011;
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_JAL    = 7'b1101111;

    always @(*) begin
        ALU_Src     = 1'b0;
        Reg_Write   = 1'b0;
        Mem_Write   = 1'b0;
        Branchbeq   = 1'b0;
        Branchjal   = 1'b0;
        Branchjalr  = 1'b0;
        memtoreg    = 1'b0;
        ALU_Control = 4'd0;
        immcontrol  = 3'd0;

        case (opcode)

            OP_RTYPE: begin
                ALU_Src    = 1'b0;
                Reg_Write  = 1'b1;
                immcontrol = 3'd0;

                case (funct3)
                    3'b000:
                        ALU_Control = (funct7 == 7'b0100000) ? 4'd1 : 4'd0;
                    3'b111: ALU_Control = 4'd2;
                    3'b101: ALU_Control = 4'd3;
                    default: ALU_Control = 4'd0;
                endcase
            end

            OP_CUSTOM: begin
                Reg_Write = 1'b1;

                case (funct3)
                    3'b000: ALU_Control = 4'd5; // add_v
                    3'b001: ALU_Control = 4'd6; // avg_v
                    default: ALU_Control = 4'd0;
                endcase
            end

            OP_ITYPE: begin
                ALU_Src    = 1'b1;
                Reg_Write  = 1'b1;
                immcontrol = 3'd1;
                ALU_Control = 4'd0;
            end

            OP_BRANCH: begin
                Branchbeq  = 1'b1;
                immcontrol = 3'd4;

                case (funct3)
                    3'b000: ALU_Control = 4'd1; // BEQ
                    3'b100: ALU_Control = 4'd4; // BLT
                    default: ALU_Control = 4'd1;
                endcase
            end

            OP_LOAD: begin
                ALU_Src    = 1'b1;
                Reg_Write  = 1'b1;
                memtoreg   = 1'b1;
                immcontrol = 3'd1;
                ALU_Control = 4'd0;
            end

            OP_STORE: begin
                ALU_Src    = 1'b1;
                Mem_Write  = 1'b1;
                immcontrol = 3'd3;
                ALU_Control = 4'd0;
            end

            OP_JAL: begin
                Reg_Write  = 1'b1;
                Branchjal  = 1'b1;
                immcontrol = 3'd5;
            end

            OP_JALR: begin
                ALU_Src    = 1'b1;
                Reg_Write  = 1'b1;
                Branchjalr = 1'b1;
                immcontrol = 3'd1;
            end

        endcase
    end
endmodule


module imm_decode(
    input  [2:0]  immcontrol,
    input  [31:0] instruction,
    output reg [31:0] Immop
);
    always @(*) begin
        case (immcontrol)
            3'd1: Immop = {{20{instruction[31]}}, instruction[31:20]}; // I-type
            3'd2: Immop = {instruction[31:12], 12'b0};                 // U-type
            3'd3: Immop = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type
            3'd4: Immop = {{19{instruction[31]}},
                           instruction[31],
                           instruction[7],
                           instruction[30:25],
                           instruction[11:8],
                           1'b0}; // B-type
            3'd5: Immop = {{11{instruction[31]}},
                           instruction[31],
                           instruction[19:12],
                           instruction[20],
                           instruction[30:21],
                           1'b0}; // J-type
            default: Immop = 32'd0;
        endcase
    end
endmodule


module data_mem(
    input clk,
    input we,
    input  [31:0] address,
    input  [31:0] wd,
    output [31:0] rd
);
    reg [31:0] RAM[0:63];

    initial begin
        $readmemh("memfile_data.hex", RAM);
    end

    assign rd = RAM[address[31:2]];

    always @(posedge clk) begin
        if (we)
            RAM[address[31:2]] <= wd;
    end
endmodule


module inst_mem(
    input  [5:0] address,
    output [31:0] rd
);
    reg [31:0] RAM[0:63];

    initial begin
        $readmemh("memfile_inst.hex", RAM);
    end

    assign rd = RAM[address];
endmodule


module MUX2to1_DataMemory(
    input  [31:0] input0,
    input  [31:0] input1,
    input         select,
    output [31:0] out
);
    assign out = (select) ? input1 : input0;
endmodule

`default_nettype wire
