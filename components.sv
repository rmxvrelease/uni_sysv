module mux32_1(
  input logic [31:0] s0,
  input logic [31:0] s1,
  input logic c,
  output logic [31:0] o
);
  assign o = (c) ? s1 : s0;
endmodule


module ula(
  input logic [31:0] a,
  input logic [31:0] b,
  input logic [4:0] op,
  output logic [31:0] out,
  output logic zero
);
  assign zero = ~|(a-b);
  always_comb begin
    case (op)
      5'b00000: out <= a + b;
      5'b00001: out <= a - b;
      5'b00010: out <= a & b;
      5'b00011: out <= a | b;
      5'b00100: begin
        if($signed(a) < $signed(b)) begin
            out <= 32'b1;
        end else begin
            out <= 32'b0;
        end
      end
      default: out <= 32'b0;
    endcase
  end
endmodule


module adder(
input logic [31:0] a,
input logic [31:0] b,
output logic [31:0] out
);
  assign out = a + b;
endmodule


module plus_four(
  input logic [31:0] a,
  output logic [31:0]out
);
  assign out = a + 32'b00000000000000000000000000000001;
endmodule


module imediato(
    input  logic [31:0] instr,
    output logic [31:0] imm
); 
always_comb begin
    case (instr[6:0])
        7'b1101111: imm = { {11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0 }; // jal
        7'b0100011: imm = { {20{instr[31]}}, instr[31:25], instr[11:7] }; // sw
        7'b1100011: imm = { {19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };  // beq
        7'b0110111: imm = {instr[31:12], 12'b0}; // lui
        7'b0000011,  // lw
        7'b1100111,  // jalr
        7'b0010011:  imm = { {20{instr[31]}}, instr[31:20] };
        default: imm = 32'b0;
    endcase
end
endmodule


module regfile(
    input clk,
    input rst,
    input we,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] disp,
    input [4:0] rd,
    input [31:0] wd,
    output [31:0] rd1,
    output [31:0] rd2,
    output [31:0] rdisp
);

    reg [31:0] regs [31:0];
	 
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 32; i = i + 1)
                regs[i] <= 32'b0;
            regs[2] <= 32'h100103FC;
        end 
        else if (we && rd != 0)
            regs[rd] <= wd;
    end
	 
    assign rd1 = (rs1 == 0) ? 32'b0 : regs[rs1];
    assign rd2 = (rs2 == 0) ? 32'b0 : regs[rs2];
    assign rdisp = (disp == 0) ? 32'b0 : regs[disp];

endmodule


module control_unit(
	input logic [31:0] instruction,
	output logic mem_to_reg,
	output logic write_to_mem, 
	output logic branch,
	output logic [4:0] alu_op,
	output logic alu_origin,
	output logic write_to_reg,
  output logic jal,
  output logic imd_to_reg
);
	always_comb begin
    case (instruction[6:0])
    7'b0110011: begin
      // add, and, sub, or, slt
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b0;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
      case (instruction[31:25])
      7'b0100000: begin
        alu_op <= 5'b00001;
      end
      default: begin
        case (instruction[14:12])
          3'b000: begin
            alu_op <= 5'b00000;
          end
          3'b010: begin
            alu_op <= 5'b00100;
          end
          3'b110: begin
            alu_op <= 5'b00011;
          end
          default: begin
            alu_op <= 5'b00010;
          end
        endcase
        end
      endcase
    end
    7'b0000011: begin
      // lw
      mem_to_reg <= 1'b1;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
    end
    7'b0100011: begin
      // sw
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b1;
      write_to_reg <= 1'b0;
      alu_origin <= 1'b1;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
    end
    7'b1100011: begin
      // beq
      mem_to_reg <= 1'b0;
      branch <= 1'b1;
      alu_op <= 5'b00001;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b0;
      alu_origin <= 1'b0;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
    end
    7'b0010011: begin
      //addi
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
    end
    7'b1101111: begin
      // jal (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
      jal <= 1'b1;
      imd_to_reg <= 1'b0;
    end
    7'b1100111: begin
      // jalr (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
      jal <= 1'b1;
      imd_to_reg <= 1'b0;
    end
    7'b0110111: begin
      // lui (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b1;
      alu_origin <= 1'b1;
      jal <= 1'b0;
      imd_to_reg <= 1'b1;
    end
    default: begin
      // lui (não implementado)
      mem_to_reg <= 1'b0;
      branch <= 1'b0;
      alu_op <= 5'b00000;
      write_to_mem <= 1'b0;
      write_to_reg <= 1'b0;
      alu_origin <= 1'b0;
      jal <= 1'b0;
      imd_to_reg <= 1'b0;
    end
    endcase
	end
endmodule


module program_counter(
  input logic clk,
  input logic [31:0] in,
  output logic [31:0] out
);
  reg [31:0] pc_value;
  always @(posedge clk) begin
    pc_value <= in;
  end
  assign out = pc_value;
endmodule


module mem_placeholder(
  input logic clk,
  input logic [31:0] add,
  output logic [31:0] ins
);
  always @(posedge clk) begin
    case (add)
    32'b00000: ins = 32'b00000000000100001000000010010011;
    32'b00100: ins = 32'b00000000000100010000000100010011;
    32'b01000: ins = 32'b00000000001000001000000110110011;
    default: ins = 32'b0;
    endcase
  end
endmodule

module reg_inst_split(
  input logic [31:0] inst,
  output logic [4:0] rs1,
  output logic [4:0] rs2,
  output logic [4:0] rd
);
  assign rs1 = inst[19:15];
  assign rs2 = inst[24:20];
  assign rd = inst[11:7];
endmodule

module bus_32_to_9(
  input logic [31:0] in,
  output logic [9:0] out
);
  assign out = in[9:0];
endmodule
