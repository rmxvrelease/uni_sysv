module mux32_1(
  input logic [31:0] s0,
  input logic [31:0] s1,
  input logic c,
  output logic [31:0] o
);
  assign o = (c) ? s0 : s1;
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
  output logic out
);
  assign out = a + 31'b0100;
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
