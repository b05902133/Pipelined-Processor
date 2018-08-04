////
//// Internal signal constants
////

// DECODE SEL LINE VDECES
`define DEC_ADD      9'b000000000
`define DEC_SUB      9'b000000001
`define DEC_SLL      9'b000000010
`define DEC_SRL      9'b000000011
`define DEC_SRA      9'b000000100
`define DEC_XOR      9'b000000101
`define DEC_SLT      9'b000000110
`define DEC_SLTU     9'b000000111
`define DEC_OR       9'b000001000
`define DEC_AND      9'b000001001
`define DEC_ADDI     9'b001001010
`define DEC_SUBI     9'b001001011
`define DEC_SLLI     9'b001001100
`define DEC_SRLI     9'b001001101
`define DEC_SRAI     9'b001001110
`define DEC_XORI     9'b001001111
`define DEC_SLTI     9'b001010000
`define DEC_SLTIU    9'b001010001
`define DEC_ORI      9'b001010010
`define DEC_ANDI     9'b001010011
`define DEC_LB       9'b010010100
`define DEC_LH       9'b010010101
`define DEC_LW       9'b010010110
`define DEC_LBU      9'b010010111
`define DEC_LHU      9'b010011000
`define DEC_SB       9'b011011001
`define DEC_SH       9'b011011010
`define DEC_SW       9'b011011011
`define DEC_LUI      9'b100011100
`define DEC_AUIPC    9'b101011101
`define DEC_BEQ      9'b110011110
`define DEC_BNE      9'b110011111
`define DEC_BLT      9'b110100000
`define DEC_BGE      9'b110100001
`define DEC_BLTU     9'b110100010
`define DEC_BGEU     9'b110100011
`define DEC_JAL      9'b111100100
`define DEC_JALR     9'b111100101
`define DEC_ECALL    9'b111100110

// To group the type of instructions
`define R_TYPE      3'b000
`define I_TYPE      3'b001
`define L_TYPE      3'b010
`define S_TYPE      3'b011
`define LUI_TYPE    3'b100
`define AUIPC_TYPE  3'b101
`define B_TYPE      3'b110
`define J_TYPE      3'b111

// Saturation Counter values
`define ST  2'b11
`define WT  2'b10
`define WNT 2'b01
`define SNT 2'b00 









