/**
 * riscv_decode.sv
 *
 * RISC-V 32-bit Processor
 *
 *
 * This file contains the implementation of the RISC-V decoder.
 *
 * This takes in information about the current RISC-V instruction and produces
 * the appropriate contol signals to get the processor to execute the current
 * instruction.
 *
 * Revision History:
 *  - 1/31/17: Added in missing ctrl_we signal to the ADD decoding.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "internal_defines.vh"      // Definition of ALU operations

// Control lines for the data-path. Gives deciding
// signals to guide the ALU and other hardware
module riscv_decode
    (input  opcode_t                opcode,
     input  rtype_int_funct3_t      rtype_int_funct3,
     input  itype_int_funct3_t      itype_int_funct3,
     input  rtype_funct7_t          rtype_funct7,
     input  itype_funct12_t         sys_funct12,
     output logic                   ctrl_we, ctrl_Sys, ctrl_RI,
     output logic [8:0]             dec__sel, 
     output logic [2:0]             inst_type);

     assign inst_type = dec__sel[8:6];

     // This logic decides the control lines based on the instruction types.
     // Sets control line values to the internal defines defined in the 
     // included file.
     always_comb begin
        // Various control signals going to data-path
        dec__sel = 9'hx;
        ctrl_we = 1'b0;
        ctrl_Sys = 1'b0;
        ctrl_RI = 1'b0;

        // Based on the opcode of the instruction
        case (opcode)
            // General R-type instruction
            OP_OP: begin
                // Case on the funct7 field
                case (rtype_funct7)
                    // 7-bit function code for a general R-type integer operation
                    FUNCT7_INT: begin
                        case (rtype_int_funct3)
                            FUNCT3_ADD: begin
                                dec__sel = `DEC_ADD;
                                ctrl_we = 1'b1;
                            end

                            FUNCT3_SLL: begin
                                dec__sel = `DEC_SLL;
                                ctrl_we = 1'b1;            
                            end

                            FUNCT3_SLT: begin
                                dec__sel = `DEC_SLT;
                                ctrl_we = 1'b1;
                            end

                            FUNCT3_SLTU: begin
                                dec__sel = `DEC_SLTU;
                                ctrl_we = 1'b1; 
                            end

                            FUNCT3_XOR: begin
                                dec__sel = `DEC_XOR;
                                ctrl_we = 1'b1;
                            end

                            FUNCT3_SRL: begin 
                                dec__sel = `DEC_SRL;
                                ctrl_we = 1'b1;
                            end 

                            FUNCT3_OR: begin
                                dec__sel = `DEC_OR;
                                ctrl_we = 1'b1;
                            end

                            FUNCT3_AND: begin
                                dec__sel = `DEC_AND;
                                ctrl_we = 1'b1;
                            end

                            default: begin
                                ctrl_RI = 1'b1;
                            end
                        endcase
                    end
  
                    // Alternate int types
                    FUNCT7_ALT_INT: begin
                        case (rtype_int_funct3)                         
                            FUNCT3_SUB: begin
                                dec__sel = `DEC_SUB;
                                ctrl_we = 1'b1;
                            end

                            FUNCT3_SRA: begin
                                dec__sel = `DEC_SRA;
                                ctrl_we = 1'b1;
                            end

                            default: begin
                                ctrl_RI = 1'b1;
                            end                        
                        endcase
                    end

                    default: begin
                        ctrl_RI = 1'b1;
                    end
                endcase
            end

            // General I-type arithmetic operation
            OP_IMM: begin
                // Case on the funct3 field of the instruction
                case (itype_int_funct3)
                    FUNCT3_ADDI: begin
                        dec__sel = `DEC_ADDI;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_SLTI: begin
                        dec__sel = `DEC_SLTI;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_SLTIU: begin
                        dec__sel = `DEC_SLTIU;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_XORI: begin
                        dec__sel = `DEC_XORI;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_ORI: begin
                        dec__sel = `DEC_ORI;
                        ctrl_we = 1'b1; 
                    end  

                    FUNCT3_ANDI: begin
                        dec__sel = `DEC_ANDI;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_SLLI: begin
                        dec__sel = `DEC_SLLI;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_SRLI_SRAI: begin
                        if (rtype_funct7 == FUNCT7_SRLI) begin
                            dec__sel = `DEC_SRLI;
                            ctrl_we = 1'b1; 
                        end
                        else begin
                            dec__sel = `DEC_SRAI;
                            ctrl_we = 1'b1;
                        end
                    end

                    default: begin
                        ctrl_RI = 1'b1;
                    end
                endcase
            end

            // Load type instructions
            OP_LOAD: begin
                case (itype_int_funct3)
                    FUNCT3_LB: begin
                        //ALU ADD
                        dec__sel = `DEC_LB;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_LH: begin
                        //ALU ADD 
                        dec__sel = `DEC_LH;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_LW: begin
                        //ALU ADD
                        dec__sel = `DEC_LW;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_LBU: begin
                        //ALU ADD
                        dec__sel = `DEC_LBU;
                        ctrl_we = 1'b1;
                    end

                    FUNCT3_LHU: begin
                        //ALU ADD
                        dec__sel = `DEC_LHU;
                        ctrl_we = 1'b1;
                    end

                    default: begin
                        ctrl_RI = 1'b1;
                    end
                endcase
            end

            // Store type instructions 
            OP_STORE: begin //ALL ARE ALU ADD
                case (itype_int_funct3)
                    FUNCT3_SB: begin
                        dec__sel = `DEC_SB;
                        ctrl_we = 1'b0;
                    end

                    FUNCT3_SH: begin
                        dec__sel = `DEC_SH;
                        ctrl_we = 1'b0;
                    end

                    FUNCT3_SW: begin
                        dec__sel = `DEC_SW;
                        ctrl_we = 1'b0;
                    end

                    default: begin
                        ctrl_RI = 1'b1;
                    end
                endcase
            end

            // Load upper immediate
            OP_LUI: begin
                dec__sel = `DEC_LUI;
                ctrl_we = 1'b1;                   
            end

            // add upper immediate to PC
            OP_AUIPC: begin
                dec__sel = `DEC_AUIPC;
                ctrl_we = 1'b1;
            end

            // Jump and link 
            OP_JAL: begin
                dec__sel = `DEC_JAL;
                ctrl_we = 1'b1; 
            end

            // Jump and link register 
            OP_JALR: begin //(ALU ADD)
                dec__sel = `DEC_JALR;
                ctrl_we = 1'b1;
            end

            // branch instructions 
            OP_BRANCH: begin
                case (itype_int_funct3)       
                    FUNCT3_BEQ: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BEQ;
                    end

                    FUNCT3_BNE: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BNE;
                    end

                    FUNCT3_BLT: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BLT;
                    end

                    FUNCT3_BGE: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BGE;
                    end

                    FUNCT3_BLTU: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BLTU;
                    end

                    FUNCT3_BGEU: begin
                        ctrl_we = 1'b0;
                        dec__sel = `DEC_BGEU;
                    end
                endcase 
            end
      
            // General system operation
            OP_SYSTEM: begin
                case (sys_funct12)
                    FUNCT12_ECALL: begin
                        ctrl_Sys = 1'b1;
                        dec__sel = `DEC_ECALL;
                    end
                endcase
            end

            default: begin
                ctrl_RI = 1'b1;
            end
        endcase
    end

endmodule: riscv_decode
