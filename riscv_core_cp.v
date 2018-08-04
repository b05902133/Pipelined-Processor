/**
 * Mario Srouji     msrouji
 * Karan Dhabalia   kdhabali
 *
 * riscv_core.sv
 *
 * RISC-V 32-bit Processor
 *
 * ECE 18-447
 * Carnegie Mellon University
 *
 * This is the core part of the processor, and is responsible for executing the
 * current instruction and updating the CPU state appropriately.
 *
 * This is where you can start to add code and make modifications to
 * implement the processor. You can add any additional files or change and
 * delete files as you need to implement the simulator, under the src directory.
 *
 * The Makefile will automatically find any files you add, provided they are
 * under the src directory and have either a *.v, *.vh, or *.sv extension. The
 * compiler supports both Verilog and System Verilog constructs and syntax, so
 * you can write either Verilog or System Verilog code, or mix both.
 **/

/*----------------------------------------------------------------------------*
 *  You may edit this file and add or change any files in the src directory.  *
 *----------------------------------------------------------------------------*/

//////
////// RISC-V 447: A single-cycle RISC-V ISA simulator
//////

`include "riscv_abi.vh"             // ABI registers and definitions
`include "riscv_isa.vh"             // RISC-V ISA definitions
`include "internal_defines.vh"      // Definition of ALU operations

////
//// The RISC-V standalone processor module
////
////   clk              (input)  - The clock
////   inst_addr        (output) - Address of instruction to load
////   inst             (input)  - Instruction from memory
////   inst_excpt       (input)  - inst_addr not valid
////   mem_addr         (output) - Address of data to load
////   mem_data_store   (output) - Data for memory store
////   mem_data_load    (input)  - Data from memory load
////   mem_write_en     (output) - Memory write mask
////   mem_excpt        (input)  - mem_addr not valid
////   halted           (output) - Processor halted
////   reset            (input)  - Reset the processor
////

module riscv_core(
    // Outputs
    inst_addr, mem_addr, mem_data_store, mem_write_en, halted,
    // Inputs
    clk, inst_excpt, mem_excpt, inst, mem_data_load, rst_b
    );

    /* #################################################### Intitializations */

    parameter text_start  = `USER_TEXT_START; /* Initial value of $pc */

    // Core Interface
    input logic         clk, inst_excpt, mem_excpt;
    output logic [29:0] inst_addr;
    output logic [29:0] mem_addr;
    input  logic [31:0] inst[3:0];
    input logic [31:0]  mem_data_load;
    output logic [31:0] mem_data_store;
    output logic [3:0]  mem_write_en;
    output logic        halted;
    input logic         rst_b;

    // Internal signals
    /*
    logic [31:0]  pc, pc_plus_four, pc_jump_branch, nextpc;
    logic         exception_halt, syscall_halt, internal_halt, bcond_taken;
    logic         load_epc, load_bva, load_bva_sel, load_ex_regs;
    logic [31:0]  rs1_data[1:0], rs2_data[1:0], rd_data[1:0];
    logic [31:0]  alu__out, r_a0;
    logic [31:0]  epc, cause, bad_v_addr;
    logic [4:0]   cause_code;
    logic [31:0] alu__op2;
    */    

    // Decode signals
    opcode_t      dcd_op_I0;
    logic [6:0]   dcd_funct7_I0;
    logic [4:0]   dcd_rs1_I0, dcd_rs2_I0;
    logic [2:0]   dcd_funct3_I0;
    logic [11:0]  dcd_funct12_I0;

    opcode_t      dcd_op_I1;
    logic [6:0]   dcd_funct7_I1;
    logic [4:0]   dcd_rs1_I1, dcd_rs2_I1;
    logic [2:0]   dcd_funct3_I1;
    logic [11:0]  dcd_funct12_I1;

    // Decoder signals 
    /*
    logic [8:0]  dec__sel;       // From Decoder of riscv_decode.v
    logic        ctrl_RI;        // From Decoder of riscv_decode.v
    logic        ctrl_Sys;       // From Decoder of riscv_decode.v
    logic        ctrl_we;        // From Decoder of riscv_decode.v
    logic [2:0]  inst_type;       // Assigned here, uses dec__sel from decoder
    */

    // Sign Extender Signals
    //logic [31:0] se_offset;

    // Load Extender Signals
    //logic [31:0] extended_load;

    // Pipeline registers defines
    /*
    logic pc_enable;

    logic [61:0] btb_out_p0;
    logic [31:0] pc_plus_four_p0, ir_p0, pc_p0;
    logic pc_plus_four_en0, ir_en0, pc_en0, btb_en0;
     
    logic [31:0] pc_plus_four_reg_in_IF, inst_reg_in_IF, pc_reg_in_IF;
    logic [61:0] btb_out_reg_in_IF;
    logic bubble_IF;

    logic [61:0] btb_out_p1;
    logic [31:0] pc_plus_four_p1, rs1_data_p1, rs2_data_p1, ir_p1, pc_p1;
    logic [31:0] se_offset_p1;
    logic [14:0] decode_out_p1;
    logic decode_out_en1, pc_plus_four_en1, rs1_data_en1, rs2_data_en1,ir_en1;
    logic pc_en1, se_offset_en1, btb_en1;

    logic [14:0] decode_pipeline_reg_in_ID;
    logic [31:0] pcplusfour_pipeline_reg_in_ID, rs1data_pipeline_reg_in_ID;
    logic [31:0] rs2data_pipeline_reg_in_ID, ir_pipeline_reg_in_ID;
    logic [31:0] pc_pipeline_reg_in_ID, seoffset_pipeline_reg_in_ID;
    logic [61:0] btb_out_pipeline_reg_in_ID;
    logic bubble_ID;

    logic [14:0] decode_out_p2;
    logic [31:0] rs2_data_p2;
    logic [31:0] alu__out_p2, ir_p2;
    logic decode_out_en2;
    logic rs2_data_en2, alu__out_en2, ir_en2;

    logic [31:0] ir_p3, alu__out_p3, extended_load_p3;
    logic [14:0] decode_out_p3;
    logic decode_out_en3;
    logic ir_en3, alu__out_en3, extended_load_en3;
    */

    // Stall Condition
    //logic stall_condition;

    // Forward Condition
    //logic [1:0] forward_rs1_data, forward_rs2_data;

    // Branch prediction Variables
    /*
    logic [61:0] btb_out, btb_in, compute_actual_outcome;
    logic        btb_we, mispredict;
    */

    // Decode the opcode, and registers for the instruction
    assign        dcd_op = opcode_t'(ir_p0[6:0]);
    assign        dcd_rs1 = ir_p0[19:15];
    assign        dcd_rs2 = ir_p0[24:20];
    assign        dcd_rd = ir_p0[11:7];

    // Decode the various function codes for the instruction
    assign        dcd_funct3 = ir_p0[14:12];
    assign        dcd_funct7 = ir_p0[31:25];
    assign        dcd_funct12 = ir_p0[31:20];

    /* #################################################### Intitializations */

    /* ########################################### Stall Condition Functions */

    // The following function calculates the rs1_field of a given inst
    function logic [4:0] rs1_field
        (input logic [31:0] instruction);

        return instruction[19:15];

    endfunction: rs1_field  

    // The following function is used to check if the instruction
    // requires the rs1 field
    function logic use_rs1
      (input logic [14:0] decode_out,
       input logic [31:0] instruction);
      
      logic [4:0] check_x0;
      logic [2:0] inst_type;
      logic [8:0] dec__sel;

      check_x0 = rs1_field(instruction);

      inst_type = decode_out[5:3];
      dec__sel = decode_out[14:6];

      case(inst_type)
          `R_TYPE: return check_x0 && 1'b1; 
          `I_TYPE: return check_x0 && 1'b1;
          `L_TYPE: return check_x0 && 1'b1;
          `S_TYPE: return check_x0 && 1'b1;
          `B_TYPE: return check_x0 && 1'b1;
          `J_TYPE: begin
            if (dec__sel == `DEC_JALR)
              return check_x0 && 1'b1;
            else
              return check_x0 && 1'b0;
          end
          `AUIPC_TYPE: return check_x0 && 1'b0;
          `LUI_TYPE: return check_x0 && 1'b0;
      endcase

    endfunction: use_rs1
 
    // The following function calculates the rs2_field of a given inst
    function logic [4:0] rs2_field
        (input logic [31:0] instruction);

        return instruction[24:20];

    endfunction: rs2_field  

    // The following function is used to check if the instruction
    // requires the rs2 field
    function logic use_rs2
      (input logic [2:0] inst_type,
       input logic [31:0] instruction);
      
      logic [4:0] check_x0;
      check_x0 = rs2_field(instruction);
      case(inst_type)
          `R_TYPE: return check_x0 && 1'b1; 
          `I_TYPE: return check_x0 && 1'b0;
          `L_TYPE: return check_x0 && 1'b1;
          `S_TYPE: return check_x0 && 1'b1;
          `B_TYPE: return check_x0 && 1'b1;
          `J_TYPE: return check_x0 && 1'b0;
          `AUIPC_TYPE: return check_x0 && 1'b0;
          `LUI_TYPE: return check_x0 && 1'b0;
      endcase

    endfunction: use_rs2

    // The following function calculates the rd_field of a given inst
    function logic [4:0] rd_field
        (input logic [31:0] instruction);

        return instruction[11:7];

    endfunction: rd_field

    // This function determines whether or not
    // to stall
    function logic stall
        (input logic [31:0] inst_p0, inst_p1, 
         input logic [14:0] dec_out, dec_out_p1);

        logic [4:0] rs1_ID, rs2_ID;
        logic use_rs1_ID, use_rs2_ID;
        logic [4:0] rd_EX;
        logic mem_read_EX;

        rs1_ID = rs1_field(inst_p0);
        rs2_ID = rs2_field(inst_p0);

        use_rs1_ID = use_rs1(dec_out, inst_p0);
        use_rs2_ID = use_rs2(dec_out[5:3], inst_p0);

        rd_EX = rd_field(inst_p1);

        mem_read_EX = dec_out_p1[5:3] == `L_TYPE;

        if ((rs1_ID == rd_EX && use_rs1_ID && mem_read_EX) ||
            (rs2_ID == rd_EX && use_rs2_ID && mem_read_EX))
            return 1'b1;
        else
            return 1'b0;

    endfunction: stall
 
    /* ########################################### Stall Condition Functions */

    /* ######################################### Forward Condition Functions */

    // This function determines whether or not the rs1
    // value in upstream in the pipeline can be forwarded
    // to the decode phase. The function outputs a value depending
    // on which stage to forward from. This value gets fed into the 
    // muxes in the ID phase that determine which values to pass
    // through to the EX phase.
    function logic [1:0] forward_rs1_condition
        (input logic [31:0] inst_p0, inst_p1, inst_p2, inst_p3,
         input logic [14:0] dec_out_p1, dec_out_p2, dec_out_p3);       

        logic [4:0] rs1_ID, rd_EX, rd_MEM, rd_WB;
        logic reg_write_EX, reg_write_MEM, reg_write_WB;

        rs1_ID = rs1_field(inst_p0);
        rd_EX = rd_field(inst_p1);
        rd_MEM = rd_field(inst_p2);
        rd_WB = rd_field(inst_p3);

        reg_write_EX = dec_out_p1[2];
        reg_write_MEM = dec_out_p2[2];
        reg_write_WB = dec_out_p3[2];

        if (rs1_ID != 0 && rs1_ID == rd_EX && reg_write_EX)
            return 2'b01;
        else if (rs1_ID != 0 && rs1_ID == rd_MEM && reg_write_MEM)
            return 2'b10;
        else if (rs1_ID != 0 && rs1_ID == rd_WB && reg_write_WB)
            return 2'b11;
        else
            return 2'b00;

    endfunction: forward_rs1_condition

    // This function determines whether or not the rs2
    // value in upstream in the pipeline can be forwarded
    // to the decode phase. The function outputs a value depending
    // on which stage to forward from. This value gets fed into the 
    // muxes in the ID phase that determine which values to pass
    // through to the EX phase. 
    function logic [1:0] forward_rs2_condition
        (input logic [31:0] inst_p0, inst_p1, inst_p2, inst_p3,
         input logic [14:0] dec_out_p1, dec_out_p2, dec_out_p3);       

        logic [4:0] rs2_ID, rd_EX, rd_MEM, rd_WB;
        logic reg_write_EX, reg_write_MEM, reg_write_WB;

        rs2_ID = rs2_field(inst_p0);
        rd_EX = rd_field(inst_p1);
        rd_MEM = rd_field(inst_p2);
        rd_WB = rd_field(inst_p3);

        reg_write_EX = dec_out_p1[2];
        reg_write_MEM = dec_out_p2[2];
        reg_write_WB = dec_out_p3[2];

        if (rs2_ID != 0 && rs2_ID == rd_EX && reg_write_EX)
            return 2'b01;
        else if (rs2_ID != 0 && rs2_ID == rd_MEM && reg_write_MEM)
            return 2'b10;
        else if (rs2_ID != 0 && rs2_ID == rd_WB && reg_write_WB)
            return 2'b11;
        else
            return 2'b00;

    endfunction: forward_rs2_condition

    /* ######################################### Forward Condition Functions */

    /* ############################## For Debugging and Performance Analysis */

    /* When the design is compiled for simulation, the Makefile defines
     * SIMULATION_18447. You can use this to have code that is there for
     * simulation, but is discarded when the design is synthesized. Useful
     * for constructs that can't be synthesized. */

`ifdef SIMULATION_18447

  logic [15:0] cycles_elapsed, instructions_fetched, instructions_executed;
  logic [15:0] forward_conditionals_executed, backward_conditionals_executed;
  logic [15:0] forward_conditionals_without_btb;
  logic [15:0] backward_conditionals_without_btb;  
  logic [15:0] forward_conditionals_taken, backward_conditionals_taken;
  logic [15:0] forward_conditionals_taken_and_predicted_correctly;
  logic [15:0] backward_conditionals_taken_and_predicted_correctly;
  logic [15:0] forward_conditionals_not_taken_and_predicted_correctly;
  logic [15:0] backward_conditionals_not_taken_and_predicted_correctly;
  logic [15:0] forward_unconditionals_executed;
  logic [15:0] backward_unconditionals_executed;
  logic [15:0] forward_unconditionals_without_btb;
  logic [15:0] backward_unconditionals_without_btb;
  logic [15:0] forward_unconditionals_predicted_correctly;
  logic [15:0] backward_unconditionals_predicted_correctly;

  counter c0 (cycles_elapsed, 
              rst_b, 
              clk, 
              1'b1, 
              1'b1, 
              1'b0); 

  counter c1 (instructions_fetched, 
              rst_b, 
              clk, 
              !stall_condition, 
              1'b1, 
              1'b0); 

  counter c2 (instructions_executed, 
              rst_b, 
              clk, 
              (!stall_condition), 
              !mispredict, 
              1'b0);

  counter c3 (forward_conditionals_executed, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 < pc_jump_branch), 
              1'b1, 
              1'b0);

  counter c4 (backward_conditionals_executed, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 > pc_jump_branch), 
              1'b1, 
              1'b0); 

  counter c5 (forward_conditionals_without_btb, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 < pc_jump_branch && pc_p1[31:2] != btb_out_p1[61:32]),
              1'b1, 
              1'b0);

  counter c6 (backward_conditionals_without_btb, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 > pc_jump_branch && pc_p1[31:2] != btb_out_p1[61:32]), 
              1'b1, 
              1'b0); 

  counter c7 (forward_conditionals_taken, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 < pc_jump_branch && bcond_taken),
              1'b1, 
              1'b0); 

  counter c8 (backward_conditionals_taken, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 > pc_jump_branch && bcond_taken),
              1'b1, 
              1'b0); 

  counter c9 (forward_conditionals_taken_and_predicted_correctly, 
              rst_b, 
              clk, 
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 < pc_jump_branch && bcond_taken && !mispredict),
              1'b1, 
              1'b0); 

  counter c10 (backward_conditionals_taken_and_predicted_correctly, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 > pc_jump_branch && bcond_taken && !mispredict), 
              1'b1, 
              1'b0); 

  counter c11 (forward_conditionals_not_taken_and_predicted_correctly, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 < pc_jump_branch && !bcond_taken && !mispredict), 
              1'b1, 
              1'b0); 

  counter c12 (backward_conditionals_not_taken_and_predicted_correctly, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `B_TYPE && pc_p1 > pc_jump_branch && !bcond_taken && !mispredict), 
              1'b1, 
              1'b0); 

  counter c13 (forward_unconditionals_executed, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 < pc_jump_branch), 
              1'b1, 
              1'b0); 

  counter c14 (backward_unconditionals_executed, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 > pc_jump_branch), 
              1'b1, 
              1'b0); 

  counter c15 (forward_unconditionals_without_btb, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 < pc_jump_branch && pc_p1[31:2] != btb_out_p1[61:32]), 
              1'b1, 
              1'b0); 

  counter c16 (backward_unconditionals_without_btb, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 > pc_jump_branch && pc_p1[31:2] != btb_out_p1[61:32]), 
              1'b1, 
              1'b0); 

  counter c17 (forward_unconditionals_predicted_correctly, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 < pc_jump_branch && !mispredict), 
              1'b1, 
              1'b0); 

  counter c18 (backward_unconditionals_predicted_correctly, 
              rst_b, 
              clk,
              (decode_out_p1[5:3] == `J_TYPE && pc_p1 > pc_jump_branch && !mispredict), 
              1'b1, 
              1'b0); 

  always @(posedge clk) begin

    if (decode_out_p3[14:6] == `DEC_ECALL) begin
      $display("");
      $display("cycles_elapsed :                                            %d", cycles_elapsed);
      $display("instructions_fetched :                                      %d", instructions_fetched);
      $display("instructions_executed :                                     %d", instructions_executed);
      $display("forward_conditionals_executed :                             %d", forward_conditionals_executed);
      $display("backward_conditionals_executed :                            %d", backward_conditionals_executed);
      $display("forward_conditionals_without_btb :                          %d", forward_conditionals_without_btb);
      $display("backward_conditionals_without_btb :                         %d", backward_conditionals_without_btb);
      $display("forward_conditionals_taken :                                %d", forward_conditionals_taken);
      $display("backward_conditionals_taken :                               %d", backward_conditionals_taken);
      $display("forward_conditionals_taken_and_predicted_correctly :        %d", forward_conditionals_taken_and_predicted_correctly);
      $display("backward_conditionals_taken_and_predicted_correctly :       %d", backward_conditionals_taken_and_predicted_correctly);
      $display("forward_conditionals_not_taken_and_predicted_correctly :    %d", forward_conditionals_not_taken_and_predicted_correctly);
      $display("backward_conditionals_not_taken_and_predicted_correctly :   %d", backward_conditionals_not_taken_and_predicted_correctly);
      $display("forward_unconditionals_executed :                           %d", forward_unconditionals_executed);
      $display("backward_unconditionals_executed :                          %d", backward_unconditionals_executed);
      $display("forward_unconditionals_without_btb :                        %d", forward_unconditionals_without_btb);
      $display("backward_unconditionals_without_btb :                       %d", backward_unconditionals_without_btb); 
      $display("forward_unconditionals_predicted_correctly :                %d", forward_unconditionals_predicted_correctly);
      $display("backward_unconditionals_predicted_correctly :               %d", backward_unconditionals_predicted_correctly); 
      $display("");

    end

  end

`endif

/*
`ifdef SIMULATION_18447
    always @(posedge clk) begin
        $display("---------------------------------------------------------------------------------START\n");
        $display("PC VALUES:");
        $display("pc = %h, nextpc = %h, pc_p0 = %h, pc_p1 = %h\n",pc,nextpc, pc_p0,pc_p1);
        $display("INSTR VALUES:");
        $display("inst = %h, ir_p0 = %h, ir_p1 = %h, ir_p2 = %h, ir_p3 = %h\n",inst,ir_p0,ir_p1,ir_p2,ir_p3);
        $display("PC JUMP BRANCH");
        $display("pc_jump_branch = %h\n", pc_jump_branch);
        $display("DECODE INPUT VALUES:");
        $display("dcd_op = %h, dcd_funct3 = %h, dcd_funct7 = %h, dcd_funct12 = %h\n", dcd_op, dcd_funct3, dcd_funct7, dcd_funct12);
        $display("DEC__SEL VALUES:");
        $display("dec__sel = %h, dec__sel_p1 = %h, dec__sel_p2 = %h, dec__sel_p3 = %h\n", dec__sel, decode_out_p1[14:6], decode_out_p2[14:6],decode_out_p3[14:6]);
        $display("REG FILE VALUES:");
        $display("dcd_rs1 = %h, dcd_rs2 = %h, dcd_rd = %h\n", dcd_rs1, dcd_rs2, dcd_rd);
        $display("INST_TYPE VALUES:");
        $display("inst_type = %h, inst_type_p1 = %h, inst_type_p2 = %h, inst_type_p3 = %h\n", inst_type, decode_out_p1[5:3], decode_out_p2[5:3],decode_out_p3[5:3]);
        $display("CTRL_SYS VALUES:");
        $display("ctrl_Sys = %h, ctrl_Sys_p1 = %h, ctrl_Sys_p2 = %h, ctrl_Sys_p3 = %h\n", ctrl_Sys, decode_out_p1[1], decode_out_p2[1],decode_out_p3[1]);
        $display("SE_OFFSET VALUES:");
        $display("se_offset = %h, se_offset_p1 = %h\n", se_offset, se_offset_p1);
        $display("WRITE ENABLE OF REGISTER:");
        $display("ctrl_we = %h, ctrl_we_p1 = %h, ctrl_we_p2 = %h, ctrl_we_p3 = %h\n", ctrl_we, decode_out_p1[2], decode_out_p2[2], decode_out_p3[2]);
        $display("RS1_DATA:");
        $display("rs1_data = %h, rs1_data_p1= %h\n", rs1_data,rs1_data_p1);
        $display("RS2_DATA:");
        $display("rs2_data = %h, rs2_data_p1= %h, rs2_data_p2 = %h\n",rs2_data, rs2_data_p1, rs2_data_p2);
        $display("ALU INPUTS:");
        $display("alu__op1 = %h, alu__op2 = %h\n", rs1_data_p1, alu__op2);
        $display("ALU OUTPUTS:");
        $display("alu__out = %h, alu__out_p2 = %h, alu__out_p3 = %h\n", alu__out, alu__out_p2, alu__out_p3);
        $display("MEM_DATA_LOAD (OUT OF MEMORY):");
        $display("mem_data_load = %h\n", mem_data_load);
        $display("MEM_WRITE_EN");
        $display("mem_write_en = %h\n", mem_write_en);
        $display("EXTENDED LOAD:");
        $display("extended_load = %h, extended_load_p3 = %h\n", extended_load, extended_load_p3);
        $display("RD_DATA (VALUE GOES INTO REGFILE):");
        $display("rd_data = %h\n", rd_data);
        $display("HALT SIGNALS");
        $display("syscall_halt = %h, exception_halt = %h, internal_halt = %h, halted = %h\n", syscall_halt, exception_halt, internal_halt, halted);
        $display("CONDITIONS:");
        $display("stall_condition = %h, forward_rs1_data = %h, forward_rs2_data = %h\n", stall_condition, forward_rs1_data, forward_rs2_data);
        $display("BTB OUT VALUES:");
        $display("btb_out = %h, btb_out_p0 = %h, btb_out_p1 = %h\n", btb_out, btb_out_p0, btb_out_p1);
        $display("BTB IN/COMPUTE ACTUAL OUTCOME VALUE VALUES:");
        $display("btb_in = %h, compute_actual_outcome = %h\n", btb_in, compute_actual_outcome);
        $display("BTB WRITE ENABLE, MISPREDICT:");
        $display("btb_we = %h, mispredict = %h\n", btb_we, mispredict);
        $display("BRANCH CONDITION TAKEN:");
        $display("bcond_taken = %h\n", bcond_taken);
        $display("---------------------------------------------------------------------------------END\n");
        $display("---------------------------------------------------------------------------------END\n");
 
    end
`endif
*/
    /* ############################## For Debugging and Performance Analysis */

    /* ##################################################### Stall Condition */

    // Assign the stall condition 
    assign stall_condition = stall(ir_p0, ir_p1,
                                   {dec__sel, inst_type, ctrl_we, 
                                    ctrl_Sys, ctrl_RI}, decode_out_p1);

    /* ##################################################### Stall Condition */

    /* ################################################### Forward Condition */

    // Assign the forward condition to rs1
    assign forward_rs1_data = forward_rs1_condition(ir_p0, ir_p1, ir_p2, ir_p3,
                                                    decode_out_p1,
                                                    decode_out_p2, 
                                                    decode_out_p3);

    // Assign the forward condition to rs2
    assign forward_rs2_data = forward_rs2_condition(ir_p0, ir_p1, ir_p2, ir_p3,
                                                    decode_out_p1,
                                                    decode_out_p2, 
                                                    decode_out_p3);

    /* ################################################### Forward Condition */

    /* ############################################################ IF Phase */

    // Enable the pc register to go to the next instruction     
    assign pc_enable = ~internal_halt && !stall_condition;  

    // PC register
    register #(32, text_start) PCReg (pc, nextpc, clk, pc_enable, rst_b);


    // PC adder for pc + 4
    add_const #(4) PC4       (pc_plus_four, pc);

    // PC adder for pc + 8
    add_const #(8) PC8       (pc_plus_eight, pc);

    // The following mux is used to calculate next pc based on
    // the instruction type. The possible values are pc+4, 
    // jump and branch targets.
    nextpc_mux npc (.nextpc(nextpc),
                    .btb_out(btb_out),
                    .pc_plus_eight(pc_plus_eight),
                    .pc_plus_eight_p1(pc_plus_eight_p1),
                    .pc_jump_branch(pc_jump_branch), 
                    .pc(pc),
                    .mispredict(mispredict), 
                    .branch_taken(bcond_taken));

    // Address going into instruction memory (Decode phase)
    assign inst_addr = pc[31:2];

    // The following is a SRAM module of size 64-bit by 128 word.
    // It implements the BTB branch predictor.
    btbsram #(62, 7) btb (.rd_data(btb_out), 
                          .rd_idx(pc[8:2]), 
                          .wr_idx(pc_p1[8:2]),
                          .wr_data(btb_in), 
                          .wr_we(btb_we), 
                          .clk(clk), 
                          .rst_b(rst_b));

    // Interpret output of BTB as the following fields 
    assign btb_in = compute_actual_outcome;

    /* ############################################################ IF Phase */

    /* ############################################ IF/ID Pipeline Registers */

    // Enables for the pipeline registers 
    assign pc_en0 = !stall_condition;
    assign pc_plus_four_en0 = !stall_condition; 
    assign pc_plus_eight_en0 = !stall_condition;
    assign ir_en0 = !stall_condition;
    assign btb_en0 = !stall_condition;
    assign bubble_p0 = mispredict && btb_we;

    // Mux for passing a bubble through the IF phase for mispredictions.
    pipeline_register_mux #(32) PCMuxP0 (pc_mux_p0, pc, 
                                         32'b0, bubble_p0);

    // Mux for passing a bubble through the IF phase for mispredictions.
    pipeline_register_mux #(32) PC4MuxP0 (pc_plus_four_mux_p0, 
                                          pc_plus_four, 
                                          32'b0, bubble_p0);

    // Mux for passing a bubble through the IF phase for mispredictions.
    pipeline_register_mux #(32) PC8MuxP0 (pc_plus_eight_mux_p0, 
                                          pc_plus_eight, 
                                          32'b0, bubble_p0);
 
    // Mux for passing a bubble through the IF phase for mispredictions.
    pipeline_register_mux #(128) IRMuxP0 (ir_mux_p0, inst, 
                                          128'b0, bubble_p0);

    // Mux for passing a bubble through the IF phase for mispredictions.
    pipeline_register_mux #(62) BTBMuxP0 (btb_out_mux_p0, btb_out, 
                                          62'b0, bubble_p0);

    // Passes along the PC
    register #(32, 0) PCRegP0 (pc_p0, pc_mux_p0, 
                               clk, pc_en0, rst_b);

    // Passes along pc+4 value
    register #(32, 0) PC4RegP0 (pc_plus_four_p0, 
                                pc_plus_four_mux_p0,
                                clk, pc_plus_four_en0, rst_b);

    // Passes along pc+8 value
    register #(32, 0) PC8RegP0 (pc_plus_eight_p0, 
                                pc_plus_eight_mux_p0,
                                clk, pc_plus_eight_en0, rst_b);

    // Passes along instruction 
    register #(128, 0) IRRegP0 (ir_p0, ir_mux_p0,
                                clk, ir_en0, rst_b);

    // Passes along the BTB output 
    register #(62, 0) BTBRegP0 (btb_out_p0, btb_out_mux_p0,
                                clk, btb_en0, rst_b);

    /* ############################################ IF/ID Pipeline Registers */

    /* ############################################################ ID Phase */ 

    // Generates control signals for I0 of the datapath 
    riscv_decode DecI0(.opcode(dcd_op_I0),
                       .rtype_int_funct3(rtype_int_funct3_t'(dcd_funct3_I0)),
                       .itype_int_funct3(itype_int_funct3_t'(dcd_funct3_I0)),
                       .rtype_funct7(rtype_funct7_t'(dcd_funct7_I0)),
                       .sys_funct12(itype_funct12_t'(dcd_funct12_I0)),
                       .ctrl_we(ctrl_we_I0),
                       .ctrl_Sys(ctrl_Sys_I0),
                       .ctrl_RI(ctrl_RI_I0),
                       .dec__sel(dec__sel_I0), 
                       .inst_type(inst_type_I0));

    // Generates control signals for I1 of the datapath 
    riscv_decode DecI1(.opcode(dcd_op_I1),
                       .rtype_int_funct3(rtype_int_funct3_t'(dcd_funct3_I1)),
                       .itype_int_funct3(itype_int_funct3_t'(dcd_funct3_I1)),
                       .rtype_funct7(rtype_funct7_t'(dcd_funct7_I1)),
                       .sys_funct12(itype_funct12_t'(dcd_funct12_I1)),
                       .ctrl_we(ctrl_we_I1),
                       .ctrl_Sys(ctrl_Sys_I1),
                       .ctrl_RI(ctrl_RI_I1),
                       .dec__sel(dec__sel_I1), 
                       .inst_type(inst_type_I1));

    // Register File instantiation 
    regfile Rf (.rs1_data({rs1_data_I1, rs1_data_I0}), 
                .rs2_data({rs2_data_I1, rs1_data_I0}), 
                .rs1_num({dcd_rs1_I1, dcd_rs1_I0}),
                .rs2_num({dcd_rs2_I1, dcd_rs2_I0}), 
                .rd_num({dcd_rd_I1, dcd_rd_I0}), 
                .rd_data({rd_data_I1, rd_data_I0}), 
                .rd_we({decode_out_I1_p3[2], decode_out_I0_p3[2]}),
                .clk(clk), 
                .rst_b(rst_b), 
                .halted(halted));

    // sign extends values inside of the instruction. handles
    // all offsets depending on instruction type.
    sign_extender SeI0 (.se_offset(se_offset_I0), 
                        .inst(ir_p0[0]),
                        .inst_type(inst_type_I0), 
                        .dec__sel(dec__sel_I0));

    // sign extends values inside of the instruction. handles
    // all offsets depending on instruction type.
    sign_extender SeI1 (.se_offset(se_offset_I1), 
                        .inst(ir_p0[1]),
                        .inst_type(inst_type_I1), 
                        .dec__sel(dec__sel_I1));

    // Miscellaneous (Exceptions, syscalls, and halt)
    exception_unit EU(.exception_halt(exception_halt), .pc(pc_p0), 
                      .rst_b(rst_b),
                      .clk(clk), .load_ex_regs(load_ex_regs),
                      .load_bva(load_bva), .load_bva_sel(load_bva_sel),
                      .cause(cause_code),
                      .IBE(inst_excpt),
                      .DBE(1'b0),
                      .RI(ctrl_RI_I0),
                      .Ov(1'b0),
                      .BP(1'b0),
                      .AdEL_inst(pc_p0[1:0]?1'b1:1'b0),
                      .AdEL_data(1'b0),
                      .AdES(1'b0),
                      .CpU(1'b0));

    // Generates syscalls, most importantly halting the processor.
    syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc_p0), .clk(clk), 
                    .Sys(decode_out_I0_p3[1]), .r_a0(32'hA), .rst_b(rst_b));
    assign        internal_halt = exception_halt | syscall_halt;
    register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
    register #(32, 0) EPCReg(epc, pc_p0, clk, load_ex_regs, rst_b);
    register #(32, 0) CauseReg(cause,
                               {25'b0, cause_code, 2'b0},
                               clk, load_ex_regs, rst_b);
    register #(32, 0) BadVAddrReg(bad_v_addr, pc_p0, clk, load_bva, rst_b);

    /* ############################################################ ID Phase */

    /* ############################################# ID/EX Pipeline Register */

    // Enables for the pipeline registers 
    assign decode_out_I0_en1 = 1'b1;
    assign decode_out_I1_en1 = 1'b1;
    assign pc_en1 = 1'b1;
    assign pc_plus_four_en1 = 1'b1;
    assign pc_plus_eight_en1 = 1'b1;
    assign rs1_data_I0_en1 = 1'b1;
    assign rs1_data_I1_en1 = 1'b1;
    assign rs2_data_I0_en1 = 1'b1;
    assign rs2_data_I1_en1 = 1'b1;
    assign ir_en1 = 1'b1;
    assign se_offset_I0_en1 = 1'b1;
    assign se_offset_I1_en1 = 1'b1;
    assign btb_en1 = 1'b1;
    assign bubble_p1 = stall_condition || (mispredict && btb_we);

    // Either pass along the decode information, or a bubble
    pipeline_register_mux #(15) DecMuxI0P1 (decode_out_mux_I0_p1, 
                                            {dec__sel_I0, 
                                             inst_type_I0, 
                                             ctrl_we_I0,
                                             ctrl_Sys_I0, 
                                             ctrl_RI_I0},
                                            15'b0, 
                                            bubble_p1); 

    // Either pass along the decode information, or a bubble
    pipeline_register_mux #(15) DecMuxI1P1 (decode_out_mux_I1_p1, 
                                            {dec__sel_I1, 
                                             inst_type_I1, 
                                             ctrl_we_I1,
                                             ctrl_Sys_I1, 
                                             ctrl_RI_I1},
                                            15'b0, 
                                            bubble_p1); 

    // Either pass along the pc information, or a bubble
    pipeline_register_mux #(32) PCMuxP1 (pc_mux_p1, 
                                         pc_p0,
                                         32'b0, 
                                         bubble_p1); 

    // Either pass along the pc+4 information, or a bubble
    pipeline_register_mux #(32) PC4MuxP1 (pc_plus_four_mux_p1, 
                                          pc_plus_four_p0,
                                          32'b0, 
                                          bubble_p1); 

    // Either pass along the pc+8 information, or a bubble
    pipeline_register_mux #(32) PC8MuxP1 (pc_plus_eight_mux_p1, 
                                          pc_plus_eight_p0,
                                          32'b0, 
                                          bubble_p1); 

    // Either pass along the rs1 data information, EX forward data, 
    // MEM forward data, WB forward data, or a bubble.
    forwarding_mux RS1MuxI0P1 (rs1_data_mux_I0_p1, 
                               rs1_data_I0, 
                               alu__out_I0, 
                               extended_load, 
                               alu__out_I0_p2,   
                               rd_data_I0, 
                               bubble_p1, 
                               forward_rs1_data, 
                               decode_out_I0_p2[5:3]);

    // Either pass along the rs1 data information, EX forward data, 
    // MEM forward data, WB forward data, or a bubble.
    forwarding_mux RS1MuxI1P1 (rs1_data_mux_I1_p1, 
                               rs1_data_I1, 
                               alu__out_I1, 
                               extended_load, 
                               alu__out_I1_p2,   
                               rd_data_I1, 
                               bubble_p1, 
                               forward_rs1_data, 
                               decode_out_I1_p2[5:3]);

    // Either pass along the rs2 data information, EX forward data, 
    // MEM forward data, WB forward data, or a bubble.
    forwarding_mux RS2MuxI0P1 (rs2_data_mux_I0_p1, 
                               rs2_data_I0, 
                               alu__out_I0, 
                               extended_load,   
                               alu__out_I0_p2,
                               rd_data_I0, 
                               bubble_p1, 
                               forward_rs2_data, 
                               decode_out_I0_p2[5:3]);

    // Either pass along the rs2 data information, EX forward data, 
    // MEM forward data, WB forward data, or a bubble.
    forwarding_mux RS2MuxI1P1 (rs2_data_mux_I1_p1, 
                               rs2_data_I1, 
                               alu__out_I1, 
                               extended_load,   
                               alu__out_I1_p2,
                               rd_data_I1, 
                               bubble_p1, 
                               forward_rs2_data, 
                               decode_out_I1_p2[5:3]);

    // Either pass along the instruction information, or a bubble
    pipeline_register_mux #(32) IRMuxP1 (ir_mux_p1, 
                                         ir_p0,
                                         32'b0, 
                                         bubble_p1); 

    // Either pass along the sign extended offset information, or a bubble
    pipeline_register_mux #(32) SEOMuxI0P1 (se_offset_mux_I0_p1, 
                                            se_offset_I0,
                                            32'b0, 
                                            bubble_p1); 

    // Either pass along the sign extended offset information, or a bubble
    pipeline_register_mux #(32) SEOMuxI1P1 (se_offset_mux_I1_p1, 
                                            se_offset_I1,
                                            32'b0, 
                                            bubble_p1); 

    // Either pass along the BTB output or a bubble
    pipeline_register_mux #(62) BTBMuxP1 (btb_out_mux_p1,
                                          btb_out_p0, 
                                          62'b0, 
                                          bubble_p1); 

    // decode pipeline register
    register #(15, 0) DecRegI0P1 (decode_out_I0_p1,
                                  decode_out_mux_I0_p1, clk,
                                  decode_out_I0_en1, rst_b);

    // decode pipeline register
    register #(15, 0) DecRegI1P1 (decode_out_I1_p1,
                                  decode_out_mux_I1_p1, clk,
                                  decode_out_I1_en1, rst_b);

    // rs1 data pipeline register
    register #(32, 0) RS1RegI0P1 (rs1_data_I0_p1, rs1_data_mux_I0_p1,
                                  clk,
                                  rs1_data_I0_en1, rst_b);

    // rs1 data pipeline register
    register #(32, 0) RS1RegI1P1 (rs1_data_I1_p1, rs1_data_mux_I1_p1, 
                                  clk,
                                  rs1_data_I1_en1, rst_b);

    // rs2 data pipeline register
    register #(32, 0) RS2RegI0P1 (rs2_data_I0_p1, rs2_data_mux_I0_p1, 
                                  clk,
                                  rs2_data_I0_en1, rst_b);

    // rs2 data pipeline register
    register #(32, 0) RS2RegI1P1 (rs2_data_I1_p1, rs2_data_mux_I1_p1, 
                                  clk,
                                  rs2_data_I1_en1, rst_b);

    // instruction pipeline register
    register #(128, 0) IRRegP1 (ir_p1, ir_mux_p1, clk,
                                ir_en1, rst_b);

    // pc pipeline register 
    register #(32, 0) PCRegP1 (pc_p1, pc_mux_p1, clk,
                               pc_en1, rst_b);

    // pc+4 pipeline register 
    register #(32, 0) PC4RegP1 (pc_plus_four_p1, pc_plus_four_mux_p1, clk,
                                pc_plus_four_en1, rst_b);

    // pc+8 pipeline register 
    register #(32, 0) PC8RegP1 (pc_plus_eight_p1, pc_plus_eight_mux_p1, clk,
                                pc_plus_eight_en1, rst_b);

    // sign extended offset pipeline register 
    register #(32, 0) SEORegI0P1 (se_offset_I0_p1, se_offset_mux_I0_p1, 
                                  clk,
                                  se_offset_I0_en1, rst_b);

    // sign extended offset pipeline register 
    register #(32, 0) SEORegI1P1 (se_offset_I1_p1, se_offset_mux_I1_p1, 
                                  clk,
                                  se_offset_I1_en1, rst_b);

    // Passes along BTB output
    register #(62, 0) BTBRegP1 (btb_out_p1, btb_out_mux_p1, 
                                clk, btb_en1, rst_b);

    /* ############################################# ID/EX Pipeline Register */

    /* ############################################################ EX Phase */

    //  Adder for the jump and branch instructions 
    jump_branch_adder JumpAndBranch (pc_jump_branch, pc_p1, 
                                     se_offset_p1, 
                                     rs1_data_p1, 
                                     decode_out_p1[14:6]);

    //The following is an input to alu__op2 and is decided by a mux
    alu__op2_mux ALU2I0Mux (.alu__op2(alu__op2_I0), 
                            .inst_type(decode_out_I0_p1[5:3]), 
                            .rs2_data(rs2_data_I0_p1), 
                            .se_offset(se_offset_I0_p1));

    // ALU that takes in all instructions except ECALL, and computes 
    // necessary output
    riscv_ALU ALUI0 (.alu__out(alu__out_I0),
                     .bcond_taken(bcond_taken_I0),
                     .alu__op1(rs1_data_I0_p1),
                     .alu__op2(alu__op2_I0),
                     .dec__sel(decode_out_I0_p1[14:6]), 
                     .inst(ir_p1[0]), 
                     .pc(pc_p1), 
                     .pc_plus_four(pc_plus_four_p1));

    //The following is an input to alu__op2 and is decided by a mux
    alu__op2_mux ALU2I1Mux (.alu__op2(alu__op2_I1), 
                            .inst_type(decode_out_I1_p1[5:3]), 
                            .rs2_data(rs2_data_I1_p1), 
                            .se_offset(se_offset_I1_p1));

    // ALU that takes in all instructions except ECALL, and computes 
    // necessary output
    riscv_ALU ALUI1 (.alu__out(alu__out_I1),
                     .bcond_taken(bcond_taken_I1),
                     .alu__op1(rs1_data_I1_p1),
                     .alu__op2(alu__op2_I1),
                     .dec__sel(decode_out_I1_p1[14:6]), 
                     .inst(ir_p1[1]), 
                     .pc(pc_plus_four_p1), 
                     .pc_plus_four(pc_plus_eight_p1));

    // Compute actual outcome determines what the actual output
    // of the branch was and also determined what to feed into 
    // BTB.
    compute_actual_outcome CAOI0 (.btb_in(compute_actual_outcome_I0),
                                  .btb_out(btb_out_I0_p1),
                                  .pc_jump_branch(pc_jump_branch_I0),
                                  .pc_plus_four(pc_plus_four_p1),
                                  .pc(pc_p1),
                                  .branch_taken(bcond_taken_I0));

    // Compute actual outcome determines what the actual output
    // of the branch was and also determined what to feed into 
    // BTB.
    compute_actual_outcome CAOI1 (.btb_in(compute_actual_outcome_I1),
                                  .btb_out(btb_out_I1_p1),
                                  .pc_jump_branch(pc_jump_branch_I1),
                                  .pc_plus_four(pc_plus_eight_p1),
                                  .pc(pc_plus_four_p1),
                                  .branch_taken(bcond_taken_I1));

    // Check prediction checks whether what we precited was 
    // accurate or not
    mispredict_detector CheckPredictionI0 (.mispredict(mispredict_I0),
                                           .btb_out(btb_out_I0_p1),
                                           .pc(pc_p1),
                                           .pc_jump_branch(pc_jump_branch_I0),
                                           .branch_taken(bcond_taken_I0), 
                                           .btb_we(btb_we_I0));

    // Check prediction checks whether what we precited was 
    // accurate or not
    mispredict_detector CheckPredictionI1 (.mispredict(mispredict_I1),
                                           .btb_out(btb_out_I1_p1),
                                           .pc(pc_plus_four_p1),
                                           .pc_jump_branch(pc_jump_branch_I1),
                                           .branch_taken(bcond_taken_I1), 
                                           .btb_we(btb_we_I1));

    // Determines we for BTB SRAM.
    btb_write_logic BTBweI0 (btb_we_I0, decode_out_I0_p1[5:3]);

    // Determines we for BTB SRAM.
    btb_write_logic BTBweI1 (btb_we_I1, decode_out_I1_p1[5:3]);

    /* ############################################################ EX Phase */

    /* ########################################### EX/MEM Pipeline Registers */

    // Pipeline register enables     
    assign decode_out_I0_en2 = 1'b1;
    assign decode_out_I1_en2 = 1'b1;
    assign rs2_data_I0_en2 = 1'b1;
    assign rs2_data_I1_en2 = 1'b1;
    assign alu__out_I0_en2 = 1'b1;
    assign alu__out_I1_en2 = 1'b1;
    assign ir_en2 = 1'b1;

    // decode pipeline register
    register #(15, 0) DECRegI0P2 (decode_out_I0_p2, decode_out_I0_p1, 
                                  clk, decode_out_I0_en2, rst_b);

    // decode pipeline register
    register #(15, 0) DECRegI1P2 (decode_out_I1_p2, decode_out_I1_p1, 
                                  clk, decode_out_I1_en2, rst_b);

    // rs2 data pipeline register
    register #(32, 0) RS2RegI0P2 (rs2_data_I0_p2, rs2_data_I0_p1, 
                                   clk, rs2_data_I0_en2, rst_b);

    // rs2 data pipeline register
    register #(32, 0) RS2RegI1P2 (rs2_data_I1_p2, rs2_data_I1_p1, 
                                   clk, rs2_data_I1_en2, rst_b);

    // alu out pipeline register
    register #(32, 0) ALURegI0P2 (alu__out_I0_p2, alu__out_I0, 
                                  clk, alu__out_I0_en2, rst_b);
    // alu out pipeline register
    register #(32, 0) ALURegI1P2 (alu__out_I1_p2, alu__out_I1, 
                                  clk, alu__out_I1_en2, rst_b);

    // instruction pipeline register 
    register #(128, 0) IRRegP2 (ir_p2, ir_p1, 
                                clk, ir_en2, rst_b);

    /* ########################################### EX/MEM Pipeline Registers */

    /* ########################################################### MEM Phase */

    instruction_memory_usage IMU (instruction_number,
                                  decode_out_I0_p2[5:3], 
                                  decode_out_I1_p2[5:3]);

    // Address of memory location to access
    mem_addr_mux MemAddrMux (mem_addr, 
                             alu__out_I0_p2, 
                             alu__out_I1_p2, 
                             instruction_number);

    // This mux decides how to arrange the rs2_data in order
    // to satisfy storing the bytes in the correct order in memory
    // based on the lower bits of the address (since non-aligned
    // stores are allowed, but we can only access aligned addresses).
    mem_data_store_mux mdsmux (.mem_data_store(mem_data_store), 
                               .dec__sel(decode_out_p2[14:6]), 
                               .alu_lower_bits(alu__out_p2[1:0]), 
                               .rs2_data_I0(rs2_data_I0_p2), 
                               .rs2_data_I1(rs2_data_I1_p2), 
                               .instruction_number(instruction_number));

    // Decide what to set mem write enable to based on instruction 
    mem_write_mux mwm (.mem_write_en(mem_write_en),
                       .dec__sel_I0(decode_out_I0_p2[14:6]),
                       .dec__sel_I1(decode_out_I1_p2[14:6]),
                       .alu_lower_bits_I0(alu__out_I0_p2[1:0]), 
                       .alu_lower_bits_I1(alu__out_I1_p2[1:0]), 
                       .instruction_number(instruction_number));

    // The following is load_extender that takes the memory_data_load
    // as input and calculates whether to sign extend the data or not, 
    // as well as how to re arrange the bits to return correct portions 
    load_extender le (.extended_load(extended_load),
                      .load_data(mem_data_load),
                      .dec__sel_I0(decode_out_I0_p2[14:6]),
                      .dec__sel_I1(decode_out_I1_p2[14:6]),
                      .alu_lower_bits_I0(alu__out_I0_p2[1:0]), 
                      .alu_lower_bits_I1(alu__out_I1_p2[1:0]),
                      .instruction_number(instruction_number));

    /* ########################################################### MEM Phase */

    /* ########################################### MEM/WB Pipeline Registers */

    // Enables for pipeline registers
    assign decode_out_I0_en3 = 1'b1;
    assign decode_out_I1_en3 = 1'b1;
    assign ir_en3 = 1'b1;
    assign alu__out_I0_en3 = 1'b1;
    assign alu__out_I1_en3 = 1'b1;
    assign extended_load_en3 = 1'b1;
    assign instruction_number_en3 = 1'b1;

    // decode pipeline register
    register #(15, 0) DECRegI0P3 (decode_out_I0_p3,
                                  decode_out_I0_p2, 
                                  clk,
                                  decode_out_I0_en3, 
                                  rst_b);

    // decode pipeline register
    register #(15, 0) DECRegI1P3 (decode_out_I1_p3,
                                  decode_out_I1_p2, 
                                  clk,
                                  decode_out_I1_en3, 
                                  rst_b);

    // instruction pipeline register
    register #(128, 0) IRRegP3 (ir_p3, ir_p2, clk,
                                ir_en3, rst_b);

    // alu out pipeline register
    register #(32, 0) ALURegI0P3 (alu__out_I0_p3, alu__out_I0_p2, clk,
                                  alu__out_I0_en3, rst_b);

    // alu out pipeline register
    register #(32, 0) ALURegI1P3 (alu__out_I1_p3, alu__out_I1_p2, clk,
                                  alu__out_I1_en3, rst_b);

    // extended load pipeline register
    register #(32, 0) LoadExtendRegP3 (extended_load_p3, extended_load, 
                                       clk, extended_load_en3, rst_b); 

    register #(1, 0) InstructionNumberRegP3 (instruction_number_p3, 
                                             instruction_number, 
                                             clk, instruction_number_en3, 
                                             rst_b);

    /* ########################################### MEM/WB Pipeline Registers */

    /* ############################################################ WB Phase */

    // This mux is the rd_data mux to decide whether or not to store the
    // data out of the alu or memory
    rd_data_mux RDMux (.rd_data({rd_data_I1, rd_data_I0}), 
                       .inst_type_I0(decode_out_I0_p3[5:3]), 
                       .inst_type_I1(decode_out_I1_p3[5:3]),
                       .extended_load(extended_load_p3), 
                       .alu__out_I0(alu__out_I0_p3), 
                       .alu__out_I1(alu__out_I1_p3));
    
    /* ############################################################ WB Phase */

endmodule // riscv_core

/* ################################################## Data Path Main Modules */

////
//// riscv_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed
////
module riscv_ALU(alu__out, bcond_taken, alu__op1, alu__op2, 
                 dec__sel, inst, pc, pc_plus_four);

    // outputs and inputs 
    output logic [31:0] alu__out;
    output logic bcond_taken;  
    input logic [31:0] alu__op1, alu__op2;
    input logic [8:0] dec__sel;
    input logic [31:0] inst, pc, pc_plus_four;

    // Initializations 
    logic [31:0] adder__out0, adder__out1, shift_logical__out;
    logic [31:0] logical_xor__out, logical_or_and__out0, logical_or_and__out1;
    logic [31:0] shift_arithmetic_right__out;
    logic op1_eq_op2, op1_neq_op2, op1_lt_op2, op1_ge_op2;
    logic op1_ltu_op2, op1_geu_op2;
    logic sub;

    // The following are modules that are used to compute 
    // various results for the different instruction types
    adder         add0 (adder__out0, alu__op1, alu__op2, sub);

    shift_logical  ls  (shift_logical__out, alu__op1, alu__op2, dec__sel[0]);

    logical_xor  lxor  (logical_xor__out, alu__op1, alu__op2);

    logical_or_and       orand0 (logical_or_and__out0, alu__op1, alu__op2, 
                                 dec__sel[0]);

    shift_arithmetic_right sra  (shift_arithmetic_right__out, 
                                 alu__op1, alu__op2);

    equal                  eq   (op1_eq_op2, alu__op1, alu__op2);

    less_than              lt   (op1_lt_op2, alu__op1, alu__op2);

    less_than_unsigned     ltu  (op1_ltu_op2, alu__op1, alu__op2);

    logical_or_and  orand1 (logical_or_and__out1, inst, 32'hFFFFF000, 1'b1);

    adder           add1   (adder__out1, logical_or_and__out1, pc, 1'b0);
 
    // This logic determines what to set the (sub) value to. This value
    // is fed into the first adder module above. If sub = 1, then the adder
    // subtracts the operands. Otherwise it adds. 
    always_comb begin
        if (dec__sel == `DEC_LB || dec__sel == `DEC_LH || dec__sel == `DEC_LW
            || dec__sel == `DEC_LBU || dec__sel == `DEC_LHU || 
            dec__sel == `DEC_SB || dec__sel == `DEC_SH ||
            dec__sel == `DEC_SW || dec__sel == `DEC_JALR) begin
            
            sub = 0;

        end
        else begin
            
            sub = dec__sel[0];
  
        end
    end

    // The following always_comb block determines which output to set
    // based on the select line from the decoder (instruction).
    always_comb begin
        bcond_taken = 0;
        alu__out = 0;
        case (dec__sel)
            `DEC_ADD: alu__out = adder__out0;

            `DEC_SUB: alu__out = adder__out0;

            `DEC_SLL: alu__out = shift_logical__out;

            `DEC_SRL: alu__out = shift_logical__out;

            `DEC_SRA: alu__out = shift_arithmetic_right__out;

            `DEC_XOR: alu__out = logical_xor__out;

            `DEC_SLT: alu__out = op1_lt_op2;
  
            `DEC_SLTU: alu__out = op1_ltu_op2;

            `DEC_OR: alu__out = logical_or_and__out0;

            `DEC_AND: alu__out = logical_or_and__out0;

            `DEC_ADDI: alu__out = adder__out0;

            `DEC_SUBI: alu__out = adder__out0;

            `DEC_SLLI: alu__out = shift_logical__out;

            `DEC_SRLI: alu__out = shift_logical__out;

            `DEC_SRAI: alu__out = shift_arithmetic_right__out;

            `DEC_XORI: alu__out = logical_xor__out;

            `DEC_SLTI: alu__out = op1_lt_op2;
  
            `DEC_SLTIU: alu__out = op1_ltu_op2;

            `DEC_ORI: alu__out = logical_or_and__out0;

            `DEC_ANDI: alu__out = logical_or_and__out0;
    
            `DEC_LB: alu__out = adder__out0;

            `DEC_LH: alu__out = adder__out0;
        
            `DEC_LW: alu__out = adder__out0;

            `DEC_LBU: alu__out = adder__out0;

            `DEC_LHU: alu__out = adder__out0;

            `DEC_SB: alu__out = adder__out0;
  
            `DEC_SH: alu__out = adder__out0;

            `DEC_SW: alu__out = adder__out0;

            `DEC_BEQ: bcond_taken = op1_eq_op2;

            `DEC_BNE: bcond_taken = !op1_eq_op2;

            `DEC_BLT: bcond_taken = op1_lt_op2;

            `DEC_BGE: bcond_taken = !op1_lt_op2;

            `DEC_BGEU: bcond_taken = !op1_ltu_op2;

            `DEC_BLTU: bcond_taken = op1_ltu_op2;

            `DEC_JALR: begin
              alu__out = pc_plus_four;
              bcond_taken = 1'b1;
             end 

            `DEC_LUI: alu__out = logical_or_and__out1;

            `DEC_AUIPC: alu__out = adder__out1;

            `DEC_JAL: begin
              alu__out = pc_plus_four;  
              bcond_taken = 1'b1;
            end    

            /** These instructions do not use the ALU:
             * DEC_ECALL 
             */
            default: alu__out = 0;

        endcase 
    end

endmodule

// The following is a sign extender that depending on 
// the type of instruction, decides how to make the offset
// and whether to sign extend it or not
module sign_extender
    (output logic [31:0] se_offset,
     input logic [31:0] inst,
     input logic [2:0] inst_type, 
     input logic [8:0] dec__sel); 
   
    // Cases on the instruction type, to form 
    // the offset value used in the datapath 
    always_comb begin
        case (inst_type)
            `I_TYPE: begin
                        if (dec__sel == `DEC_SLLI || dec__sel == `DEC_SRLI
                            || dec__sel == `DEC_SRAI) begin
                            se_offset = {{27{1'b0}}, inst[24:20]};  
                        end
                        else begin
                            se_offset = {{21{inst[31]}}, inst[30:20]};
                        end
                    end

            `L_TYPE: se_offset = {{21{inst[31]}}, inst[30:20]};

            `S_TYPE: se_offset = {{21{inst[31]}}, inst[30:25], inst[11:7]};

            `J_TYPE: begin
                        if (dec__sel == `DEC_JAL) begin
                            se_offset = {{12{inst[31]}}, inst[19:12], inst[20],
                                         inst[30:21], 1'b0}; 
                        end
                        else begin
                            se_offset = {{21{inst[31]}}, inst[30:20]};
                        end
                    end

            `B_TYPE: begin
                    se_offset = {{20{inst[31]}}, inst[7], inst[30:25],
                                 inst[11:8], 1'b0};
                    end

            default: se_offset = 0;
        endcase
    end 
    
endmodule 

// The following takes the output from the memory load and
// arranges the data in the proper format to account for
// misaligned loads and aligned memory accesses. It also 
// accounts for sign extension 
module load_extender 
    (output logic [31:0] extended_load, 
     input logic [31:0] load_data, 
     input logic [8:0] dec__sel_I0, dec__sel_I1, 
     input logic [1:0] alu_lower_bits_I0, alu_lower_bits_I1, 
     input logic instruction_number);

    logic [7:0] load_byte;
    logic [15:0] load_half; 
    logic [8:0] dec__sel;
    logic [1:0] alu_lower_bits;

    assign dec__sel = (instruction_number == 1'b0) ? dec__sel_I0 : dec__sel_I1;
    assign alu_lower_bits = ((instruction_number == 1'b0) ? 
                             alu_lower_bits_I0 : 
                             alu_lower_bits_I1);

    // Sets values of the byte and half variables
    // depending on the necessary alignment of the 
    // lower address bits that are ignored during
    // memory access.
    always_comb begin
        load_byte = 0; load_half = 0;
        if (dec__sel == `DEC_LB || dec__sel == `DEC_LBU) begin
            case (alu_lower_bits)
                2'b00: load_byte = load_data[7:0];
                2'b01: load_byte = load_data[15:8];
                2'b10: load_byte = load_data[23:16];
                2'b11: load_byte = load_data[31:24];
            endcase   
        end 
        else begin
            case (alu_lower_bits)
                2'b00: load_half = load_data[15:0];
                2'b01: load_half = 0;
                2'b10: load_half = load_data[31:16];
                2'b11: load_half = 0;
            endcase
        end
    end  

    // The following sets the output to the sign extended 
    // values depending on the instructions
    always_comb begin
        case (dec__sel)
            `DEC_LB: extended_load = {{25{load_byte[7]}}, load_byte[6:0]};

            `DEC_LH: extended_load = {{17{load_half[15]}}, load_half[14:0]};
        
            `DEC_LW: extended_load = load_data;
      
            `DEC_LBU: extended_load = {{24{1'b0}}, load_byte[7:0]};

            `DEC_LHU: extended_load = {{16{1'b0}}, load_half[15:0]};
          
            default: extended_load = 0; 
        endcase 
    end

endmodule  

/* ################################################## Data Path Main Modules */

/* ############################################### Branch Prediction Modules */

// This module defines the logic for the write enable for the BTB SRAM.
// It decides when it is necessary to modify the BTB prediction.
module modify_btb
  (output logic change_btb,
   input logic old_prediction, new_prediction);

  assign change_btb = (old_prediction != new_prediction);

endmodule

// This module determines whether or not to 
// take the prediction based on the saturation counter.
module evaluate_counter
  (output logic take_prediction, 
   input logic [1:0] current_counter);

  always_comb begin 
    unique case (current_counter)    
  
      `SNT: take_prediction = 1'b0;

      `WNT: take_prediction = 1'b0;

      `WT: take_prediction = 1'b1;

      `ST: take_prediction = 1'b1;     

    endcase
  end

endmodule 

// This is the 2 bit saturation counter for the branch prediction.
// It determines how to modify the saturation counter for the 
// current PC.
module saturation_counter
  (output logic [1:0] new_counter, 
   input logic [1:0] old_counter,   
   input logic branch_taken);

  always_comb begin 
    unique case (old_counter)    
  
      `SNT: new_counter = (branch_taken) ? `WNT : `SNT;

      `WNT: new_counter = (branch_taken) ? `WT : `SNT;

      `WT: new_counter = (branch_taken) ? `ST : `WNT;

      `ST: new_counter = (branch_taken) ? `ST : `WT;     

    endcase
  end

endmodule 

// This module determines whether there was a misprediction or not
// for the current control flow instruction in the EX phase.
module mispredict_detector
  (output logic mispredict, 
   input logic [61:0] btb_out, 
   input logic [31:0] pc, pc_jump_branch,
   input logic branch_taken, btb_we);

  // Internal variables
  logic [29:0] tag_pc, next_pc;
  logic [1:0] history;
  logic take_prediction;

  // Fields of BTB out
  assign {tag_pc, history, next_pc} = btb_out;

  // Evaluates saturation counter of BTB
  evaluate_counter eval (take_prediction, history);

  // Mispredict logic 
  always_comb begin

    if (btb_we) begin
    
      if (tag_pc == pc[31:2] && take_prediction) begin
        if (branch_taken && next_pc == pc_jump_branch[31:2])
          mispredict = 1'b0;
        else
          mispredict = 1'b1; 
      end

      else begin
        if (branch_taken)
          mispredict = 1'b1;
        else
          mispredict = 1'b0;
      end

    end

    else begin
      mispredict = 1'b0;
    end

  end

endmodule 

// This module computes the outcome of the control flow instruction
// in order to update the BTB with the required information (if applicable).
module compute_actual_outcome
  (output logic [61:0] btb_in, 
   input logic [61:0] btb_out, 
   input logic [31:0] pc_jump_branch, 
   input logic [31:0] pc_plus_four,
   input logic [31:0] pc, 
   input logic branch_taken);

  // Internal variables
  logic [29:0] tag_pc, next_pc;
  logic [1:0] history;

  logic [29:0] new_tag_pc, new_next_pc;
  logic [1:0] new_history;

  logic change_btb, prediction, new_prediction;

  // Fields of BTB out
  assign {tag_pc, history, next_pc} = btb_out;

  // Calculates the new saturation counter value
  saturation_counter satcnt (new_history, history, branch_taken);

  // Evaluate both history counters to determine prediction values.
  evaluate_counter eval0 (prediction, history);
  evaluate_counter eval1 (new_prediction, new_history);

  // Decides whether to modify the BTB prediction, 
  // and not just the saturation counter.
  modify_btb modbtb (change_btb, prediction, new_prediction);

  // Calculates new BTB input 
  always_comb begin
    
    if (change_btb) begin

      if (branch_taken)
        btb_in = {pc[31:2], new_history, pc_jump_branch[31:2]};
      else
        btb_in = {pc[31:2], new_history, pc_plus_four[31:2]};
        
    end

    else begin

      btb_in = {tag_pc, new_history, next_pc};

    end

  end

endmodule 

// Logic to determine whether or not to write into the BTB.
module btb_write_logic
  (output logic btb_we, 
   input logic [2:0] inst_type);

  assign btb_we = (inst_type == `B_TYPE || inst_type == `J_TYPE) ? 1'b1 : 1'b0; 

endmodule

/* ############################################### Branch Prediction Modules */

/* ######################################################## Reusable Modules */

// Performance Counter implementation 
module counter 
  (output logic [15:0] count, 
   input logic rst_b, clk, enable, up, amount);

  logic [15:0] add_sub;
  assign add_sub = (amount) ? 16'd2 : 16'd1;

  always_ff @(posedge clk, negedge rst_b) begin
  
    if (~rst_b)
      count <= 0;

    else if (enable) begin

      if (up)
        count <= count + add_sub;

      else
        count <= count - add_sub;

    end

  end

endmodule 

//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

    parameter
             width = 32,
             reset_value = 0;

    output [(width-1):0]  q;
    reg    [(width-1):0]  q;
    input  [(width-1):0]  d;
    input                 clk, enable, rst_b;

    always @(posedge clk or negedge rst_b)
        if (~rst_b)
            q <= reset_value;
        else if (enable)
            q <= d;

endmodule // register

////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(out, in1, in2, sub);
    output [31:0] out;
    input [31:0]  in1, in2;
    input         sub;

    assign        out = sub?(in1 - in2):(in1 + in2);

endmodule // adder

////
//// add_const: An adder that adds a fixed constant value
////
//// out (output) - adder result
//// in  (input)  - Operand
////
module add_const(out, in);

    parameter add_value = 1;

    output   [31:0] out;
    input    [31:0] in;

    assign   out = in + add_value;

endmodule // adder

// Sets output to op1 == op2
module equal
    (output logic op1_eq_op2, 
     input logic [31:0] alu__op1, alu__op2);

    assign op1_eq_op2 = (alu__op1 == alu__op2);

endmodule 

// op1 < op2 signed
module less_than
    (output logic op1_lt_op2, 
     input logic [31:0] alu__op1, alu__op2);

    assign op1_lt_op2 = ($signed(alu__op1) < $signed(alu__op2));

endmodule 

// op1 < op2 unsigned
module less_than_unsigned
    (output logic op1_ltu_op2, 
     input logic [31:0] alu__op1, alu__op2);

    assign op1_ltu_op2 = ($unsigned(alu__op1) < $unsigned(alu__op2));

endmodule 

// shift_logical: A shifter that logically shifts either right or left
module shift_logical
    (output logic [31:0] out, 
     input logic [31:0] in1, in2, 
     input logic right);

    assign out = (right) ? in1 >> (in2 & 32'h0x1F) : in1 << (in2 & 32'h0x1F);    

endmodule  

// The following module is used to XOR two given inputs
module logical_xor
    (output logic [31:0] out, 
     input logic [31:0] in1, in2);

    assign out = in1 ^ in2;
  
endmodule

// The following module is used to or/and two given inputs
module logical_or_and
    (output logic [31:0] out, 
     input logic [31:0] in1, in2, 
     input logic A);

    assign out = (A) ? in1 & in2 : in1 | in2;

endmodule

// The following module is used to shift right arithmetic
module shift_arithmetic_right
    (output logic [31:0] out, 
     input logic [31:0] in1, in2);

    assign out = $signed(in1) >>> (in2 & 32'h0x1F);

endmodule 

/* ######################################################## Reusable Modules */

/* ##################################################### Nonreusable Modules */

// Special adder for Jump and Branch instructions.
module jump_branch_adder
  (output logic [31:0] pc_jump_branch, 
   input logic [31:0] pc, se_offset, rs1_data,
   input logic [8:0] dec__sel);
  
  logic [31:0] adder_in, sum;

  assign adder_in = (dec__sel == `DEC_JALR) ? rs1_data : pc;

  // Internal adder
  adder add (sum, adder_in, se_offset, 1'b0);

  always_comb begin

    if (dec__sel == `DEC_JALR)
      pc_jump_branch = {sum[31:1], 1'b0};

    else
      pc_jump_branch = sum;

  end

endmodule 

/* ##################################################### Nonreusable Modules */

/* ########################################################### Various Muxes */

// Mux used to assign output to rs1/rs2 data, EX forward data, 
// MEM forward data, WB forward data, or a 0 for a bubble
module forwarding_mux
    (output logic [31:0] out, 
     input logic [31:0] data_ID, data_EX, data_load_MEM, data_alu_MEM, data_WB, 
     input logic stall, 
     input logic [1:0] forward, 
     input logic [2:0] inst_type);

    always_comb begin
        if (stall)
            out = 0;
        else begin
            case (forward)
                2'b00: out = data_ID;
                2'b01: out = data_EX;
                2'b10: begin
                    if (inst_type == `L_TYPE)
                        out = data_load_MEM;
                    else
                        out = data_alu_MEM;
                end
                2'b11: out = data_WB;
            endcase
        end
    end 

endmodule 

// Two input Mux used for IF/ID pipeline registers (stall)
module pipeline_register_mux
    #(parameter W = 32)
    (output logic [W-1:0] out, 
     input logic [W-1:0] in0, in1, 
     input logic select);

    always_comb begin
        case (select)
            1'b0: out = in0;
            1'b1: out = in1; 
        endcase 
    end

endmodule

// Decides how to align the store data based on the
// lower bits of the address that are ignored when storing 
// in memory (due to aligned memory accesses, but allowing
// misaligned memory operations).
module mem_data_store_mux
    (output logic [31:0] mem_data_store, 
     input logic [8:0] dec__sel, 
     input logic [1:0] alu_lower_bits, 
     input logic [31:0] rs2_data_I0, rs2_data_I1, 
     input logic instruction_number);

    logic [31:0] rs2_data;
    assign rs2_data = (instruction_number == 1'b0) ? rs2_data_I0 : rs2_data_I1;

    always_comb begin
        if (dec__sel == `DEC_SB) begin
            case (alu_lower_bits)
                2'b00: mem_data_store = rs2_data;
                2'b01: mem_data_store = rs2_data << 8;
                2'b10: mem_data_store = rs2_data << 16;
                2'b11: mem_data_store = rs2_data << 24;
            endcase
        end
        else if (dec__sel == `DEC_SH) begin
            case (alu_lower_bits)
                2'b00: mem_data_store = rs2_data; 
                2'b01: mem_data_store = 0;
                2'b10: mem_data_store = rs2_data << 16;
                2'b11: mem_data_store = 0;
            endcase 
        end
        else begin
            mem_data_store = rs2_data;
        end
    end

endmodule

// Decides how to load alu op2
module alu__op2_mux
    (output logic [31:0] alu__op2,
     input logic [2:0] inst_type, 
     input logic [31:0] rs2_data, se_offset);

    always_comb begin
        if (inst_type == `R_TYPE || inst_type == `B_TYPE)
            alu__op2 = rs2_data;
        else
            alu__op2 = se_offset;
    end

endmodule

// Decides value of nextpc based on instruction type
module nextpc_mux
    (output logic [31:0] nextpc, 
     input logic [61:0] btb_out,
     input logic [31:0] pc_plus_eight, pc_plus_eight_p1, 
     input logic [31:0] pc_jump_branch,
     input logic [31:0] pc,
     input logic        mispredict, branch_taken);

    // Internal defines
    logic [29:0] tag_pc, predicted_pc;
    logic [1:0] history;
    logic take_prediction;
    logic [29:0] tag_pc_p1, predicted_pc_p1;
    logic [1:0] history_p1;

    // Grab fields 
    assign {tag_pc, history, predicted_pc} = btb_out;

    // Evaluates the history counter prediction
    evaluate_counter eval (take_prediction, history);
  
    // Computer nextpc
    always_comb begin

      if (mispredict) begin

        if (branch_taken)
          nextpc = pc_jump_branch;

        else
          nextpc = pc_plus_eight_p1;
    
      end

      else begin

        if (tag_pc == pc[31:2] && take_prediction)
          nextpc = {predicted_pc, 2'b00};

        else
          nextpc = pc_plus_eight;
  
      end

    end 
  
endmodule 

// The following mux is used to decide what
// the mem_write_en bits will look like based on
// instruction and desired mask
module mem_write_mux
    (output logic [3:0] mem_write_en,
     input logic [8:0] dec__sel_I0, dec__sel_I1,
     input logic [1:0] alu_lower_bits_I0, alu_lower_bits_I1, 
     input logic instruction_number);

    logic [8:0] dec__sel;
    logic [1:0] alu_lower_bits;
    assign dec__sel = (instruction_number == 1'b0) ? dec__sel_I0 : dec__sel_I1;
    assign alu_lower_bits = ((instruction_number == 1'b0) ? 
                             alu_lower_bits_I0 : 
                             alu_lower_bits_I1);

    always_comb begin
      case (dec__sel)
        `DEC_SB: mem_write_en = (4'b0001 << alu_lower_bits);
        
        `DEC_SH: mem_write_en = (4'b0011 << alu_lower_bits);
      
        `DEC_SW: mem_write_en = (4'b1111);
 
        default: mem_write_en = 0;
      endcase
    end
endmodule

module instruction_memory_usage
  (output logic instruction_number, 
   input logic [2:0] inst_type_I0, inst_type_I1);

  assign instruction_number = ((inst_type_I0 == `L_TYPE || inst_type_I0 == `S_TYPE) ?
                               1'b0 :
                               1'b1);

endmodule 

module mem_addr_mux
  (output logic [29:0] mem_addr, 
   input logic [31:0] alu__out_I0, alu__out_I1, 
   input logic instruction_number);

  assign mem_addr = ((instruction_number == 1'b0) ?
                     alu__out_I0[31:2] :
                     alu__out_I1[31:2]);

endmodule 

// Decides how to load rd data 
module rd_data_mux
    (output logic [31:0] rd_data[1:0], 
     input logic [2:0] inst_type_I0, inst_type_I1,  
     input logic [31:0] extended_load, 
     input logic [31:0] alu__out_I0, alu__out_I1);

    always_comb begin
        if (inst_type_I0 == `L_TYPE)
            rd_data = {alu__out_I1, extended_load};
        else if (inst_type_I1 == `L_TYPE) 
            rd_data = {extended_load, alu__out_I0};
        else
            rd_data = {alu__out_I1, alu__out_I0};
    end

endmodule

/* ########################################################### Various Muxes */

// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:zi
