/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  sys_defs.svh                                        //
//                                                                     //
//  Description :  This file defines macros and data structures used   //
//                 throughout the processor.                           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_SVH__
`define __SYS_DEFS_SVH__

// all files should `include "sys_defs.svh" to at least define the timescale
`timescale 1ns / 100ps

///////////////////////////////////
// ---- Starting Parameters ---- //
///////////////////////////////////

// some starting parameters that you should set
// this is *your* processor, you decide these values (try analyzing which is best!)

// =========================================
// R10K Processor Parameters (6-stage pipeline)
// =========================================

// superscalar width (3-way superscalar)
`define N 2
`define CDB_SZ `N // This MUST match superscalar width
`define MAX_RS_FREE_CNT 3 // max broadcasted free spots in RS

// structure sizes
`define ROB_SZ 32 // ROB Size
`define RS_SZ 16 // Reservation Station Size
`define PHYS_REG_SZ_P6 32 // 32 Physical Register Size for P6
`define PHYS_REG_SZ_R10K (32 + `ROB_SZ)  // 64 physical registers for R10K

// physical register and index bit widths
`define PHYS_TAG_BITS $clog2(`PHYS_REG_SZ_R10K)  // 6 bits for phys tag
`define ROB_IDX_BITS $clog2(`ROB_SZ)            // 5 bits for ROB index
`define RS_IDX_BITS $clog2(`RS_SZ)              // 4 bits for RS index
`define NUM_CATS 4                              // Number of OP_CATEGORY values (0-4)

// branch prediction
`define BRANCH_PRED_SZ 512  // Branch predictor size

// functional units
`define NUM_FU_ALU 3      // Enough for superscalar width
`define NUM_FU_MULT 1     // Single pipelined multiplier
`define NUM_FU_BRANCH 1   // Single branch resolver
`define NUM_FU_MEM 1     // Single address calculator for mem ops
`define NUM_FU_TOTAL (`NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_BRANCH + `NUM_FU_MEM)

// number of mult stages (2, 4) (you likely don't need 8)
`define MULT_STAGES 4

// cache parameters
`define ICACHE_ASSOC 2           // 2-way associative I-cache
`define ICACHE_LINES 32
`define ICACHE_LINE_BITS $clog2(`ICACHE_LINES)
`define ICACHE_LINE_BYTES 16     // 16 bytes (4 instructions) for superscalar support
`define VICTIM_CACHE_SZ 4        // Small victim cache

`define DCACHE_ASSOC 2           // 2-way associative D-cache
`define DCACHE_LINES 32
`define DCACHE_LINE_BYTES 8      // 8 bytes/line (2 words; 256 bytes total)
`define DCACHE_VICTIM_SZ 4       // Small victim cache

// Load/Store Queue (not implemented in base design)
`define LSQ_SZ 8

///////////////////////////////
// ---- Basic Constants ---- //
///////////////////////////////

// NOTE: the global CLOCK_PERIOD is defined in the Makefile

// useful boolean single-bit definitions
`define FALSE 1'h0
`define TRUE 1'h1

// word and register sizes
typedef logic [31:0] ADDR;
typedef logic [31:0] DATA;
typedef logic [4:0] REG_IDX;

// =========================================
// R10K Typedefs (used across all stages)
// =========================================

// Physical register tag
typedef logic [`PHYS_TAG_BITS-1:0] PHYS_TAG;

// ROB index
typedef logic [`ROB_IDX_BITS-1:0] ROB_IDX;

// RS index
typedef logic [`RS_IDX_BITS-1:0] RS_IDX;

// the zero register
// In RISC-V, any read of this register returns zero and any writes are thrown away
`define ZERO_REG 5'd0

// Basic NOP instruction. Allows pipline registers to clearly be reset with
// an instruction that does nothing instead of Zero which is really an ADDI x0, x0, 0
`define NOP 32'h00000013

//////////////////////////////////
// ---- Memory Definitions ---- //
//////////////////////////////////

// you are not allowed to change this definition for your final processor
// the project 3 processor has a massive boost in performance just from having no mem latency
// see if you can beat it's CPI in project 4 even with a 100ns latency!
//`define MEM_LATENCY_IN_CYCLES  0
`define MEM_LATENCY_IN_CYCLES (100.0/`CLOCK_PERIOD+0.49999)
// the 0.49999 is to force ceiling(100/period). The default behavior for
// float to integer conversion is rounding to nearest

// memory tags represent a unique id for outstanding mem transactions
// 0 is a sentinel value and is not a valid tag
`define NUM_MEM_TAGS 15
typedef logic [3:0] MEM_TAG;

// icache definitions
`define ICACHE_LINES 32
`define ICACHE_LINE_BITS $clog2(`ICACHE_LINES)

`define MEM_SIZE_IN_BYTES (64*1024)
`define MEM_64BIT_LINES (`MEM_SIZE_IN_BYTES/8)

// A memory or cache block
typedef union packed {
    logic [7:0][7:0]  byte_level;
    logic [3:0][15:0] half_level;
    logic [1:0][31:0] word_level;
    logic [63:0]      dbbl_level;
} MEM_BLOCK;

typedef enum logic [1:0] {
    BYTE   = 2'h0,
    HALF   = 2'h1,
    WORD   = 2'h2,
    DOUBLE = 2'h3
} MEM_SIZE;

// Memory bus commands
typedef enum logic [1:0] {
    MEM_NONE  = 2'h0,
    MEM_LOAD  = 2'h1,
    MEM_STORE = 2'h2
} MEM_COMMAND;

// icache tag struct
typedef struct packed {
    logic [12-`ICACHE_LINE_BITS:0] tags;
    logic                          valid;
} ICACHE_TAG;

// CDB entry
typedef struct packed {
    logic valid;
    PHYS_TAG tag;
    DATA data;
} CDB_ENTRY;

// CDB early tag entry (used to broadcast tags a cycle early)
typedef struct packed {
    logic valid;  // Valid broadcasts this cycle
    PHYS_TAG tag;  // Physical dest tags
} CDB_EARLY_TAG_ENTRY;

///////////////////////////////
// ---- Exception Codes ---- //
///////////////////////////////

/**
 * Exception codes for when something goes wrong in the processor.
 * Note that we use HALTED_ON_WFI to signify the end of computation.
 * It's original meaning is to 'Wait For an Interrupt', but we generally
 * ignore interrupts in 470
 *
 * This mostly follows the RISC-V Privileged spec
 * except a few add-ons for our infrastructure
 * The majority of them won't be used, but it's good to know what they are
 */

typedef enum logic [3:0] {
    INST_ADDR_MISALIGN  = 4'h0,
    INST_ACCESS_FAULT   = 4'h1,
    ILLEGAL_INST        = 4'h2,
    BREAKPOINT          = 4'h3,
    LOAD_ADDR_MISALIGN  = 4'h4,
    LOAD_ACCESS_FAULT   = 4'h5,
    STORE_ADDR_MISALIGN = 4'h6,
    STORE_ACCESS_FAULT  = 4'h7,
    ECALL_U_MODE        = 4'h8,
    ECALL_S_MODE        = 4'h9,
    NO_ERROR            = 4'ha,  // a reserved code that we use to signal no errors
    ECALL_M_MODE        = 4'hb,
    INST_PAGE_FAULT     = 4'hc,
    LOAD_PAGE_FAULT     = 4'hd,
    HALTED_ON_WFI       = 4'he,  // 'Wait For Interrupt'. In 470, signifies the end of computation
    STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

///////////////////////////////////
// ---- Instruction Typedef ---- //
///////////////////////////////////

// from the RISC-V ISA spec
typedef union packed {
    logic [31:0] inst;
    struct packed {
        logic [6:0] funct7;
        logic [4:0] rs2;  // source register 2
        logic [4:0] rs1;  // source register 1
        logic [2:0] funct3;
        logic [4:0] rd;  // destination register
        logic [6:0] opcode;
    } r;  // register-to-register instructions
    struct packed {
        logic [11:0] imm;  // immediate value for calculating address
        logic [4:0] rs1;  // source register 1 (used as address base)
        logic [2:0] funct3;
        logic [4:0] rd;  // destination register
        logic [6:0] opcode;
    } i;  // immediate or load instructions
    struct packed {
        logic [6:0] off;  // offset[11:5] for calculating address
        logic [4:0] rs2;  // source register 2
        logic [4:0] rs1;  // source register 1 (used as address base)
        logic [2:0] funct3;
        logic [4:0] set;  // offset[4:0] for calculating address
        logic [6:0] opcode;
    } s;  // store instructions
    struct packed {
        logic       of;      // offset[12]
        logic [5:0] s;       // offset[10:5]
        logic [4:0] rs2;     // source register 2
        logic [4:0] rs1;     // source register 1
        logic [2:0] funct3;
        logic [3:0] et;      // offset[4:1]
        logic       f;       // offset[11]
        logic [6:0] opcode;
    } b;  // branch instructions
    struct packed {
        logic [19:0] imm;  // immediate value
        logic [4:0] rd;  // destination register
        logic [6:0] opcode;
    } u;  // upper-immediate instructions
    struct packed {
        logic       of;      // offset[20]
        logic [9:0] et;      // offset[10:1]
        logic       s;       // offset[11]
        logic [7:0] f;       // offset[19:12]
        logic [4:0] rd;      // destination register
        logic [6:0] opcode;
    } j;  // jump instructions

    // extensions for other instruction types
`ifdef ATOMIC_EXT
    struct packed {
        logic [4:0] funct5;
        logic       aq;
        logic       rl;
        logic [4:0] rs2;
        logic [4:0] rs1;
        logic [2:0] funct3;
        logic [4:0] rd;
        logic [6:0] opcode;
    } a;  // atomic instructions
`endif
`ifdef SYSTEM_EXT
    struct packed {
        logic [11:0] csr;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  rd;
        logic [6:0]  opcode;
    } sys;  // system call instructions
`endif

} INST;  // instruction typedef, this should cover all types of instructions

////////////////////////////////////////
// ---- Datapath Control Signals ---- //
////////////////////////////////////////

// ALU opA input mux selects
typedef enum logic [1:0] {
    OPA_IS_RS1  = 2'h0,
    OPA_IS_NPC  = 2'h1,
    OPA_IS_PC   = 2'h2,
    OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
    OPB_IS_RS2   = 4'h0,
    OPB_IS_I_IMM = 4'h1,
    OPB_IS_S_IMM = 4'h2,
    OPB_IS_B_IMM = 4'h3,
    OPB_IS_U_IMM = 4'h4,
    OPB_IS_J_IMM = 4'h5
} ALU_OPB_SELECT;

////////////////////////////////////////
// ----      Major Structures    ---- //
////////////////////////////////////////

// ALU function code
typedef enum logic [3:0] {
    ADD,
    SUB,
    AND,
    SLT,
    SLTU,
    OR,
    XOR,
    SLL,
    SRL,
    SRA,
    CSRRW,
    CSRRS,
    CSRRC
} ALU_FUNC;

// MULT funct3 code
// we don't include division or rem options
typedef enum logic [2:0] {
    MUL,
    MULH,
    MULHSU,
    MULHU
} MULT_FUNC;

typedef enum logic [3:0] {
    LOAD_BYTE,
    LOAD_HALF,
    LOAD_WORD,
    LOAD_DOUBLE,
    STORE_BYTE,
    STORE_HALF,
    STORE_WORD,
    STORE_DOUBLE,
    LOAD_BYTE_U,
    LOAD_HALF_U
} MEM_FUNC;

typedef enum logic [3:0] {
    EQ,
    NE,
    LT,
    GE,
    LTU,
    GEU,
    JAL,
    JALR
} BRANCH_FUNC;

typedef enum logic [2:0] {
    CAT_ALU = 3'b000,
    CAT_MULT = 3'b001,
    CAT_MEM = 3'b010,
    CAT_BRANCH = 3'b011
} OP_CATEGORY;

// Packed struct for OP_TYPE (total 7 bits)
typedef struct packed {
    OP_CATEGORY category;  // 3 bits
    logic [3:0] func;      // 4 bits for sub-op (e.g., ADD=4'h0, MUL=4'h0, BYTE=4'h0)
} OP_TYPE;

// Constants for specific ops (assign struct values)
const OP_TYPE OP_ALU_ADD = '{category: CAT_ALU, func: ADD};
const OP_TYPE OP_ALU_SUB = '{category: CAT_ALU, func: SUB};
const OP_TYPE OP_ALU_AND = '{category: CAT_ALU, func: AND};
const OP_TYPE OP_ALU_SLT = '{category: CAT_ALU, func: SLT};
const OP_TYPE OP_ALU_SLTU = '{category: CAT_ALU, func: SLTU};
const OP_TYPE OP_ALU_OR = '{category: CAT_ALU, func: OR};
const OP_TYPE OP_ALU_XOR = '{category: CAT_ALU, func: XOR};
const OP_TYPE OP_ALU_SLL = '{category: CAT_ALU, func: SLL};
const OP_TYPE OP_ALU_SRL = '{category: CAT_ALU, func: SRL};
const OP_TYPE OP_ALU_SRA = '{category: CAT_ALU, func: SRA};

// CSR operations
const OP_TYPE OP_CSRRW = '{category: CAT_ALU, func: CSRRW};
const OP_TYPE OP_CSRRS = '{category: CAT_ALU, func: CSRRS};
const OP_TYPE OP_CSRRC = '{category: CAT_ALU, func: CSRRC};

// Multiply operations
const OP_TYPE OP_MULT_MUL = '{category: CAT_MULT, func: MUL};
const OP_TYPE OP_MULT_MULH = '{category: CAT_MULT, func: MULH};
const OP_TYPE OP_MULT_MULHSU = '{category: CAT_MULT, func: MULHSU};
const OP_TYPE OP_MULT_MULHU = '{category: CAT_MULT, func: MULHU};

// Memory operations (func[3]=1 for unsigned loads, func[2:0] for size)
const OP_TYPE OP_LOAD_BYTE = '{category: CAT_MEM, func: LOAD_BYTE};  // Signed byte
const OP_TYPE OP_LOAD_HALF = '{category: CAT_MEM, func: LOAD_HALF};  // Signed half
const OP_TYPE OP_LOAD_WORD = '{category: CAT_MEM, func: LOAD_WORD};  // Signed word
const OP_TYPE OP_LOAD_DOUBLE = '{category: CAT_MEM, func: LOAD_DOUBLE};  // Signed double
const OP_TYPE OP_STORE_BYTE = '{category: CAT_MEM, func: STORE_BYTE};
const OP_TYPE OP_STORE_HALF = '{category: CAT_MEM, func: STORE_HALF};
const OP_TYPE OP_STORE_WORD = '{category: CAT_MEM, func: STORE_WORD};
const OP_TYPE OP_STORE_DOUBLE = '{category: CAT_MEM, func: STORE_DOUBLE};
const OP_TYPE OP_LOAD_BYTE_U = '{category: CAT_MEM, func: LOAD_BYTE_U};  // Unsigned byte
const OP_TYPE OP_LOAD_HALF_U = '{category: CAT_MEM, func: LOAD_HALF_U};  // Unsigned half

// Branch operations
const OP_TYPE OP_BR_EQ = '{category: CAT_BRANCH, func: EQ};
const OP_TYPE OP_BR_NE = '{category: CAT_BRANCH, func: NE};
const OP_TYPE OP_BR_LT = '{category: CAT_BRANCH, func: LT};
const OP_TYPE OP_BR_GE = '{category: CAT_BRANCH, func: GE};
const OP_TYPE OP_BR_LTU = '{category: CAT_BRANCH, func: LTU};
const OP_TYPE OP_BR_GEU = '{category: CAT_BRANCH, func: GEU};
const OP_TYPE OP_JAL = '{category: CAT_BRANCH, func: JAL};
const OP_TYPE OP_JALR = '{category: CAT_BRANCH, func: JALR};

// for ROB via Complete
typedef struct packed {
    logic [`N-1:0]   valid;           // Valid updates this cycle
    ROB_IDX [`N-1:0] idx;             // ROB indices to update
    DATA [`N-1:0]    values;          // Values to store (if applicable)
    logic [`N-1:0]   branch_taken;    // Resolved taken/not taken (if branch)
    ADDR [`N-1:0]    branch_targets;  // Resolved branch targets (if branch)
} ROB_UPDATE_PACKET;

// RS entry structure (extended for full control signals)
typedef struct packed {
    logic          valid;        // Entry occupied
    ALU_OPA_SELECT opa_select;   // From decode (where is OPA coming from)
    ALU_OPB_SELECT opb_select;   // From decode (where is OPB coming from)
    OP_TYPE        op_type;      // Which unit are we routing to in EX and what suboperation
    PHYS_TAG       src1_tag;     // Physical source 1 tag
    logic          src1_ready;   // Source 1 ready
    DATA           src1_value;   // Source 1 value if immediate
    PHYS_TAG       src2_tag;     // Physical source 2 tag
    logic          src2_ready;   // Source 2 ready
    DATA           src2_value;   // Source 2 value if immediate
    PHYS_TAG       dest_tag;     // Physical destination tag
    ROB_IDX        rob_idx;      // Associated ROB index (for flush and potential age selection)
    logic          rob_wrap;     // signal to indicate wrap around in the ROB
    ADDR           PC;           // PC for branch/debug (MIGHT merge with SRC but only if we can resolve mispredicts othersive)
    // Added for branches (prediction info from fetch via dispatch)
    logic          pred_taken;
    ADDR           pred_target;
} RS_ENTRY;

// ROB entry structure
typedef struct packed {
    logic          valid;          // Entry occupied
    ADDR           PC;             // PC of instruction
    INST           inst;           // Full instruction
    REG_IDX        arch_rd;        // Architectural destination reg
    PHYS_TAG       phys_rd;        // Assigned physical dest reg
    PHYS_TAG       prev_phys_rd;   // Previous physical mapping (for free on commit)
    DATA           value;          // Computed value (from Complete, if needed)
    logic          complete;       // Instruction has completed
    EXCEPTION_CODE exception;      // Any exception code
    logic          branch;         // Is this a branch?
    ADDR           branch_target;  // Resolved branch target
    logic          branch_taken;   // Resolved taken/not taken
    ADDR           pred_target;    // Predicted branch target
    logic          pred_taken;     // Predicted taken/not taken
    logic          halt;           // Is this a halt?
    logic          illegal;        // Is this illegal?
} ROB_ENTRY;

`endif  // __SYS_DEFS_SVH__
