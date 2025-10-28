`timescale 1ns/100ps
`include "sys_defs.svh"

// ------------------------------------------------------------------
// N-way freelist (no branch recovery)
// - Holds free physical registers in a circular buffer
// - Up to N allocs and N returns per cycle
// - Reset seeds the queue with tags [START_TAG .. PR_COUNT-1]
// - Oldest lane (0) has priority on allocation
// ------------------------------------------------------------------
module freelist #(
  parameter int N            = `N,
  parameter int PR_COUNT     = `PHYS_REG_SZ_R10K,
  parameter int ARCH_COUNT   = 32,
  parameter bit EXCLUDE_ZERO = 1'b1
)(
  input  logic                 clock,
  input  logic                 reset_n,
  input  logic   [N-1:0]       DispatchEN,
  input  logic   [N-1:0]       RetireEN,
  input  PHYS_TAG [N-1:0]      RetireReg,
  output PHYS_TAG [N-1:0]      FreeReg,       // valid when FreeRegValid[i] = 1
  output logic   [N-1:0]       FreeRegValid
);
