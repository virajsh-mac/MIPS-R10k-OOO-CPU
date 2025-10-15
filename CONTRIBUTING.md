# Contributing to VeriSimpleV

## Code Standards

- **Verilog/SystemVerilog Style:**
  - Use descriptive names for modules, signals, and variables (e.g., `alloc_valid` instead of `av`).
  - Indent with 4 spaces; no tabs.
  - Place module ports on separate lines for readability.
  - Use `logic` for all wires and regs unless a specific type is needed.
  - Comment complex logic and state machines.
  - Avoid blocking assignments (`=`) in always_ff blocks; use non-blocking (`<=`).
  - Parameterize designs where possible (e.g., `parameter RS_SZ = 16`).
  - Make sure to use the SystemVerilog and Verilog Formatter which should automatically apply when saving files.

- **General:**
  - Keep modules focused and single-responsibility.
  - Use assertions for critical invariants.
  - Ensure synthesizability: avoid initial blocks in synthesizable modules, use generate blocks for conditional logic.

## Pull Request Guidelines

Every PR must include these 4 sections in the description:

1. **Overview:** A single sentence summarizing the change.
2. **Key Changes:** Comprehensive bullet-point list of all modifications, including affected files and rationale.
3. **Notes:** Optional; any additional context, trade-offs, or future work.
4. **Verification Evidence:** Screenshots or logs showing passing tests in both simulation (`make <module>.pass`) and synthesis (`make <module>.syn.pass`). Include waveforms if debugging issues.

PRs should target the `main` branch. Before submitting:
- Run `make clean` and rebuild to ensure a clean slate.
- Verify all relevant testbenches pass.
- Update documentation if interfaces change.

## Merging to Main

- Always use **squash merge** to consolidate commits into a single, descriptive commit message.
- This preserves a linear, compiling history on `main` where every commit passes all tests.
- After merging, run `make simulate_all` and `make simulate_all_syn` to confirm.

## Testbench Guidelines

Testbenches validate individual modules (e.g., `rs`, `rob`) and follow a structured format. See `test/rs_test.sv` as an example.

### Structure
1. **Includes and Defines:**
   - `include "sys_defs.svh"` for shared definitions.
   - Override placeholders (e.g., ``define RS_SZ 16 ``, ``define N 3 ``) if not set in `sys_defs.svh`.
   - Define custom types (e.g., `typedef struct packed { ... } CDB_PACKET;`).

2. **Module Declaration:**
   - `module testbench;`
   - Declare clock, reset, and `logic failed;`.
   - Declare inputs/outputs matching the DUT ports.

3. **DUT Instantiation:**
   - Instantiate the module under test (DUT) with all ports connected.
   - Example: `rs dut (...);`

4. **Clock Generation:**
   - `always begin #5 clock = ~clock; end` (adjust period as needed).

5. **Helper Tasks/Functions:**
   - `task apply_inputs(...)` to drive inputs and advance clock.
   - `task check_*` for assertions (e.g., `check_free_count`, `check_entries`).
   - `function` for comparisons (e.g., `compare_rs_entry`).
   - Helpers for defaults (e.g., `empty_entry`, `all_empty`).

6. **Initial Block:**
   - Initialize signals.
   - Sequence tests with `$display` headers (e.g., "Test 1: Reset state").
   - Use tasks to apply stimuli and check results.
   - Handle edge cases: allocation, wakeup, clear, mispredict, stress.
   - End with: `if (failed) $display("@@@ Failed\n"); else $display("@@@ Passed\n");`
   - Optional: `$dumpfile` and `$dumpvars` for VCD debugging.

### Best Practices
- Test all functionality: reset, normal ops, boundaries (full/empty), interactions (alloc+clear).
- Use explicit comparisons; print mismatches with `$display`.
- Keep tests self-contained; no external dependencies.
- Aim for 100% coverage; use `make <module>.cov` to check.

## Running Testbenches

Use the Makefile to compile and run tests for modules listed in `MODULES` (e.g., `cpu`, `mult`, `rob`, `rs`).

### Simulation
- Compile: `make build/<module>.simv`
- Run: `make <module>.out`
- Check pass/fail: `make <module>.pass` (greps for "@@@ Passed/Failed")
- Verdi debug: `make <module>.verdi`
- Coverage: `make <module>.cov` (terminal report) or `make <module>.cov.verdi`

### Synthesis
- Synthesize: `make synth/<module>.vg`
- Compile synthesized: `make build/<module>.syn.simv`
- Run: `make <module>.syn.out`
- Check pass/fail: `make <module>.syn.pass`
- Verdi: `make <module>.syn.verdi`
- Slack check: `make slack` (for timing)

For full CPU runs: `make simulate_all` (simulation) or `make simulate_all_syn` (synthesis).

## Additional Tips
- Load modules: `module load vcs/2023.12-SP2-1` (for CAEN).
- Clean up: `make clean` (build files) or `make nuke` (everything).
- Programs: Compile with `make compile_all`; run with `make <program>.out`.

