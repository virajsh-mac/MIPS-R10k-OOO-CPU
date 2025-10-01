
# EECS 470 Final Project

Welcome to the EECS 470 Final Project!

This is the repository for your implementation of an out-of-order,
synthesizable, RISC-V processor with advanced features.

This repository has multiple changes from Project 3. So please read the
following sections to get up-to-date! In particular, the Makefile has
been improved to make it easy to add individual module testbenches
(**Use this for MS1!**).

The [Project Specification](https://www.eecs.umich.edu/courses/eecs470/labs/FinalProject-470-F24.pdf)
has more details on the overall structure of the project and deadlines.

To summarize the deadlines:
- Milestone 1 is due by **Wednesday, 10/15 at 11:59pm** (one module + testbench + 3 interfaces)
- Milestone 2 is due by **Friday, 10/31 at 11:59pm** (mult_no_lsq working)
- Milestone 3 is due by **Wednesday, 11/19 at 11:59pm** (most programs correct)
- Your final autograder submission is due by **Saturday 12/6 at 11:59pm**
- Your final report (10-20 pages) is due by **Monday, 12/8 at 11:59pm**

### Autograder Submission

For autograder submissions, we have these requirements:

1.  Running `make simv` will compile a simulation executable for your
    processor

2.  Running `make syn_simv` will compile a synthesis executable for your
    processor

3.  Running `make my_program.out` and `make my_program.syn.out` will run
    a program by loading a memory file in `programs/my_program.mem` (as
    in project 3)

4.  This must write the correct memory output (lines starting with @@@)
    to stdout when it runs, and you must generate the same
    `output/my_program.cpi` file exactly as in project 3.

One note on memory ouput: when you start implementing your data cache,
you will need to ensure that any dirty cache values get written in the
memory output instead of the value from memory. This will require
exposing your cache at the top level and editing the
`show_mem_and_status` task in `test/cpu_test.sv`.

## Getting Started

Start the project by working on your first module, either the ReOrder
Buffer (ROB) or the Reservation Station (RS). Implement the modules in
files in the `verilog/` folder, and write testbenches for them in the
`test/` folder. If you're writing the ROB, name these like:
`verilog/rob.sv` and `test/rob_test.sv` which implement and test the
module named `rob`.

Once you have something written, try running the new Makefile targets.
Add `rob` to the MODULES variable in the Makefile, then run
`make rob.out` to compile, run, and check the testbench. Do the same for
synthesis with `make rob.syn.out`. And finally, check your testbench's
coverage with `make rob.cov`

If you have your testbench output "@@@ Passed" and "@@@ Failed", then
you can use `make rob.pass rob.syn.pass` targets to print these in green
and red!

After you have the first module written and tested, keep going and work
towards a full processor. Try to pass the `mult_no_lsq` program for
milestone 2 -- you can verify this using the .wb file from project 3!

## Changes from Project 3

Many of the files from project 3 are still present or kept the same, but
there are a number of notable changes:

### The Makefile

The final project requires writing many modules, so we've added a new
section to the Makefile to compile arbitrary modules and testbenches.

To make it work for a module `mod`, create the files `verilog/mod.sv`
and `test/mod_test.sv` which implement and test the module. If you
update the `MODULES` variable in the Makefile, then it will be able to
link the new targets below.

The most straightforward targets are `make mod.out`, `make mod.syn.out`
and `make mod.cov`, which run the module on its testbench in simulation,
run the module in synthesis, and print the coverage results for the
testbench.

We also now put the VCS compilation results in a `build/` folder so the
top level folder doesn't get too messy!

``` make
# ---- Module Testbenches ---- #
# NOTE: these require files like: 'verilog/rob.sv' and 'test/rob_test.sv'
#       which implement and test the module: 'rob'
make <module>.pass   <- greps for "@@@ Passed" or "@@@ Failed" in the output
make <module>.out    <- run the testbench (via build/<module>.simv)
make <module>.verdi  <- run in verdi (via build/<module>.simv)
make build/<module>.simv  <- compile the testbench executable

make <module>.syn.pass   <- greps for "@@@ Passed" or "@@@ Failed" in the output
make <module>.syn.out    <- run the synthesized module on the testbench
make <module>.syn.verdi  <- run in verdi (via <module>.syn.simv)
make synth/<module>.vg        <- synthesize the module
make build/<module>.syn.simv  <- compile the synthesized module with the testbench

# ---- module testbench coverage ---- #
make <module>.cov        <- print the coverage hierarchy report to the terminal
make <module>.cov.verdi  <- open the coverage report in verdi
make cov_report_<module>      <- run urg to create human readable coverage reports
make build/<module>.cov.vdb   <- runs the executable and makes a coverage output dir
make build/<module>.cov.simv  <- compiles a coverage executable for the testbench
```

### `verilog/sys_defs.svh`

`sys_defs` has received a few changes to prepare the final project:

1.  We've defined `CACHE_MODE`, affecting `test/mem.sv` and changing
    the way the processor interacts with memory.

2.  We've added a memory latency of 100ns, so memory is now much
    slower, and handling it with caching is necessary.

3.  There is a new 'Parameters' section giving you a starting point
    for some common macros that will likely need to be decided on like
    the size of the ROB, the number of functional units, etc.

3.  The ALU functions have separated the multiplier operations out

### CPU Files

The two files `verilog/cpu.sv` and `test/cpu_test.sv` have been edited
to comment-out or remove project 3 specific code, so you should be able
to re-use them when you want to start integrating your modules into a
full processor again.

## New Files

We've added an `icache` module in `verilog/icache.sv`. That file has
more comments explaining how it works, but the idea is it stores
memory's response tag until memory returns that tag with the data. More
about how our processor's memory works will be presented in the final
lab section.

The file `psel_gen.sv` implements an incredibly efficient parameterized
priority selector (remember project 1?!). many tasks in superscalar
processors come down to priority selection, so instead of writing
manual for-loops, try to use this module. It is faster than any
priority selector the instructors are aware of (as far as my last
conversation about it with Brehob).

As promised, we've also copied the multiplier from project 2 and moved
the `` `STAGES`` definition to `sys_defs.svh` as `` `MULT_STAGES``.
This is set to 4 to start, but you can change it to 2 or 8 depending on
your processor's clock period.

### `verilog/p3` and the `decoder.sv`

The project 3 files are no longer relevant to your final processor, but
they are still good references, so project 3's starter verilog source
files have been moved to `verilog/p3/`. Notably, the decoder has been
pulled out as a new file `verilog/decoder.sv`. This new decoder also
adds a new flag for multiply instructions for adding mult as a
functional unit.

## P3 Makefile Target Reference

These are the Makefile targets for project 3, I've left it here for
reference. Most of the P3 portion of the Makefile is unchanged.

To run a program on the processor, run `make <my_program>.out`. This
will assemble a RISC-V `*.mem` file which will be loaded into `mem.sv`
by the testbench, and will also compile the processor and run the
program.

All of the "`<my_program>.abc`" targets are linked to do both the
executable compilation step and the `.mem` compilation steps if
necessary, so you can run each without needing to run anything else
first.

`make <my_program>.out` should be your main command for running
programs: it creates the `<my_program>.out`, `<my_program>.cpi`,
`<my_program>.wb`, and `<my_program>.ppln` output, CPI, writeback, and
pipeline output files in the `output/` directory. The output file
includes the processor status and the final state of memory, the CPI
file contains the total runtime and CPI calculation, the writeback file
is the list of writes to registers done by the program, and the pipeline
file is the state of each of the pipeline stages as the program is run.

The following Makefile rules are available to run programs on the
processor:

``` make
# ---- Program Execution ---- #
# These are your main commands for running programs and generating output
make <my_program>.out      <- run a program on build/cpu.simv
                              output *.out, *.cpi, *.wb, and *.ppln files
make <my_program>.syn.out  <- run a program on build/cpu.syn.simv and do the same

# ---- Program Memory Compilation ---- #
# Programs to run are in the programs/ directory
make programs/<my_program>.mem  <- compile a program to a RISC-V memory file
make compile_all                <- compile every program at once (in parallel with -j)

# ---- Dump Files ---- #
make <my_program>.dump  <- disassembles compiled memory into RISC-V assembly dump files
make *.debug.dump       <- for a .c program, creates dump files with a debug flag
make dump_all           <- create all dump files at once (in parallel with -j)

# ---- Verdi ---- #
make <my_program>.verdi     <- run a program in verdi via build/cpu.simv
make <my_program>.syn.verdi <- run a program in verdi via build/cpu.syn.simv

# ---- Cleanup ---- #
make clean            <- remove per-run files and compiled executable files
make nuke             <- remove all files created from make rules
```
