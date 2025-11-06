/*
  Simple branch test program using bne instruction
  Tests basic branching functionality in a loop
*/
    li   x1, 0        # Initialize counter to 0
    li   x2, 5        # Loop limit
loop:
    addi x1, x1, 1    # Increment counter
    bne  x1, x2, loop # Branch back if x1 != x2
    # Loop exits when x1 == x2 (counter reaches 5)
    wfi               # End program
