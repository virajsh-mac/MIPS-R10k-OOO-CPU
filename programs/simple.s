    addi x1, x0, 0x100     
    addi x2, x0, 0x42       

    sw  x2, 0(x1)
    sw  x2, 4(x1)
    sw  x2, 8(x1)
    sw  x2, 12(x1)
    sw  x2, 16(x1)
    sw  x2, 20(x1)
    sw  x2, 24(x1)
    sw  x2, 28(x1)
    sw  x2, 32(x1)
    wfi                    