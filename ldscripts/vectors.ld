/*
    STM32F103C interrupts vectors table

    Every `.isr_vector.__vec*` input section, if defined,
    goes to a specific offset from start of a vector table.
    This saves some space if some of interrupts are not used.

    Size of an input section must be equal to 4 bytes.
    Input section names are unique.

    Use GCC's __attribute__ to put a vector into a section.
*/

/* Stack pointer value */
.isr_vector.sp (. + 0x00) : {
    _sivt = . ;                         /* _sivt */
    KEEP(*(.isr_vector.__sp_val))
}

/* Reset vector */
.isr_vector.reset (_sivt + 0x04) : {
    KEEP(*(.isr_vector.__vec_IRQ_Reset))
}

/* System exceptions */
.isr_vector.sysex  : {
    . = DEFINED(__vec_IRQ_NMI         ) ? (_sivt + 0x08) : . ;  KEEP(*(.isr_vector.__vec_IRQ_NMI         ))
    . = DEFINED(__vec_IRQ_HardFault   ) ? (_sivt + 0x0c) : . ;  KEEP(*(.isr_vector.__vec_IRQ_HardFault   ))
    . = DEFINED(__vec_IRQ_MemManage   ) ? (_sivt + 0x10) : . ;  KEEP(*(.isr_vector.__vec_IRQ_MemManage   ))
    . = DEFINED(__vec_IRQ_BusFault    ) ? (_sivt + 0x14) : . ;  KEEP(*(.isr_vector.__vec_IRQ_BusFault    ))
    . = DEFINED(__vec_IRQ_UsageFault  ) ? (_sivt + 0x18) : . ;  KEEP(*(.isr_vector.__vec_IRQ_UsageFault  ))
    . = DEFINED(__vec_IRQ_SVCall      ) ? (_sivt + 0x2c) : . ;  KEEP(*(.isr_vector.__vec_IRQ_SVCall      ))
    . = DEFINED(__vec_IRQ_DebugMonitor) ? (_sivt + 0x30) : . ;  KEEP(*(.isr_vector.__vec_IRQ_DebugMonitor))
    . = DEFINED(__vec_IRQ_PendSV      ) ? (_sivt + 0x38) : . ;  KEEP(*(.isr_vector.__vec_IRQ_PendSV      ))
    . = DEFINED(__vec_IRQ_SysTick     ) ? (_sivt + 0x3c) : . ;  KEEP(*(.isr_vector.__vec_IRQ_SysTick     ))
}

/* Interrupts */
.isr_vector.irqs : {
    . = DEFINED(__vec_0 ) ? (_sivt + 0x40) : . ;  KEEP(*(.isr_vector.__vec_0 ))
    . = DEFINED(__vec_1 ) ? (_sivt + 0x44) : . ;  KEEP(*(.isr_vector.__vec_1 ))
    . = DEFINED(__vec_2 ) ? (_sivt + 0x48) : . ;  KEEP(*(.isr_vector.__vec_2 ))
    . = DEFINED(__vec_3 ) ? (_sivt + 0x4c) : . ;  KEEP(*(.isr_vector.__vec_3 ))
    . = DEFINED(__vec_4 ) ? (_sivt + 0x50) : . ;  KEEP(*(.isr_vector.__vec_4 ))
    . = DEFINED(__vec_5 ) ? (_sivt + 0x54) : . ;  KEEP(*(.isr_vector.__vec_5 ))
    . = DEFINED(__vec_6 ) ? (_sivt + 0x58) : . ;  KEEP(*(.isr_vector.__vec_6 ))
    . = DEFINED(__vec_7 ) ? (_sivt + 0x5c) : . ;  KEEP(*(.isr_vector.__vec_7 ))
    . = DEFINED(__vec_8 ) ? (_sivt + 0x60) : . ;  KEEP(*(.isr_vector.__vec_8 ))
    . = DEFINED(__vec_9 ) ? (_sivt + 0x64) : . ;  KEEP(*(.isr_vector.__vec_9 ))
    . = DEFINED(__vec_10) ? (_sivt + 0x68) : . ;  KEEP(*(.isr_vector.__vec_10))
    . = DEFINED(__vec_11) ? (_sivt + 0x6c) : . ;  KEEP(*(.isr_vector.__vec_11))
    . = DEFINED(__vec_12) ? (_sivt + 0x70) : . ;  KEEP(*(.isr_vector.__vec_12))
    . = DEFINED(__vec_13) ? (_sivt + 0x74) : . ;  KEEP(*(.isr_vector.__vec_13))
    . = DEFINED(__vec_14) ? (_sivt + 0x78) : . ;  KEEP(*(.isr_vector.__vec_14))
    . = DEFINED(__vec_15) ? (_sivt + 0x7c) : . ;  KEEP(*(.isr_vector.__vec_15))
    . = DEFINED(__vec_16) ? (_sivt + 0x80) : . ;  KEEP(*(.isr_vector.__vec_16))
    . = DEFINED(__vec_17) ? (_sivt + 0x84) : . ;  KEEP(*(.isr_vector.__vec_17))
    . = DEFINED(__vec_18) ? (_sivt + 0x88) : . ;  KEEP(*(.isr_vector.__vec_18))
    . = DEFINED(__vec_19) ? (_sivt + 0x8c) : . ;  KEEP(*(.isr_vector.__vec_19))
    . = DEFINED(__vec_20) ? (_sivt + 0x90) : . ;  KEEP(*(.isr_vector.__vec_20))
    . = DEFINED(__vec_21) ? (_sivt + 0x94) : . ;  KEEP(*(.isr_vector.__vec_21))
    . = DEFINED(__vec_22) ? (_sivt + 0x98) : . ;  KEEP(*(.isr_vector.__vec_22))
    . = DEFINED(__vec_23) ? (_sivt + 0x9c) : . ;  KEEP(*(.isr_vector.__vec_23))
    . = DEFINED(__vec_24) ? (_sivt + 0xa0) : . ;  KEEP(*(.isr_vector.__vec_24))
    . = DEFINED(__vec_25) ? (_sivt + 0xa4) : . ;  KEEP(*(.isr_vector.__vec_25))
    . = DEFINED(__vec_26) ? (_sivt + 0xa8) : . ;  KEEP(*(.isr_vector.__vec_26))
    . = DEFINED(__vec_27) ? (_sivt + 0xac) : . ;  KEEP(*(.isr_vector.__vec_27))
    . = DEFINED(__vec_28) ? (_sivt + 0xb0) : . ;  KEEP(*(.isr_vector.__vec_28))
    . = DEFINED(__vec_29) ? (_sivt + 0xb4) : . ;  KEEP(*(.isr_vector.__vec_29))
    . = DEFINED(__vec_30) ? (_sivt + 0xb8) : . ;  KEEP(*(.isr_vector.__vec_30))
    . = DEFINED(__vec_31) ? (_sivt + 0xbc) : . ;  KEEP(*(.isr_vector.__vec_31))
    . = DEFINED(__vec_32) ? (_sivt + 0xc0) : . ;  KEEP(*(.isr_vector.__vec_32))
    . = DEFINED(__vec_33) ? (_sivt + 0xc4) : . ;  KEEP(*(.isr_vector.__vec_33))
    . = DEFINED(__vec_34) ? (_sivt + 0xc8) : . ;  KEEP(*(.isr_vector.__vec_34))
    . = DEFINED(__vec_35) ? (_sivt + 0xcc) : . ;  KEEP(*(.isr_vector.__vec_35))
    . = DEFINED(__vec_36) ? (_sivt + 0xd0) : . ;  KEEP(*(.isr_vector.__vec_36))
    . = DEFINED(__vec_37) ? (_sivt + 0xd4) : . ;  KEEP(*(.isr_vector.__vec_37))
    . = DEFINED(__vec_38) ? (_sivt + 0xd8) : . ;  KEEP(*(.isr_vector.__vec_38))
    . = DEFINED(__vec_39) ? (_sivt + 0xdc) : . ;  KEEP(*(.isr_vector.__vec_39))
    . = DEFINED(__vec_40) ? (_sivt + 0xe0) : . ;  KEEP(*(.isr_vector.__vec_40))
    . = DEFINED(__vec_41) ? (_sivt + 0xe4) : . ;  KEEP(*(.isr_vector.__vec_41))
    . = DEFINED(__vec_42) ? (_sivt + 0xe8) : . ;  KEEP(*(.isr_vector.__vec_42))
}

/* end. */
