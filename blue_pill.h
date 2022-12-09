/*
    `blue_pill.h` - header file for the Blue Pill board (STM32F103).

    Include tree and dependencies:

        blue_pill.h                 - this file
        '-stm32f1xx.h               + STM peripheral library header
          |-stm32f103xb.h           - library core
          |-{system_stm32f1xx.h}    - {this header is useless, but included}
          '-core_cm3.h              + ARM CMSIS library core
            |-cmsis_version.h       - library version
            '-cmsis_compiler.h      - compiler selection
              '-cmsis_gcc.h         - compiler specific CM3 intrinsics

    Standard CMSIS headers can be downloaded
    from official ST and ARM GitHub repos:

        https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer#stm32cube-cmsis
        https://github.com/ARM-software/CMSIS_5

    Linker scripts are also provided:

        blue_pill.ld                - main linker script
          `-vectors.ld              - interrupts vectors loctions table
*/


#ifndef __BLUE_PILL
#define __BLUE_PILL


    /*  Standard ST and Arm CMSIS headers */

        #define STM32F103xB
        #include <stm32f1xx.h>


    /*  Stack pointer value initialization;
        `_estack` must be provided by a linker script */

        asm("   .pushsection .isr_vector.__sp_val,\"aG\",%progbits,_grp_sp_val,comdat   ");
        asm("      .word _estack                                                        ");
        asm("   .popsection                                                             ");


    /*  Vector definition macro:
            `vn`    - IRQ number or vector name in form of
                     `IRQ_<Vector>`, where `<Vector>` corresponds
                      to a vector Acronym from the Reference Manual;
            `fname` - exception handler function.

        Example:
            void my_handler(void);
            VECTOR(IRQ_NMI, my_handler); */

        #define VECTOR(vn, fname) __VECTOR(vn, fname)

        #define __VECTOR(vn, fname)                         \
            volatile void *__vec_##vn                       \
            __attribute__((section(".isr_vector.__vec_"#vn",\"aG\",%progbits,_grp_vec_"#vn",comdat @;"), used)) \
            = (void*)(fname + 1)


    /*  System exceptions names for using with `VECTOR()` calls
        (they cannot be used with `NVIC_*` CMSIS functions) */
        #define IRQ_Reset               IRQ_Reset
        #define IRQ_NMI                 IRQ_NMI
        #define IRQ_HardFault           IRQ_HardFault
        #define IRQ_MemManage           IRQ_MemManage
        #define IRQ_BusFault            IRQ_BusFault
        #define IRQ_UsageFault          IRQ_UsageFault
        #define IRQ_SVCall              IRQ_SVCall
        #define IRQ_DebugMonitor        IRQ_DebugMonitor
        #define IRQ_PendSV              IRQ_PendSV
        #define IRQ_SysTick             IRQ_SysTick


    /*  IRQs names for using with `VECTOR()` calls instead of numers */
        #define IRQ_WWDG                0
        #define IRQ_PVD                 1
        #define IRQ_TAMPER              2
        #define IRQ_RTC                 3
        #define IRQ_FLASH               4
        #define IRQ_RCC                 5
        #define IRQ_EXTI0               6
        #define IRQ_EXTI1               7
        #define IRQ_EXTI2               8
        #define IRQ_EXTI3               9
        #define IRQ_EXTI4               10
        #define IRQ_DMA1_Channel1       11
        #define IRQ_DMA1_Channel2       12
        #define IRQ_DMA1_Channel3       13
        #define IRQ_DMA1_Channel4       14
        #define IRQ_DMA1_Channel5       15
        #define IRQ_DMA1_Channel6       16
        #define IRQ_DMA1_Channel7       17
        #define IRQ_ADC1_2              18
        #define IRQ_USB_HP_CAN_TX       19
        #define IRQ_USB_LP_CAN_RX0      20
        #define IRQ_CAN_RX1             21
        #define IRQ_CAN_SCE             22
        #define IRQ_EXTI9_5             23
        #define IRQ_TIM1_BRK            24
        #define IRQ_TIM1_UP             25
        #define IRQ_TIM1_TRG_COM        26
        #define IRQ_TIM1_CC             27
        #define IRQ_TIM2                28
        #define IRQ_TIM3                29
        #define IRQ_TIM4                30
        #define IRQ_I2C1_EV             31
        #define IRQ_I2C1_ER             32
        #define IRQ_I2C2_EV             33
        #define IRQ_I2C2_ER             34
        #define IRQ_SPI1                35
        #define IRQ_SPI2                36
        #define IRQ_USART1              37
        #define IRQ_USART2              38
        #define IRQ_USART3              39
        #define IRQ_EXTI15_10           40
        #define IRQ_RTCAlarm            41
        #define IRQ_USBWakeup           42


    /*  Interrupt enable/disable (one at a time) */
        #define INTERRUPT_ENABLE(n)         NVIC->ISER[(n)/32] = 1 << ((n)%32)
        #define INTERRUPT_DISABLE(n)        NVIC->ICER[(n)/32] = 1 << ((n)%32)
        #define INTERRUPT_SET_PENDING(n)    NVIC->ISPR[(n)/32] = 1 << ((n)%32)
        #define INTERRUPT_CLEAR_PENDING(n)  NVIC->ICPR[(n)/32] = 1 << ((n)%32)
        #define INTERRUPT_PRIORITY(n, p)    NVIC->IP[n] = (p) << 4


    /*  Software interrupts
            `n` - softint consecutive number, must start with 0

        Example:
            #define IRQ_MySoftInt   0

            void my_softint_handler(void);
            SOFTINT_VECTOR(IRQ_MySoftInt, my_softint_handler);
            ...
                SOFTINT_ENABLE(IRQ_MySoftInt);
                ...
                SOFTINT_SET_PENDING(IRQ_MySoftInt); */

        #define SOFTINT_VECTOR(vn, fname)   VECTOR(IRQ_Soft(vn), fname)

        /* `SORT_BY_NAME` must be used in a linker script */
        #define IRQ_Soft(n)             IRQ_Soft_##n

        #define _IRQ_Soft_Num               43

        #define SOFTINT_ENABLE(n)           INTERRUPT_ENABLE((n) + _IRQ_Soft_Num)
        #define SOFTINT_DISABLE(n)          INTERRUPT_DISABLE((n) + _IRQ_Soft_Num)
        #define SOFTINT_SET_PENDING(n)      INTERRUPT_SET_PENDING((n) + _IRQ_Soft_Num)
        #define SOFTINT_CLEAR_PENDING(n)    INTERRUPT_CLEAR_PENDING((n) + _IRQ_Soft_Num)
        #define SOFTINT_PRIORITY(n, p)      INTERRUPT_PRIORITY((n) + _IRQ_Soft_Num, p)


    /* Data segment break is stored at `CoreDebug->DCRDR` */

        #define BRKR        (*(volatile uint32_t*)&CoreDebug->DCRDR)

        void* sbrk(uint32_t n) {
            uint32_t
               *b = (uint32_t*)&BRKR,
                t;

            t = *b;
           *b += n;

            return (void*)t;
        }


    /*  Startup code and reset vector initialization
        (NMI, HardFault and other vectors must be defined by user) */

        int main() asm("_start") __attribute__((noreturn));

        #ifndef NO_INIT

            __attribute__((naked, noreturn))
            __attribute__((section(".text.reset_handler,\"axG\",%progbits,_grp_reset_handler,comdat @;")))
            void reset_handler(void) {
                /*
                    Default reset handler.
                */

                /* Symbols must be provided by a linker script */
                extern uint8_t
                    _sidata, _sdata, _edata,
                    _sbss, _ebss;
                extern uint32_t
                    _sivt;

                register uint8_t
                    *src = &_sidata,
                    *dst = &_sdata;

                if (src != dst) {
                    while (dst < &_edata) {
                        *dst++ = *src++;
                    }
                }

                dst = &_sbss;

                while (dst < &_ebss) {
                    *dst++ = 0;
                }

                SCB->VTOR = (uint32_t)&_sivt;

                BRKR = (uint32_t)&_ebss;

                /* main(); */
                asm("b _start");
            }

            VECTOR(IRQ_Reset, reset_handler);

        #else

            VECTOR(IRQ_Reset, main);

        #endif /* NO_INIT */


    /*  Simple wait loop */
        __attribute__((naked))
        void wait_x3_cycles(uint32_t c) {
            asm("   1:                  ");
            asm("       subs r0, #1     ");
            asm("       bne 1b          ");
            asm("   bx lr               ");
        }

        /*  Time wait definitions (accurate only when running from
            flash and when CPU frequency or `n` is divisible by 3):
                `n`     - time to wait;
                `MHz`   - CPU frequency in MHz.

            Example:
                #define F_CPU_MHz 72
                wait_μs(1000, F_CPU_MHz); */

            #define wait_μs(n, MHz) wait_x3_cycles((n)*(MHz)/3)
            #define wait_ms(n, MHz) wait_μs((n)*1000, MHz)
            #define wait_s(n, MHz)  wait_μs((n)*1000000, MHz)


    /*  Converts normal address to bit-band address:
            `addr`  - register address;
            `bit`   - bit number.

        Example:
            #define BB_PC14_OUT  BITBAND(GPIOC->ODR, 14) */

        #define BITBAND(reg, bit)                                       \
            (                                                           \
                *(volatile uint32_t*) (                                 \
                      ( (uint32_t)(&(reg)) & 0xf0000000 ) + 0x02000000  \
                    + ( (uint32_t)(&(reg)) & 0x000fffff ) * 32          \
                    + bit * 4                                           \
                )                                                       \
            )


    /*  GPIO pin configure macro:
            `gpio`          - GPIO address;
            `pin_n`         - pin number;
            `conf_bits`     - 4 bits of configuration.

        Example:
            CONFIGURE_PIN(GPIOC, 13, O_OPEN_DRAIN); */

        #define CONFIGURE_PIN(gpio, pin_n, conf_bits)                   \
            (                                                           \
                *((volatile uint32_t*)gpio + pin_n/8) =                 \
               (*((volatile uint32_t*)gpio + pin_n/8)                   \
                &  ~(0b1111 << pin_n%8*4))                              \
                | conf_bits << pin_n%8*4                                \
            )

            #define I_ANALOG            0b0000
            #define I_FLOATING          0b0100
            #define I_PULLED            0b1000
            #define O_PUSH_PULL         0b0001
            #define O_OPEN_DRAIN        0b0101
            #define O_ALT_PUSH_PULL     0b1001
            #define O_ALT_OPEN_DRAIN    0b1101

            #define BB_PC13_OUT     BITBAND(GPIOC->ODR, 13)
            #define LED             BB_PC13_OUT
            #define LED_ON()        BB_PC13_OUT = 0
            #define LED_OFF()       BB_PC13_OUT = 1
            #define LED_TOGGLE()    BB_PC13_OUT ^= 1


    /*  PLL and SYSCLK configuration;
        (the macro works only after MC reset,
        it will NOT work if PLL is already running):
            `src` - input clock: HSI2, HSE or HSE2,
            `MHz` - PLL output frequency in MHz.

        Possible `MHz` values are:
            for HSI2 and HSE2:
                12, 16,20,24,28,32,36,40,44,48,52,56,60,64
                (8 — not guaranteed)
            for HSE:
                    16,   24,   32,   40,   48,   56,   64,   72
                (80,88,96,104,112,120,128 - not guaranteed)

        Example 1:
            #define F_CPU_MHz 72
            CONFIGURE_PLL(HSE, F_CPU_MHz);

        Example 2:
            #define F_CPU_MHz 8
            SW_EXTERNAL_8MHZ(); */

        #define CONFIGURE_PLL(src, MHz) do {                            \
            FLASH->ACR = 0x10 | (MHz <= 24 ? 0 : (MHz <= 48 ? 1 : 2));  \
            RCC->CFGR  = ((MHz / (src==HSE ? 2:1) - 8) | src) << 16;    \
            RCC->CR    = src==HSI2 ? HSION : HSEON;                     \
            while (!(src==HSI2 ? BB_HSIRDY : BB_HSERDY));               \
            RCC->CR    = PLLON | (src==HSI2 ? HSION : HSEON);           \
            while (!(BB_PLLRDY));                                       \
            RCC->CFGR  = PLL | (MHz > 36 ? RCC_CFGR_PPRE1_DIV2 : 0);    \
        } while(0)

        #define SW_EXTERNAL_8MHZ() do {                                 \
            RCC->CR = HSEON; while (!BB_HSERDY); RCC->CFGR = HSE;       \
        } while(0)

            /* PLL sources, (PLLXTPRE | PLLSRC) */
            #define HSI2    0b00
            #define HSE     0b01
            #define HSE2    0b11

            /* RCC_CFGR_SW value for PLL */
            #define PLL     0b10

            /* RCC_CR bits */
            #define PLLON   (1<<24)
            #define HSEON   (1<<16)
            #define HSION   (1<<0)
            #define BB_PLLRDY  BITBAND(RCC->CR, 25)
            #define BB_HSERDY  BITBAND(RCC->CR, 17)
            #define BB_HSIRDY  BITBAND(RCC->CR, 1)


    /*  Other definitions */

        #define NULL ((void*)0)


#endif /* __BLUE_PILL */

/* end. */
