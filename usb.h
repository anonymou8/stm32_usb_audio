/*****************************************************************************/
/* Types and macros */

    #define EPnR    ((volatile uint32_t*)USB_BASE)

    /* `rw` in the defines below is (ep_type | pa_addr);
        an example user definition to e.g. clear CTR bits is:

            #define EP0R_CLEAR_CTR(x)   EPnR_CLEAR_CTR_RW(EP0, x, (EP_TYPE_CONTROL | EP0))

        which is used like:

            EP0R_CLEAR_CTR(CTR_RX);
            EP0R_CLEAR_CTR(CTR_TX);
    */

    #define __CTR   (CTR_RX | CTR_TX)

    #define EPnR_TOGGLE_RW(ep, x, rw)       EPnR[ep] = ((rw) | __CTR) | (x)
    #define EPnR_CLEAR_CTR_RW(ep, x, rw)    EPnR[ep] = ((rw) | __CTR) ^ (x)

    #define EPnR_SET_STAT_RX_RW(ep, x, rw)  EPnR[ep] = ((rw) | __CTR) | ((EPnR[ep] & STAT_RX) ^ ((x)<<12))
    #define EPnR_SET_STAT_TX_RW(ep, x, rw)  EPnR[ep] = ((rw) | __CTR) | ((EPnR[ep] & STAT_TX) ^ ((x)<<4))

    enum _endpoint_numbers {
        EP0, EP1, EP2, EP3, EP4, EP5, EP6, EP7
    };

    typedef struct {
        uint32_t addr;
        uint32_t count;
    } _pma_bdt_record[2];

    #define BDT             ((volatile _pma_bdt_record*) 0x40006000)
    #define BDT_END_PMA     ((2*4)*8) /*64; 448 of 512 bytes left */

    enum _alloc_pma_buf_types {
        TX  = 0,    RX  = 1,
        TX0 = 0,    TX1 = 1,
        RX0 = 0,    RX1 = 1
    };

    #define BL_SIZE_2(x) ((x)/2 << 10)
    #define BL_SIZE_32(x) (((x)/32 | 0x20) << 10)

    /* alloc_pma(): PMA buffer allocation macro */
    #define alloc_pma(ep, buf_type, org, cnt) (uint32_t*) (                         \
        BDT[ep][buf_type].addr  = (org),                                            \
        BDT[ep][buf_type].count = (cnt) < 64 ? BL_SIZE_2(cnt) : BL_SIZE_32(cnt),    \
        /* return */ ((uint8_t*)BDT + (org)*2)                                      \
    )

    #define DESCRIPTOR_SIZE(desc)         ((uint8_t*)desc)[0]
    #define CONFIG_DESCRIPTOR_SIZE(desc)  ((uint16_t*)desc)[1]


/*****************************************************************************/
/* Bits and values defines */

    /* USB_EPnR */
    #define CTR_RX      (1<<15)
    #define DTOG_RX     (1<<14)
    #define SW_BUF_TX   (1<<14)
    #define STAT_RX     (3<<12)
    #define SETUP       (1<<11)
    #define CTR_TX      (1<<7)
    #define DTOG_TX     (1<<6)
    #define SW_BUF_RX   (1<<6)
    #define STAT_TX     (3<<4)

    /* Statuses */
    #define DISABLED    0b00
    #define STALL       0b01
    #define NAK         0b10
    #define VALID       0b11

    /* Endpoint types */
    #define EP_TYPE_BULK            (0b00<<9)
    #define EP_TYPE_CONTROL         (0b01<<9)
    #define EP_TYPE_ISOCHRONOUS     (0b10<<9)
    #define EP_TYPE_INTERRUPT       (0b11<<9)

    /* Standard USB requests */
    #define GET_STATUS               0
    #define SET_ADDRESS              5
    #define GET_DESCRIPTOR           6
    #define GET_CONFIGURATION        8
    #define SET_CONFIGURATION        9
    #define GET_INTERFACE           10
    #define SET_INTERFACE           11

    /* USB descriptor types */
    #define DESC_TYPE_DEVICE         1
    #define DESC_TYPE_CONFIG         2
    #define DESC_TYPE_STRING         3
    #define DESC_TYPE_INTERFACE      4
    #define DESC_TYPE_ENDPOINT       5


/*****************************************************************************/
/* Functions */

    /*  PMA copy routines
        (`cnt` must be divisible by 2, if not - extra
        byte next to last one will be copied anyway) */

    __attribute__((naked))
    uint16_t* copy_to_pma(uint16_t* from, uint32_t* to, uint32_t cnt) {
        asm("       cmp     r2, #0          ");
        asm("       beq 2f                  ");
        asm("   1:                          ");
        asm("       ldrh    r3, [r0], #2    ");
        asm("       strh    r3, [r1], #4    ");
        asm("       subs    r2, #2          ");
        asm("       bgt 1b                  ");
        asm("   2:                          ");
        asm("       bx lr                   ");
    }

    __attribute__((naked))
    uint16_t* copy_from_pma(uint32_t* from, uint16_t* to, uint32_t cnt) {
        asm("       cmp     r2, #0          ");
        asm("       beq 2f                  ");
        asm("   1:                          ");
        asm("       ldrh    r3, [r0], #4    ");
        asm("       strh    r3, [r1], #2    ");
        asm("       subs    r2, #2          ");
        asm("       bgt 1b                  ");
        asm("   2:                          ");
        asm("       mov     r0, r1          ");
        asm("       bx lr                   ");
    }


    /*  USB enabling routine
        (system clock must alredy be properly configured;
        IRQ and EF are not set!) */
    #define ENABLE_USB_PERIPHERAL(interrupts_mask) do {             \
        RCC->APB1ENR |= RCC_APB1ENR_USBEN;                          \
        /* After reset USB->CNTR == (PDWN | FRES) */                \
        USB->CNTR = (0<<USB_CNTR_PDWN_Pos) | USB_CNTR_FRES;         \
         wait_μs(1, 72 /* MHz */);                                  \
        USB->CNTR = (0<<USB_CNTR_FRES_Pos) | (interrupts_mask);     \
        USB->ISTR = 0;                                              \
    } while (0)


/*****************************************************************************/
/* Redefine CMSIS' `USB` for 32-bit access */

    /*  This redefine eliminates usage of `ldrh` and corresponding
        halfword conversions. New definition leads to smaller and
        faster executable.

        A cite from the Reference Manual:
            > 23.5 USB REGISTERS
            >   ...
            >   Due to the common limitation of APB1 bridges on word
            >   addressability, all register addresses are aligned
            >   to 32-bit word boundaries although they are 16-bit wide.
            >   ...
            >   The peripheral registers can be accessed by half-words
            >   (16-bit) or words (32-bit). */

    typedef struct {
        __IO uint32_t EP0R;
        __IO uint32_t EP1R;
        __IO uint32_t EP2R;
        __IO uint32_t EP3R;
        __IO uint32_t EP4R;
        __IO uint32_t EP5R;
        __IO uint32_t EP6R;
        __IO uint32_t EP7R;
        __IO uint32_t RESERVED[8];
        __IO uint32_t CNTR;
        __IO uint32_t ISTR;
        __IO uint32_t FNR;
        __IO uint32_t DADDR;
        __IO uint32_t BTABLE;
    } My_USB_TypeDef;

    #undef  USB
    #define USB  ((My_USB_TypeDef*)USB_BASE)

/*****************************************************************************/
