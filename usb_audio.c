/*****************************************************************************/
/* Description */

    /*
        USB Audio Class capture device for STM32F103.

            - 1 channel¹
            - 192'000 samples per second
            - true 15 bits per sample


        Pins used:
            - PA1   - analog input
            - PC13² - onboard green LED, indicates USB address assignment
            - PC14² - external LED, indicates active state (recording)
            - PB15  - test signal, push-pull output if enabled


        Input range and output format:
            Input is the regular ADC one. No scaling or shifting is done.
            12-bit ADC measures from Vref+ to Vref- (in case of Blue Pill
            they are internally connected to Vdda and Vssa: +3.3 V and
            GND respectively).

            For output, 8 12-bit samples are added together, then the sum
            is multiplied by 2 to compose 16-bit values. A value of 32768
            is subtracted to make samples signed. Endianness is "little".


        Test signal:
            If enabled, it will produce 192000 toggles per second (96 kHz
            square wave) between Vdd and Vss. Even if it's not directly
            connected to the input it will generate a bit of noise.

            When test signal is connected to ADC input a waveform should
            be like alternating samples of a close to maximum amplitude.


        USB device has only one configuration, one interface altsetting,
        no audio controls such as mute or volume.


        ¹ - If someone wants to make 2-channel version, then he must write
        additional proper oversamples summation function, set ADC second
        channel and change channels number and sampling rate in the USB
        configuration descriptor. If done, then one would have 96 ksps
        and only 14 bits per sample.

        ² - Pins PC13-PC15 should only sink or source no more than 3 mA.

    */

/*****************************************************************************/
/* Includes */

    #include "blue_pill.h"
    #include "usb.h"

    asm(".include \"descriptors_audio.s\"");


/*****************************************************************************/
/* Common defines and vars */

    #define F_CPU_MHz       72

    /* Pin will be configured as push-pull output if 1 */
    #define ENABLE_TEST_SIGNAL_ON_PB15      0

    /* Decreases CPU usage even more - to ~6% when running from flash */
    #define USE_FAST_ASM_COPY_FUNCTIOIN     1

    /* LED on PC14 pin indicates active state - recording */
    #define BB_RED_OUT      BITBAND(GPIOC->ODR, 14)
    #define RED             BB_RED_OUT
    #define RED_ON()        BB_RED_OUT = 0
    #define RED_OFF()       BB_RED_OUT = 1
    #define RED_TOGGLE()    BB_RED_OUT ^= 1

    /* Count of 16-bit samples * (8) oversamples - determines buffer size */
    #define ADC_RAW_BUF_CNT     ((8)*(192+52))      /* 3904 bytes */
    uint16_t
        adc_raw_buffer[ADC_RAW_BUF_CNT] __attribute__((aligned(4)));


/*****************************************************************************/
/* USB related functions, vars and defs */

    #define N_EPs           2   /* Total number of endpoints used */

    #define EP0_BUF_SZ      8   /* See `descriptors_audio.s` file */
    #define EP1_BUF_SZ      0   /* Not used in this program */

    const uint32_t
        ep_typ_adr[N_EPs] = { (EP_TYPE_CONTROL | EP0), (EP_TYPE_ISOCHRONOUS | EP1) },
        bufs_sizes[N_EPs] = {  EP0_BUF_SZ,              EP1_BUF_SZ                 };

    #define EPnR_CLEAR_CTR(ep, x)       EPnR_CLEAR_CTR_RW(ep, x, ep_typ_adr[ep])
    #define EPnR_SET_STAT_RX(ep, x)     EPnR_SET_STAT_RX_RW(ep, x, ep_typ_adr[ep])
    #define EPnR_SET_STAT_TX(ep, x)     EPnR_SET_STAT_TX_RW(ep, x, ep_typ_adr[ep])

    int32_t
        device_address,
        current_configuration,      /* Currently not used */
        interface_1_altsetting;

    extern uint16_t
        device_descriptor[],
        config_descriptor[],
       *string_descriptor[];

    /* Setup packet */
    struct {
        uint32_t bmRequestType;
        uint32_t bRequest;
        uint32_t wValue;
        uint32_t wIndex;
        uint32_t wLength;
    } SP;

    /* PMA buffers addresses in normal address space;
       2 pointers (for tx/tx0/rx0 and rx/tx1/rx1 buffers)
       for each of N_EPs endpoints */
    uint32_t
      *pma_bufs[N_EPs][2];

    /* Buffers origins in PMA space (0..511) */
    #define  PMA_BUF0_ORG   (N_EPs * 8)                     /* EP0; same for tx and rx */
    #define  PMA_BUF1_ORG   (PMA_BUF0_ORG + EP0_BUF_SZ)     /* EP1; same for tx0 and tx1 */


    void init_ep0() {
        /* PMA buffers are arrays of `uint32_t`
           which can carry only 16-bit values */
        pma_bufs[EP0][TX] = alloc_pma(EP0, TX, PMA_BUF0_ORG, 0);
        pma_bufs[EP0][RX] = alloc_pma(EP0, RX, PMA_BUF0_ORG, EP0_BUF_SZ);

        /* After reset USB->EPnR == 0; set
           STAT_RX = VALID; STAT_TX = STALL; */
        EPnR[EP0] = ep_typ_adr[EP0] | (VALID<<12) | (STALL<<4);
    }

    void set_ep1_counters(uint32_t cnt) {
        BDT[EP1][TX0].count = cnt;
        BDT[EP1][TX1].count = cnt;
    }

    void init_ep1() {
        /* Single buffer: both pointers point to the same location */
        pma_bufs[EP1][TX0] = alloc_pma(EP1, TX0, PMA_BUF1_ORG, 0);
        pma_bufs[EP1][TX1] = alloc_pma(EP1, TX1, PMA_BUF1_ORG, 0);

        set_ep1_counters(0);

        /* STAT_TX = VALID; */
        EPnR[EP1] = ep_typ_adr[EP1] | (VALID<<4);
    }

    void init_endpoints() {
        init_ep0();
        init_ep1();
    }


    uint32_t reset_dma_counter() {
        /*
            Returns current couter value.

            CNDTR holds number of transfers, not bytes.
        */

        uint32_t
            cndtr;

        cndtr = DMA1_Channel1->CNDTR;

        #define DMA_TRANSFER_SIZE_32  (0b1010 << 8)

        /* To reset counter DMA must be disabled first */
        DMA1_Channel1->CCR = 0;
        DMA1_Channel1->CNDTR = ADC_RAW_BUF_CNT/2;
        asm("@ no push, please");
        DMA1_Channel1->CCR = 1 | DMA_TRANSFER_SIZE_32 | DMA_CCR_MINC;

        return cndtr;
    }

    void copy_to_pma_x8_oversampled(uint16_t *from, uint32_t* to, uint32_t cnt) {
        /*
            Function execution time is ~78 (~120) μs
            when running from flash (RAM) at 72 MHz
            CPU clock. `cnt` is the number of samples
            to be written.
        */

        uint32_t
            sample, i;

        for (i=0; i < cnt; i++) {
            sample = from[i*8 + 0] +
                     from[i*8 + 1] +
                     from[i*8 + 2] +
                     from[i*8 + 3] +
                     from[i*8 + 4] +
                     from[i*8 + 5] +
                     from[i*8 + 6] +
                     from[i*8 + 7];

            /* Only lower 2 bytes will be copied */
            to[i] = sample * 2 - 0x8000;
        }

        /* Test expression: will zero first sample in every USB frame */
        //~ to[0] = 0;
    }

    #if USE_FAST_ASM_COPY_FUNCTIOIN == 1
    __attribute__((naked))
    void copy_to_pma_x8_oversampled_fast(uint16_t *from, uint32_t* to, int32_t cnt) {
        /*
            Function execution time is ~52 (~78) μs
            when running from flash (RAM) at 72 MHz
            CPU clock.

            `from`  - 12-bit data as uint16_t*;
            `to`    - PMA buffer, uint16_t* as uint32_t*;
            `cnt`   - number of samples to be written.
        */

        asm("    push {r4,r5,r6}                    ");
        asm("    b 2f                               ");
        asm("                                       ");
        asm("    1:                                 ");
        asm("        ldmia   r0!, {r3,r4,r5,r6}     ");
        asm("                                       ");
        asm("        add     r3, r4                 ");
        asm("        add     r3, r5                 ");
        asm("        add     r3, r6                 ");
        asm("        add     r3, r3, r3, lsr #16    ");
        asm("                                       ");
        asm("        lsls    r3, #1                 ");
        asm("        sub     r3, #0x8000            ");
        asm("                                       ");
        asm("        str     r3, [r1], #4           ");
        asm("    2:                                 ");
        asm("        subs    r2, #1                 ");
        asm("        bpl 1b                         ");
        asm("                                       ");
        asm("    pop {r4,r5,r6}                     ");
        asm("    bx lr                              ");
    }
    #endif

    #if 0
    void start_systick_ms(uint32_t ms) {
        /*
            Maximum overflow time is 1.864 s.
        */

        if (ms) {
            SysTick->LOAD = ms * 1000 * F_CPU_MHz / 8;
            SysTick->CTRL = 3;
        } else {
            SysTick->CTRL = 0;
            SysTick->VAL  = 0;
        }
    }
    #endif


/*****************************************************************************/
/* General USB functions */

    void ep0_set_stat_tx(uint32_t status) {
        /* Compiles to 7 instructions */
        EPnR_SET_STAT_TX(EP0, status);
    }

    /* Used as arg in `epn_tx()` calls
       in place of `cnt` value when `data` == NULL */
    #define NEXT 0

    void epn_tx(uint32_t ep, uint16_t *data, int32_t cnt) {
        /*
            -1 (any negative) for `bytes_remain` means
            send nothing; if it's 0 - zero-length packet
            will be sent.
        */

        static int32_t
            bytes_remain[N_EPs];
        static uint16_t
           *s_data[N_EPs];

        if (data != NULL) {
            /* Initializatioin */
            s_data[ep] = data;
            bytes_remain[ep] = cnt;
        } else {
            /* On `NEXT` iterations */
            cnt = bytes_remain[ep];
        }

        if (cnt >= 0) {

            if (cnt >= bufs_sizes[ep]) {
                cnt = bufs_sizes[ep];
                bytes_remain[ep] -= cnt;
            } else {
                /* Last packet */
                bytes_remain[ep] = -1;
            }

            /* Will copy even number of bytes (never less);
               `cnt` may be odd only at a final chunk */
            s_data[ep] = copy_to_pma(s_data[ep], pma_bufs[ep][TX], cnt);
            BDT[ep][TX].count = cnt;

            /* Enable transfer */
            ep0_set_stat_tx(VALID);
        }

    }

    void get_descriptor(uint32_t wValue, uint32_t wLength) {
        uint32_t
            desc_type   = wValue >> 8,
            desc_index  = wValue & 0xff;
        uint16_t
           *data,
            cnt;

        switch (desc_type) {
            case DESC_TYPE_DEVICE:
                data = device_descriptor;
                cnt  = DESCRIPTOR_SIZE(data);
                break;
            case DESC_TYPE_CONFIG:
                data = config_descriptor;
                cnt  = CONFIG_DESCRIPTOR_SIZE(data);
                break;
            case DESC_TYPE_STRING:
                data = string_descriptor[desc_index];
                cnt  = DESCRIPTOR_SIZE(data);
                break;
            default:
                ep0_set_stat_tx(STALL);
                    return;
        }

        if (cnt > wLength) {
            cnt = wLength;
        }

        /* Send descriptor */
        epn_tx(EP0, data, cnt);
    }

    #if 0
    void class_request_handler(uint32_t rx_cnt) {
        uint16_t
            data  = 0,
            cnt   = 0;

        switch (SP.bRequest) {
            case 0x01:  /* SET_CUR */
                MUTED = pma_bufs[EP0][RX][0] & 0xff;
                break;
            case 0x81:  /* GET_CUR */
                data = MUTED;
                cnt = 1;
                break;

            default:
                ep0_set_stat_tx(STALL);
                    return;
        }

        /* Send short data or zero-length packet (status) */
        epn_tx(EP0, &data, cnt);
    }
    #endif

    void setup_handler() {
        /* Setup Packet is always 8 bytes */
        SP.bmRequestType = pma_bufs[EP0][RX][0] & 0x60,  /* Type only */
        SP.bRequest      = pma_bufs[EP0][RX][0] >> 8,
        SP.wValue        = pma_bufs[EP0][RX][1],
        SP.wIndex        = pma_bufs[EP0][RX][2],
        SP.wLength       = pma_bufs[EP0][RX][3];

        uint16_t
            data  = 0,
            cnt   = 0;

        if (SP.bmRequestType == 0 /* Standard */) {
            switch (SP.bRequest) {
                case GET_STATUS:
                    cnt = 2;
                    break;
                case SET_ADDRESS:
                    device_address = SP.wValue;
                    break;
                case GET_DESCRIPTOR:
                    /* `wIndex` is Lang ID or zero */
                    get_descriptor(SP.wValue, SP.wLength);
                        return;
                case SET_CONFIGURATION:
                    /* current_configuration = SP.wValue; */
                    break;
                case GET_CONFIGURATION:
                    /* data = current_configuration; */
                    data = 1;   /* There's only one config #1 */
                    cnt = 1;
                    break;
                case SET_INTERFACE:
                    if (SP.wIndex == 1) {
                        interface_1_altsetting = SP.wValue;
                    }
                    break;
                case GET_INTERFACE:
                    if (SP.wIndex == 1) {
                        data = interface_1_altsetting;
                    }
                    cnt = 1;
                    break;
                default:
                    ep0_set_stat_tx(STALL);
                        return;
            }
        } else {
            #if 0
                if (SP.bRequest & 0x80) {
                    /* Handle Class IN request */
                    class_request_handler(0);
                    return;
                } else {
                    /* Wait for Class OUT data; see `ISR_usb()` */
                }
            #else
                ep0_set_stat_tx(STALL);
                return;
            #endif
        }

        /* Send short data or zero-length packet (Status) */
        epn_tx(EP0, &data, cnt);
    }


    void ISR_usb() {
        /*
            Main USB interrupt routine.
        */

        /* Bits SOF and SOFM have same position */
        uint32_t
            istr    = USB->ISTR,
            ep      = istr & 0x0f,
            epnr    = EPnR[ep],
            reset   = istr & USB_ISTR_RESET,
            sof_int = (USB->CNTR & USB_CNTR_SOFM) & (istr & USB_ISTR_SOF),
            setup   = epnr & SETUP,
            rx      = epnr & CTR_RX,
            tx      = epnr & CTR_TX;

        static uint32_t
            ep1_tx_ok = 0;

        void stop_tim1() {
            TIM1->CR1 = 0;
            TIM1->CNT = 0;
        }


        if (reset) {
            /*
                Host must send reset requset to a device
                right after it's been attached.
            */

            init_endpoints();
            USB->ISTR = 0;
            USB->DADDR = USB_DADDR_EF;
            stop_tim1();
            ep1_tx_ok = 0;
            return;
        }


        if (sof_int) {
            /*
                SOF interrupts are enabled after an EP1 tx.

                If data haven't been transmitted in previous
                frame, turn red led off and stop conversions
                and disable SOF interrupts; on Isochronous EPs
                exactly one transaction occurs every frame.
            */

            if (!ep1_tx_ok) {
                RED_OFF();

                /* Timer 1 triggers ADCs; as group conversion
                is used, they remain converting and requesting
                DMA till group end */
                stop_tim1();

                set_ep1_counters(0);

                /* Turn off SOF interrupts */
                USB->CNTR &= ~USB_CNTR_SOFM;
            }

            ep1_tx_ok = 0;

            /* Clear SOF flag */
            USB->ISTR = ~USB_ISTR_SOF;
        }


        if (rx) {
            /*
               On receiving a Control data packet
               MC sets both STATs to NAK.
            */

            if (setup) {
                setup_handler();
            } else {
                #if 0
                /* Staus or Data phase */
                uint32_t
                    rx_cnt = BDT[ep][RX].count & 0x3ff;

                if (rx_cnt) {
                    if (ep == EP0 && SP.bmRequestType != 0) {
                        /* Handle Class OUT request data */
                        class_request_handler(rx_cnt);
                    }
                }
                #endif
            }

            /* Also clears EPnR_SETUP bit */
            EPnR_CLEAR_CTR(ep, CTR_RX);
            EPnR_SET_STAT_RX(ep, VALID);
        }


        if (tx) {
            EPnR_CLEAR_CTR(ep, CTR_TX);

            if (ep == EP0) {

                if (device_address && device_address > 0) {
                    USB->DADDR = USB_DADDR_EF | device_address;
                    device_address = -1;

                    /* Indicate successful address assignment */
                    LED_ON();
                }

                /* Transmit next if there's data;
                   first tx started elsewhere */
                epn_tx(ep, NULL, NEXT);

            } else if (ep == EP1) {

                ep1_tx_ok = 1;

                if (TIM1->CR1 == 0 /* stopped */) {
                    RED_ON();

                    /* Clear excess conversions from previous session */
                    reset_dma_counter();

                    /* Turn on SOF interrupts */
                    USB->ISTR = ~USB_ISTR_SOF;
                    USB->CNTR |= USB_CNTR_SOFM;
                }

                /* Start/continue Timer 1; enable Timer1 CC3 interrupt;
                   at the worst case the interrupt will happen in 5.2 μs */
                TIM1->SR = 0;
                TIM1->DIER = TIM_DIER_CC3IE;
                TIM1->CR1 = 1;
            }
        }

    } /* ISR_usb() */


    void ISR_tim1() {
        /*
            Timer1 CC3 interrupt.

            It is generated a little before CC1 event,
            at this moment previous DMA transactions
            for all 8 oversamples of an audio sample
            should already be over.

            Timer1 CC1 triggers ADC1 group conversion
            while every conversion of a group triggers DMA.
            Thus we have about 14 ADC cycles or 84 CPU
            cycles to reset DMA counter and copy first
            2 samples.

            Interrupt entry/exit costs 12 CPU cycles.
        */

        uint32_t
            samples_done,
            bytes_done,
            cndtr;

        cndtr = reset_dma_counter();

        /* DMA copies 2 samples at a time; there are 8 oversamples */
        samples_done = ADC_RAW_BUF_CNT/8 - cndtr/4;
        bytes_done = samples_done * 2;

        #if USE_FAST_ASM_COPY_FUNCTIOIN == 1
            copy_to_pma_x8_oversampled_fast(adc_raw_buffer, pma_bufs[EP1][TX], samples_done);
        #else
            copy_to_pma_x8_oversampled(adc_raw_buffer, pma_bufs[EP1][TX], samples_done);
        #endif

        set_ep1_counters(bytes_done);

        /* You must use DSB if an interrupt/flag
           is cleared at the handler return */
        TIM1->DIER = 0;
        __DSB();
    }


/*****************************************************************************/
/* Program entry */

    int main() {

        {   /* Configure clocks */
            CONFIGURE_PLL(HSE, F_CPU_MHz);
            RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
        }

        {   /* Enable peripherals used (except USB) */
            RCC->AHBENR |= RCC_AHBENR_DMA1EN;
            RCC->APB2ENR =
                RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN |
                RCC_APB2ENR_TIM1EN;
        }

        {   /* LEDs */
            /* LED_OFF();
               RED_OFF();
               CONFIGURE_PIN(GPIOC, 13, O_OPEN_DRAIN);
               CONFIGURE_PIN(GPIOC, 14, O_OPEN_DRAIN);
               ~= */
            GPIOC->ODR = (1<<13) | (1<<14);
            GPIOC->CRH = (O_OPEN_DRAIN << 13%8*4) | (O_OPEN_DRAIN << 14%8*4);
        }

        {   /* ADCs */
            /**
                ADC1 + ADC2 run @ 12 MHz performing 4 group conversions
                each. ADCs are triggered by Timer 1 CC1 & CC2 events.
                The result is then copyed by DMA1 channel 1 requested
                by ADC1. ADC2 starts 7 cycles before ADC1.

                At 12 MHz clock one ADC can make no more than 4
                conversions per 192k audio sample:
                    - one conversion lasts 14 ADC clock cycles;
                    - 12e6 / 192e3 / 14 ~= 4.46.

                Maximum externel trigger conversion start delay is:
                    2/12 + 1/72 = 0.180 μs (from datasheet)
            **/

            /* CONFIGURE_PIN(GPIOA, 0, I_ANALOG); */
            CONFIGURE_PIN(GPIOA, 1, I_ANALOG);

            #define A1CH 1
            #define A2CH 1
            #define TRG_T1CC1 (0b000 << ADC_CR2_EXTSEL_Pos)
            #define TRG_T1CC2 (0b001 << ADC_CR2_EXTSEL_Pos)

            ADC1->CR1  = ADC_CR1_SCAN;
            ADC2->CR1  = ADC_CR1_SCAN;
            ADC1->SQR1 = (4-1) << ADC_SQR1_L_Pos;
            ADC2->SQR1 = (4-1) << ADC_SQR1_L_Pos;
            ADC1->SQR3 = (A1CH<<(0*5)) | (A1CH<<(1*5)) | (A1CH<<(2*5)) | (A1CH<<(3*5));
            ADC2->SQR3 = (A2CH<<(0*5)) | (A2CH<<(1*5)) | (A2CH<<(2*5)) | (A2CH<<(3*5));
            ADC1->CR2  = 1 | ADC_CR2_EXTTRIG | TRG_T1CC1 | ADC_CR2_DMA;
            ADC2->CR2  = 1 | ADC_CR2_EXTTRIG | TRG_T1CC2;

            /* Calibrate ADCs */
            wait_μs(6, F_CPU_MHz);
            ADC1->CR2 |= ADC_CR2_CAL;
            ADC2->CR2 |= ADC_CR2_CAL;
            while ((ADC1->CR2 | ADC2->CR2) & ADC_CR2_CAL);
        }

        {   /* Timer 1 as ADC sync */
            /**
                TIM1_ARR - a period of ADCs triggering.

                CC3 event has an important role of updating DMA
                counter at the exact momment of time. It also
                used as the test signal generator.

                To make CCx an ADC trigger CCxE and MOE must be enabled.
            **/

            #if ENABLE_TEST_SIGNAL_ON_PB15 == 1
                /* Test PWM signal, 192k Toggles per second */
                CONFIGURE_PIN(GPIOB, 15, O_ALT_PUSH_PULL);
            #endif

            #define CC_PWM2    0b111   /* |_--| */
            #define CC_TOGGLE  0b011

            TIM1->ARR = F_CPU_MHz * 1000 / 192 - 1;
            TIM1->CCR1 = 32 + 7*6;
            TIM1->CCR2 = 32;
            TIM1->CCR3 = 32;
            TIM1->CCMR1 = (CC_PWM2 << 4) | (CC_PWM2 << 12);
            TIM1->CCMR2 = (CC_TOGGLE << 4);
            TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3NE;
            TIM1->BDTR = TIM_BDTR_MOE;

            INTERRUPT_ENABLE(IRQ_TIM1_CC);
        }

        {   /* DMA */
            DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
            DMA1_Channel1->CMAR = (uint32_t)adc_raw_buffer;
        }


        {   /* USB */
            ENABLE_USB_PERIPHERAL(USB_CNTR_RESETM | USB_CNTR_CTRM);
            /** Make USB interrupt preemptible **/
            INTERRUPT_PRIORITY(IRQ_USB_LP_CAN_RX0, 1);
            INTERRUPT_ENABLE(IRQ_USB_LP_CAN_RX0);
        }


        while (PWR /* :) */) {
            /*
                Infinite loop.

                Current CPU load is no more than 10% spent
                on summation of oversamples.
            */

            __WFI();
        }
    }


/*****************************************************************************/
/* Interrupt vectors */

    #if 0
    void ISR_systick() {
        RED_TOGGLE();
    }

    VECTOR(IRQ_SysTick, ISR_systick);
    #endif

    VECTOR(IRQ_TIM1_CC, ISR_tim1);
    VECTOR(IRQ_USB_LP_CAN_RX0, ISR_usb);


/*****************************************************************************/
