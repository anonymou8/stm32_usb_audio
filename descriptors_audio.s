.pushsection .rodata.descriptors

/*************************************************************************************/
/* Symbols */

    FREQ = 192000

    EP0_BUF_SZ = 8      /* Must cohere to C definition */
    EP1_BUF_SZ = 512    /* Max */

    VID = 0xf055        /* Unofficial FOSS VID */
    PID = 0xf055

    DESC_TYPE_DEVICE    = 1
    DESC_TYPE_CONFIG    = 2
    DESC_TYPE_STRING    = 3
    DESC_TYPE_INTERFACE = 4
    DESC_TYPE_ENDPOINT  = 5


/*************************************************************************************/
/* Device descriptor */

    .align 1

    device_descriptor:
        .byte       DD_SZ               @; bLength
        .byte       DESC_TYPE_DEVICE    @; bDescriptorType
        .2byte      0x0200              @; bcdUSB                   = USB 2.0
        .byte       0x00                @; bDeviceClass             = Interface defined
        .byte       0x00                @; bDeviceSubClass
        .byte       0x00                @; bDeviceProtocol
        .byte       EP0_BUF_SZ          @; bMaxPacketSize0          : 8, 16, 32, 64
        .2byte      VID                 @; idVendor
        .2byte      PID                 @; idProduct
        .2byte      0                   @; bcdDevice
        .byte       1                   @; iManufacturer            : String index
        .byte       2                   @; iProduct                 : String index
        .byte       0                   @; iSerialNumber            : String index = nothing
        .byte       1                   @; bNumConfigurations
    DD_SZ = . - device_descriptor


/*************************************************************************************/
/* Config descriptor */

    .align 1

    config_descriptor:              /* Configuration Descriptor */
        .byte   CD_SZ               @; bLength
        .byte   DESC_TYPE_CONFIG    @; bDescriptorType
        .2byte  CD_TOTAL_SZ         @; wTotalLength
        .byte   2                   @; bNumInterfaces
        .byte   1                   @; bConfigurationValue
        .byte   0                   @; iConfiguration           : String index
        .byte   0x80                @; bmAttributes             = Bus powered
        .byte   100/2               @; bMaxPower                : In 2 mA units
    CD_SZ = . - config_descriptor

        /* First interface (Audio Class, no endpoints) */
        interface_descriptor_ac:        /* Standard AC Interface Descriptor */
            .byte   IDAC_SZ             @; bLength
            .byte   DESC_TYPE_INTERFACE @; bDescriptorType
            .byte   0                   @; bInterfaceNumber         : Zero based
            .byte   0                   @; bAlternateSetting
            .byte   0                   @; bNumEndpoints
            .byte   0x01                @; bInterfaceClass          = AUDIO
            .byte   0x01                @; bInterfaceSubClass       = AUDIO_CONTROL
            .byte   0                   @; bInterfaceProtocol       = No protocol
            .byte   0                   @; iInterface
        IDAC_SZ = . - interface_descriptor_ac

            csi_descriptor_ac_hdr:          /* AC Interface Header Descriptor */
                .byte   CSH_SZ              @; bLength
                .byte   0x24                @; bDescriptorType      = CS_INTERFACE
                .byte   0x01                @; bDescriptorSubtype   = HEADER
                .2byte  0x0100              @; bcdADC: revision of class specification 1.00
                .2byte  CS_TOTAL_SZ         @; wTotalLength
                .byte   1                   @; bInCollection        = One streaming interface:
                .byte   1                   @; baInterfaceNr        = [1]
            CSH_SZ = . - csi_descriptor_ac_hdr

                csi_descriptor_ac_it:           /* Input Terminal Descriptor */
                    .byte   CSIT_SZ             @; bLength
                    .byte   0x24                @; bDescriptorType      = CS_INTERFACE
                    .byte   0x02                @; bDescriptorSubtype   = INPUT_TERMINAL
                    .byte   (1)                 @; bTerminalID
                    .2byte  0x0201              @; wTerminalType        : Mic = 0x0201, undefined = 0x0200
                    .byte   0                   @; bAssocTerminal
                    .byte   1                   @; bNrChannels
                    .2byte  0x00                @; wChannelConfig
                    .byte   0                   @; iChannelNames
                    .byte   0                   @; iTerminal
                CSIT_SZ = . - csi_descriptor_ac_it

            @;  csi_descriptor_ac_fu:
            @;      .byte   CSFU_SZ             @; bLength
            @;      .byte   0x24                @; bDescriptorType      = CS_INTERFACE
            @;      .byte   0x06                @; bDescriptorSubtype   = FEATURE_UNIT
            @;      .byte   (2)                 @; bUnitID
            @;      .byte   (1)                 @; bSourceID
            @;      .byte   1                   @; bControlSize         = 1 byte (size of a bmaControls[] element)
            @;      .byte   0x01                @; bmaControls[0]       = Mute (Master)
            @;      .byte   0x00                @; bmaControls[1]       = None (Channel 1)
            @;      .byte   0                   @; iFeature
            @;  CSFU_SZ = . - csi_descriptor_ac_fu

                csi_descriptor_ac_ot:           /* Output Terminal Descriptor */
                    .byte   CSOT_SZ             @; bLength
                    .byte   0x24                @; bDescriptorType      = CS_INTERFACE
                    .byte   0x03                @; bDescriptorSubtype   = OUTPUT_TERMINAL
                    .byte   (3)                 @; bTerminalID
                    .2byte  0x0101              @; wTerminalType        = USB streaming
                    .byte   0                   @; bAssocTerminal
                    .byte   (1)                 @; bSourceID
                    .byte   0                   @; iTerminal
                CSOT_SZ = . - csi_descriptor_ac_ot

            CS_TOTAL_SZ = . - csi_descriptor_ac_hdr

        /* Second interface (Audio Streaming, 1 endpoint) */
        interface_descriptor_as_alt:    /* Standard AS interface descriptor (Alt. Set. 0) */
            .byte   IDAS_ALT_SZ         @; bLength
            .byte   DESC_TYPE_INTERFACE @; bDescriptorType
            .byte   1                   @; bInterfaceNumber         : Zero based
            .byte   0                   @; bAlternateSetting
            .byte   0                   @; bNumEndpoints            = No endpoints interface altsetting
            .byte   0x01                @; bInterfaceClass          = AUDIO
            .byte   0x02                @; bInterfaceSubClass       = AUDIO_STREAMING
            .byte   0                   @; bInterfaceProtocol       : Not used, must be set to 0
            .byte   0                   @; iInterface
        IDAS_ALT_SZ = . - interface_descriptor_as_alt
      
        interface_descriptor_as:        /* Standard AS Interface Descriptor */
            .byte   IDAS_SZ             @; bLength
            .byte   DESC_TYPE_INTERFACE @; bDescriptorType
            .byte   1                   @; bInterfaceNumber         : Zero based
            .byte   1                   @; bAlternateSetting
            .byte   1                   @; bNumEndpoints
            .byte   0x01                @; bInterfaceClass          = AUDIO
            .byte   0x02                @; bInterfaceSubClass       = AUDIO_STREAMING
            .byte   0                   @; bInterfaceProtocol       : Not used, must be set to 0
            .byte   0                   @; iInterface
        IDAS_SZ = . - interface_descriptor_as

            csi_descriptor_as:              /* Class-specific AS General Interface Descriptor */
                .byte   CSAS_SZ             @; bLength
                .byte   0x24                @; bDescriptorType      = CS_INTERFACE
                .byte   0x01                @; bDescriptorSubtype   = GENERAL
                .byte   (3)                 @; bTerminalLink
                .byte   1                   @; bDelay               : In frames
                .2byte  0x0001              @; wFormatTag           = PCM
            CSAS_SZ = . - csi_descriptor_as

            csi_descriptor_as_fmt:          /* Type I Format Type Descriptor */
                .byte   CSASF_SZ            @; bLength
                .byte   0x24                @; bDescriptorType      = CS_INTERFACE
                .byte   0x02                @; bDescriptorSubtype   = FORMAT_TYPE
                .byte   0x01                @; bFormatType          = FORMAT_TYPE_I
                .byte   1                   @; bNrChannels
                .byte   2                   @; bSubFrameSize        : Bytes
                .byte   16                  @; bBitResolution       : MSBs
                .byte   1                   @; bSamFreqType         = One supported frequency
                .byte   (FREQ) & 0xff       @;
                .byte   (FREQ>>8) & 0xff    @;
                .byte   (FREQ>>16) & 0xff   @; tSamFreq
            CSASF_SZ = . - csi_descriptor_as_fmt

                endpoint_descriptor:            /* Standard AS Isochronous Audio Data Endpoint Descriptor */
                    .byte   EPD_SZ              @; bLength
                    .byte   DESC_TYPE_ENDPOINT  @; bDescriptorType
                    .byte   1 | 0x80            @; bEndpointAddress     = IN 1
                    .byte   0x01                @; bmAttributes         = Isochronous, no sync
                    .2byte  EP1_BUF_SZ          @; wMaxPacketSize
                    .byte   1                   @; bInterval            : 2^(bInterval - 1) frames
                    .byte   0                   @; bRefresh             : Not used
                    .byte   0                   @; bSynchAddress        : Not used
                EPD_SZ = . - endpoint_descriptor

                cse_descriptor:                 /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
                    .byte   CSED_SZ             @; bLength
                    .byte   0x25                @; bDescriptorType      = CS_ENDPOINT
                    .byte   0x01                @; bDescriptorSubtype   = GENERAL
                    .byte   0x00                @; bmAttributes         = None
                    .byte   0x00                @; bLockDelayUnits      : Not used
                    .2byte  0x0000              @; wLockDelay           : Not used
                CSED_SZ = . - cse_descriptor

    CD_TOTAL_SZ = . - config_descriptor


/*************************************************************************************/
/* USB-string macro */

    .macro StrDesc label, data, is_lang=0
        \label :
            .byte       "\label\()_SZ"
            .byte       DESC_TYPE_STRING

            .if \is_lang
                .2byte      "\data"
                .set        "\label\()_SZ", ( . - \label)
            .else
                .string16   "\data"
                .set        "\label\()_SZ", ( . - \label - 2)
            .endif
    .endm


/*************************************************************************************/
/* String descriptors */

    .align 1

    string_descriptor:
        .word  sd0_Lang
        .word  sd1_Manu
        .word  sd2_Prod

    LANG_EN_US = 0x0409

    StrDesc  sd0_Lang,  LANG_EN_US, is_lang=1
    StrDesc  sd1_Manu,  "WTF"
    StrDesc  sd2_Prod,  "Prod"


/*************************************************************************************/

.align 1
.popsection
