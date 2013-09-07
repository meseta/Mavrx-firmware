#include "config.h"
#include "usb/mw_usbd_rom_api.h"
#include "usb/app_usbd_cfg.h"

// ****************************************************************************
// *** MSC descriptor
// ****************************************************************************
const unsigned char MSC_DeviceDescriptor[] = {
    USB_DEVICE_DESC_SIZE,               /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(0x0200), /* 2.00 */           /* bcdUSB */
    0x00,                               /* bDeviceClass */
    0x00,                               /* bDeviceSubClass */
    0x00,                               /* bDeviceProtocol */
    USB_MAX_PACKET0,                    /* bMaxPacketSize0 */
    WBVAL(USB_VENDOR_ID),               /* idVendor */
    WBVAL(USB_MSC_ID),                  /* idProduct */
    WBVAL(0x0100), /* 1.00 */           /* bcdDevice */
    0x01,                               /* iManufacturer */
    0x02,                               /* iProduct */
    0x03,                               /* iSerialNumber */
    0x01                                /* bNumConfigurations */
};

const unsigned char MSC_ConfigDescriptor[] = {
/* Configuration 1 */
    USB_CONFIGUARTION_DESC_SIZE,        /* bLength */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,  /* bDescriptorType */
    WBVAL(                              /* wTotalLength */
    1*USB_CONFIGUARTION_DESC_SIZE +
    1*USB_INTERFACE_DESC_SIZE     +
    2*USB_ENDPOINT_DESC_SIZE
    ),
    0x01,                               /* bNumInterfaces */
    0x01,                               /* bConfigurationValue */
    0x00,                               /* iConfiguration */
    USB_CONFIG_SELF_POWERED,            /* bmAttributes */
    USB_CONFIG_POWER_MA(100),           /* bMaxPower */
/* Interface 0, Alternate Setting 0, MSC Class */
    USB_INTERFACE_DESC_SIZE,            /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    0x00,                               /* bInterfaceNumber */
    0x00,                               /* bAlternateSetting */
    0x02,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_STORAGE,           /* bInterfaceClass */
    MSC_SUBCLASS_SCSI,                  /* bInterfaceSubClass */
    MSC_PROTOCOL_BULK_ONLY,             /* bInterfaceProtocol */
    0x04,                               /* iInterface */
/* Bulk In Endpoint */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    MSC_EP_IN,                          /* bEndpointAddress */
    USB_ENDPOINT_TYPE_BULK,             /* bmAttributes */
    WBVAL(USB_MAX_BULK_PACKET),         /* wMaxPacketSize */
    0,                                  /* bInterval */
/* Bulk Out Endpoint */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    MSC_EP_OUT,                         /* bEndpointAddress */
    USB_ENDPOINT_TYPE_BULK,             /* bmAttributes */
    WBVAL(USB_MAX_BULK_PACKET),         /* wMaxPacketSize */
    0,                                  /* bInterval */
/* Terminator */
    0                                   /* bLength */
};

const unsigned char MSC_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
    0x04,                               /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(0x0409), /* US English */     /* wLANGID */
/* Index 0x01: Manufacturer */
    (18*2 + 2),                         /* bLength (8 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'N', 0,
    'X', 0,
    'P', 0,
    ' ', 0,
    'S', 0,
    'e', 0,
    'm', 0,
    'i', 0,
    'c', 0,
    'o', 0,
    'n', 0,
    'd', 0,
    'u', 0,
    'c', 0,
    't', 0,
    'o', 0,
    'r', 0,
    's', 0,
/* Index 0x02: Product */
    (14*2 + 2),                         /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'L', 0,
    'P', 0,
    'C', 0,
    '1', 0,
    '3', 0,
    '4', 0,
    '7', 0,
    ' ', 0,
    'M', 0,
    'e', 0,
    'm', 0,
    'o', 0,
    'r', 0,
    'y', 0,
/* Index 0x03: Serial Number */
    (8*2 + 2),                          /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'U', 0,
    'A', 0,
    'i', 0,
    'r', 0,
    ' ', 0,
    'M', 0,
    'S', 0,
    'C', 0,
/* Index 0x04: Interface 0, Alternate Setting 0 */
    (6*2 + 2),                          /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'M', 0,
    'e', 0,
    'm', 0,
    'o', 0,
    'r', 0,
    'y', 0,
};

// ****************************************************************************
// *** HID descriptor
// ****************************************************************************

const uint8_t HID_ReportDescriptor[] = {
    HID_UsagePageVendor ( 0x00                     ),
    HID_Usage           ( 0x01                     ),
    HID_Collection      ( HID_Application          ),
        HID_LogicalMin  ( 0                        ),  /* value range: 0 - 0xFF */
        HID_LogicalMaxS ( 0xFF                     ),
        HID_ReportSize  ( 8                        ),  /* 8 bits */
        HID_ReportCount ( HID_IN_BYTES             ),
        HID_Usage       ( 0x01                     ),
        HID_Input       ( HID_Data | HID_Variable | HID_Absolute ),
        HID_ReportCount ( HID_OUT_BYTES            ),
        HID_Usage       ( 0x01                     ),
        HID_Output      ( HID_Data | HID_Variable | HID_Absolute ),
        HID_ReportCount ( HID_FEATURE_BYTES        ),
        HID_Usage       ( 0x01                     ),
        HID_Feature     ( HID_Data | HID_Variable | HID_Absolute ),
    HID_EndCollection,
};
const uint16_t HID_ReportDescSize = sizeof(HID_ReportDescriptor);

const uint8_t HID_DeviceDescriptor[] = {
    USB_DEVICE_DESC_SIZE,               /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(0x0200), /* 2.00 */           /* bcdUSB */
    0x00,                               /* bDeviceClass */
    0x00,                               /* bDeviceSubClass */
    0x00,                               /* bDeviceProtocol */
    USB_MAX_PACKET0,                    /* bMaxPacketSize0 */
    WBVAL(USB_VENDOR_ID),               /* idVendor */
    WBVAL(USB_HID_ID),                  /* idProduct */
    WBVAL(0x0100), /* 1.00 */           /* bcdDevice */
    0x01,                               /* iManufacturer */
    0x02,                               /* iProduct */
    0x03,                               /* iSerialNumber */
    0x01                                /* bNumConfigurations */
};

const uint8_t HID_ConfigDescriptor[] = {
/* Configuration 1 */
    USB_CONFIGUARTION_DESC_SIZE,        /* bLength */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,  /* bDescriptorType */
    WBVAL(                              /* wTotalLength */
    1*USB_CONFIGUARTION_DESC_SIZE +
    1*USB_INTERFACE_DESC_SIZE     +
    HID_DESC_SIZE                 +
    2*USB_ENDPOINT_DESC_SIZE
    ),
    0x01,                               /* bNumInterfaces */
    0x01,                               /* bConfigurationValue */
    0x00,                               /* iConfiguration */
    USB_CONFIG_SELF_POWERED,            /* bmAttributes */
    USB_CONFIG_POWER_MA(100),           /* bMaxPower */
/* Interface 0, Alternate Setting 0, HID Class */
    USB_INTERFACE_DESC_SIZE,            /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    0x00,                               /* bInterfaceNumber */
    0x00,                               /* bAlternateSetting */
    0x02,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_HUMAN_INTERFACE,   /* bInterfaceClass */
    HID_SUBCLASS_NONE,                  /* bInterfaceSubClass */
    HID_PROTOCOL_NONE,                  /* bInterfaceProtocol */
    0x04,                               /* iInterface */
/* HID Class Descriptor */
/* HID_DESC_OFFSET = 0x0012 */
    HID_DESC_SIZE,                      /* bLength */
    HID_HID_DESCRIPTOR_TYPE,            /* bDescriptorType */
    WBVAL(0x0100), /* 1.00 */           /* bcdHID */
    0x00,                               /* bCountryCode */
    0x01,                               /* bNumDescriptors */
    HID_REPORT_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(sizeof(HID_ReportDescriptor)),/* wDescriptorLength */
/* Endpoint, HID Interrupt In */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    HID_EP_IN,                          /* bEndpointAddress */
    USB_ENDPOINT_TYPE_INTERRUPT,        /* bmAttributes */
    WBVAL(HID_IN_BYTES),                /* wMaxPacketSize */
    HID_RATE,                           /* bInterval */
/* Endpoint, HID Interrupt Out */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    HID_EP_OUT,                         /* bEndpointAddress */
    USB_ENDPOINT_TYPE_INTERRUPT,        /* bmAttributes */
    WBVAL(HID_OUT_BYTES),               /* wMaxPacketSize */
    HID_RATE,                           /* bInterval */
/* Terminator */
    0                                   /* bLength */
};

const uint8_t HID_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
    0x04,                               /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(0x0409), /* US English */     /* wLANGID */
/* Index 0x01: Manufacturer */
    (18*2 + 2),                         /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'N', 0,
    'X', 0,
    'P', 0,
    ' ', 0,
    'S', 0,
    'e', 0,
    'm', 0,
    'i', 0,
    'c', 0,
    'o', 0,
    'n', 0,
    'd', 0,
    'u', 0,
    'c', 0,
    't', 0,
    'o', 0,
    'r', 0,
    's', 0,
/* Index 0x02: Product */
    (12*2 + 2),                         /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'L', 0,
    'P', 0,
    'C', 0,
    '1', 0,
    '3', 0,
    '4', 0,
    '7', 0,
    ' ', 0,
    'H', 0,
    'I', 0,
    'D', 0,
    ' ', 0,
/* Index 0x03: Serial Number */
    (8*2 + 2),                          /* bLength (8 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'U', 0,
    'A', 0,
    'i', 0,
    'r', 0,
    ' ', 0,
    'H', 0,
    'I', 0,
    'D', 0,
/* Index 0x04: Interface 0, Alternate Setting 0 */
    (3*2 + 2),                          /* bLength (3 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'H', 0,
    'I', 0,
    'D', 0,
};

// ****************************************************************************
// *** CDC descriptor
// ****************************************************************************

const uint8_t CDC_DeviceDescriptor[] = {
    USB_DEVICE_DESC_SIZE,               /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,         /* bDescriptorType */
    WBVAL(0x0200), /* 2.0 */            /* bcdUSB */
    USB_DEVICE_CLASS_COMMUNICATIONS,    /* bDeviceClass CDC*/
    0x00,                               /* bDeviceSubClass */
    0x00,                               /* bDeviceProtocol */
    USB_MAX_PACKET0,                    /* bMaxPacketSize0 */
    WBVAL(USB_VENDOR_ID),               /* idVendor */
    WBVAL(USB_CDC_ID),                  /* idProduct */
    WBVAL(0x0100), /* 1.00 */           /* bcdDevice */
    0x01,                               /* iManufacturer */
    0x02,                               /* iProduct */
    0x03,                               /* iSerialNumber */
    0x01                                /* bNumConfigurations */
};

const uint8_t CDC_ConfigDescriptor[] = {
/* Configuration 1 */
    USB_CONFIGUARTION_DESC_SIZE,        /* bLength */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,  /* bDescriptorType */
    WBVAL(                              /* wTotalLength */
    1*USB_CONFIGUARTION_DESC_SIZE +
    1*USB_INTERFACE_DESC_SIZE     +     /* communication interface */
    0x0013                        +     /* CDC functions */
    1*USB_ENDPOINT_DESC_SIZE      +     /* interrupt endpoint */
    1*USB_INTERFACE_DESC_SIZE     +     /* data interface */
    2*USB_ENDPOINT_DESC_SIZE            /* bulk endpoints */
      ),
    0x02,                               /* bNumInterfaces */
    0x01,                               /* bConfigurationValue: 0x01 is used to select this configuration */
    0x00,                               /* iConfiguration: no string to describe this configuration */
    USB_CONFIG_BUS_POWERED /*|*/        /* bmAttributes */
/*USB_CONFIG_REMOTE_WAKEUP*/,
    USB_CONFIG_POWER_MA(100),           /* bMaxPower, device power consumption is 100 mA */
/* Interface 0, Alternate Setting 0, Communication class interface descriptor */
    USB_INTERFACE_DESC_SIZE,            /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    USB_CDC_CIF_NUM,                    /* bInterfaceNumber: Number of Interface */
    0x00,                               /* bAlternateSetting: Alternate setting */
    0x01,                               /* bNumEndpoints: One endpoint used */
    CDC_COMMUNICATION_INTERFACE_CLASS,  /* bInterfaceClass: Communication Interface Class */
    CDC_ABSTRACT_CONTROL_MODEL,         /* bInterfaceSubClass: Abstract Control Model */
    0x00,                               /* bInterfaceProtocol: no protocol used */
    0x04,                               /* iInterface: */
/*Header Functional Descriptor*/
    0x05,                               /* bLength: Endpoint Descriptor size */
    CDC_CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
    CDC_HEADER,                         /* bDescriptorSubtype: Header Func Desc */
    WBVAL(CDC_V1_10), /* 1.10 */        /* bcdCDC */
/*Call Management Functional Descriptor*/
    0x05,                               /* bFunctionLength */
    CDC_CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
    CDC_CALL_MANAGEMENT,                /* bDescriptorSubtype: Call Management Func Desc */
    0x01,                               /* bmCapabilities: device handles call management */
    0x01,                               /* bDataInterface: CDC data IF ID */
/*Abstract Control Management Functional Descriptor*/
    0x04,                               /* bFunctionLength */
    CDC_CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
    CDC_ABSTRACT_CONTROL_MANAGEMENT,    /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,                               /* bmCapabilities: SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */
/*Union Functional Descriptor*/
    0x05,                               /* bFunctionLength */
    CDC_CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
    CDC_UNION,                          /* bDescriptorSubtype: Union func desc */
    USB_CDC_CIF_NUM,                    /* bMasterInterface: Communication class interface is master */
    USB_CDC_DIF_NUM,                    /* bSlaveInterface0: Data class interface is slave 0 */
/*Endpoint 1 Descriptor*/               /* event notification (optional) */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    USB_CDC_EP_INT_IN,                  /* bEndpointAddress */
    USB_ENDPOINT_TYPE_INTERRUPT,        /* bmAttributes */
    WBVAL(0x0010),                      /* wMaxPacketSize */
    0x02,          /* 2ms */            /* bInterval */
/* Interface 1, Alternate Setting 0, Data class interface descriptor*/
    USB_INTERFACE_DESC_SIZE,            /* bLength */
    USB_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    USB_CDC_DIF_NUM,                    /* bInterfaceNumber: Number of Interface */
    0x00,                               /* bAlternateSetting: no alternate setting */
    0x02,                               /* bNumEndpoints: two endpoints used */
    CDC_DATA_INTERFACE_CLASS,           /* bInterfaceClass: Data Interface Class */
    0x00,                               /* bInterfaceSubClass: no subclass available */
    0x00,                               /* bInterfaceProtocol: no protocol used */
    0x04,                               /* iInterface: */
/* Endpoint, EP3 Bulk Out */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    USB_CDC_EP_BULK_OUT,                /* bEndpointAddress */
    USB_ENDPOINT_TYPE_BULK,             /* bmAttributes */
    WBVAL(USB_MAX_BULK_PACKET),         /* wMaxPacketSize */
    0x00,                               /* bInterval: ignore for Bulk transfer */
/* Endpoint, EP3 Bulk In */
    USB_ENDPOINT_DESC_SIZE,             /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    USB_CDC_EP_BULK_IN,                 /* bEndpointAddress */
    USB_ENDPOINT_TYPE_BULK,             /* bmAttributes */
    WBVAL(USB_MAX_BULK_PACKET),         /* wMaxPacketSize */
    0x00,                               /* bInterval: ignore for Bulk transfer */
/* Terminator */
    0                                   /* bLength */
};

__attribute__ ((aligned(4))) const uint8_t CDC_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
    0x04,                              /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    WBVAL(0x0409), /* US English */    /* wLANGID */
/* Index 0x01: Manufacturer */
    (18*2 + 2),                         /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'N', 0,
    'X', 0,
    'P', 0,
    ' ', 0,
    'S', 0,
    'e', 0,
    'm', 0,
    'i', 0,
    'c', 0,
    'o', 0,
    'n', 0,
    'd', 0,
    'u', 0,
    'c', 0,
    't', 0,
    'o', 0,
    'r', 0,
    's', 0,
/* Index 0x02: Product */
    (12*2 + 2),                         /* bLength (13 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'L', 0,
    'P', 0,
    'C', 0,
    '1', 0,
    '3', 0,
    '4', 0,
    '7', 0,
    ' ', 0,
    'V', 0,
    'C', 0,
    'O', 0,
    'M', 0,
/* Index 0x03: Serial Number */
    (8*2 + 2),                          /* bLength (8 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    'U', 0,
    'A', 0,
    'i', 0,
    'r', 0,
    ' ', 0,
    'C', 0,
    'D', 0,
    'C', 0,
/* Index 0x04: Interface 0, Alternate Setting 0 */
    ( 4*2 + 2),                        /* bLength (4 Char + Type + lenght) */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'V',0,
    'C',0,
    'O',0,
    'M',0,
};