#include "usb_desc.h"
#include "tusb_cdc_acm.h"

#define MS_OS_20_DESC_LEN 0xB2

// 根据 MSC 是否启用，定义 BOS 总长度
#ifdef CONFIG_TINYUSB_MSC_ENABLED
    #define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#else
    #define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN)
#endif

// 根据 DAPLINK 配置，定义 DAPLINK 描述符长度
#ifdef CONFIG_BULK_DAPLINK
    #define DAPLINK_DESC_LEN TUD_VENDOR_DESC_LEN
#else
    #define DAPLINK_DESC_LEN TUD_HID_INOUT_DESC_LEN
#endif

// 根据 MSC 是否启用，定义总描述符长度
#ifdef CONFIG_TINYUSB_MSC_ENABLED
    #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN + TUD_CDC_DESC_LEN + DAPLINK_DESC_LEN)
#else
    #define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + DAPLINK_DESC_LEN)
#endif

// HID 报告描述符
static uint8_t const desc_hid_dap_report[] = {TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)};

// 条件编译 MSC 相关的配置描述符
#ifdef CONFIG_TINYUSB_MSC_ENABLED
    // 定义带 MSC 的配置描述符
    static uint8_t const desc_configuration_total_enable_msc[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100)
    };

    // 定义 MSC 描述符
    static uint8_t const desc_configuration_msc[] = {
        TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, STRID_MSC_INTERFACE, EDPT_MSC_OUT, EDPT_MSC_IN, TUD_OPT_HIGH_SPEED ? 512 : 64)
    };

    // 定义不带 MSC 的配置描述符，即使 MSC 被启用
    static uint8_t const desc_configuration_total_disable_msc[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, (TUSB_DESC_TOTAL_LEN - TUD_MSC_DESC_LEN), TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100)
    };
#else
    // 定义不带 MSC 的配置描述符
    static uint8_t const desc_configuration_total_disable_msc[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100)
    };
#endif // CONFIG_TINYUSB_MSC_ENABLED

// 定义 CDC 描述符
static uint8_t const desc_configuration_cdc[] = {
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC_INTERFACE, EDPT_CDC_NOTIFY, 8, EDPT_CDC_OUT, EDPT_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE)
};

// 条件编译定义 Vendor 或 HID 描述符
#ifdef CONFIG_BULK_DAPLINK
    static uint8_t const desc_configuration_vendor[] = {
        TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, STRID_VENDOR_INTERFACE, EDPT_VENDOR_OUT, EDPT_VENDOR_IN, TUD_OPT_HIGH_SPEED ? 512 : 64)
    };
#else
    static uint8_t const desc_configuration_hid[] = {
        TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, STRID_HID_INTERFACE, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_dap_report), EDPT_HID_OUT, EDPT_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 1)
    };
#endif // CONFIG_BULK_DAPLINK

// 设备描述符
static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
#ifdef CONFIG_BULK_DAPLINK
    .bcdUSB = 0x0210,
#else
    .bcdUSB = 0x0200,
#endif
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = CONFIG_TINYUSB_DESC_CUSTOM_VID,
    .idProduct = CONFIG_TINYUSB_DESC_CUSTOM_PID,
    .bcdDevice = CONFIG_TINYUSB_DESC_BCD_DEVICE,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

// 字符串描述符结构体
static struct {
    const char *array[STRID_NUM];
    int string_num;
    uint8_t desc_configuration[TUSB_DESC_TOTAL_LEN];
    int configuration_length;
} descriptor = {0};

// 条件编译 Vendor 描述符相关的回调
#ifdef CONFIG_TINYUSB_VENDOR_ENABLED
    #ifdef CONFIG_TINYUSB_MSC_ENABLED
        uint8_t const *tud_descriptor_bos_cb(void) {
            // BOS Descriptor 包含 webUSB 和 Microsoft OS
            static uint8_t const desc_bos[] = {
                TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),                                    // 总长度，设备能力数量
                TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),                     // Vendor Code, iLandingPage
                TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT) // Microsoft OS 2.0 描述符
            };
            return desc_bos;
        }

        uint8_t *get_ms_descriptor(void) {
            static uint8_t desc_ms_os_20[] = {
                // Set header: length, type, windows version, total length
                U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
                // Configuration subset header: length, type, configuration index, reserved, configuration total length
                U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),
                // Function Subset header: length, type, first interface, reserved, subset length
                U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),
                // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
                U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible
                // MS OS 2.0 Registry property descriptor: length, type
                U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
                U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
                'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00,
                't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00,
                'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
                U16_TO_U8S_LE(0x0050), // wPropertyDataLength
                // bPropertyData: “{CDB3B5AD-293B-4663-AA36-1AAE46463776}”
                '{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00, 'A', 0x00,
                'D', 0x00, '-', 0x00, '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00,
                '6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00, 'A', 0x00, 'A', 0x00, '3', 0x00, '6', 0x00,
                '-', 0x00, '1', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '4', 0x00, '6', 0x00, '4', 0x00,
                '6', 0x00, '3', 0x00, '7', 0x00, '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
            };
    
            TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");
    
            return desc_ms_os_20;
        }
    #endif // CONFIG_TINYUSB_MSC_ENABLED

    // 获取 webUSB URL
    tusb_desc_webusb_url_t *get_webusb_url(void) {
        static tusb_desc_webusb_url_t desc_url = {
            3 + sizeof(CONFIG_WEBUSB_URL) - 1,
            3, // WEBUSB URL type
            1, // 0: http, 1: https
            CONFIG_WEBUSB_URL
        };
        return &desc_url;
    }
#endif // CONFIG_TINYUSB_VENDOR_ENABLED

// HID 报告描述符回调
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return desc_hid_dap_report;
}

// 获取设备描述符
tusb_desc_device_t *get_device_descriptor(void) {
    return &descriptor_config;
}

// 获取字符串描述符
const char **get_string_descriptor(bool enable_msc) {
    if (descriptor.string_num) {
        return descriptor.array;
    }

    // 支持的语言是英语 (0x0409)
    static const char desc_string_language[] = {0x09, 0x04};

    descriptor.array[STRID_LANGID] = desc_string_language;
    descriptor.array[STRID_MANUFACTURER] = CONFIG_TINYUSB_DESC_MANUFACTURER_STRING;
    descriptor.array[STRID_PRODUCT] = CONFIG_TINYUSB_DESC_PRODUCT_STRING;
    descriptor.array[STRID_SERIAL_NUMBER] = CONFIG_TINYUSB_DESC_SERIAL_STRING;
    descriptor.array[STRID_CDC_INTERFACE] = CONFIG_TINYUSB_DESC_CDC_STRING;
    descriptor.array[STRID_DAPLINK_INTERFACE] = CONFIG_DAPLINK_DESC_STRING;
    descriptor.string_num = STRID_NUM - 1;

#ifdef CONFIG_TINYUSB_MSC_ENABLED
    if (enable_msc) {
        descriptor.string_num = STRID_NUM;
        descriptor.array[STRID_MSC_INTERFACE] = CONFIG_TINYUSB_DESC_MSC_STRING;
    }
#endif // CONFIG_TINYUSB_MSC_ENABLED

    return descriptor.array;
}

// 获取字符串描述符数量
int get_string_descriptor_count(void) {
    return descriptor.string_num;
}

// 获取配置描述符
const uint8_t *get_configuration_descriptor(bool enable_msc) {
    if (descriptor.configuration_length) {
        return descriptor.desc_configuration;
    }

    uint8_t *p = descriptor.desc_configuration;

#ifdef CONFIG_TINYUSB_MSC_ENABLED
    uint8_t const *total = (enable_msc) ? (desc_configuration_total_enable_msc) : (desc_configuration_total_disable_msc);
    int total_size = (enable_msc) ? (sizeof(desc_configuration_total_enable_msc)) : (sizeof(desc_configuration_total_disable_msc));
#else
    uint8_t const *total = desc_configuration_total_disable_msc;
    int total_size = sizeof(desc_configuration_total_disable_msc);
#endif

    descriptor.configuration_length = 0;
    memcpy(p, total, total_size);
    descriptor.configuration_length += total_size;
    memcpy(p + descriptor.configuration_length, desc_configuration_cdc, sizeof(desc_configuration_cdc));
    descriptor.configuration_length += sizeof(desc_configuration_cdc);

#ifdef CONFIG_BULK_DAPLINK
    memcpy(p + descriptor.configuration_length, desc_configuration_vendor, sizeof(desc_configuration_vendor));
    descriptor.configuration_length += sizeof(desc_configuration_vendor);
#else
    memcpy(p + descriptor.configuration_length, desc_configuration_hid, sizeof(desc_configuration_hid));
    descriptor.configuration_length += sizeof(desc_configuration_hid);
#endif

#ifdef CONFIG_TINYUSB_MSC_ENABLED
    if (enable_msc) {
        memcpy(p + descriptor.configuration_length, desc_configuration_msc, sizeof(desc_configuration_msc));
        descriptor.configuration_length += sizeof(desc_configuration_msc);
    }
#endif // CONFIG_TINYUSB_MSC_ENABLED

    return descriptor.desc_configuration;
}
