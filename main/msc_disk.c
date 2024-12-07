#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include <dirent.h>
#include <errno.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "diskio_impl.h"
#include "esp_check.h"
#include "diskio_sdmmc.h"

static const char *TAG = "msc_disk";

#ifdef CONFIG_MSC_STORAGE_MEDIA_SPIFLASH
// #include "wl.h" // 确保包含了wear leveling API

static wl_handle_t wl_handle = WL_INVALID_HANDLE; // 全局变量，便于卸载时访问

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    ESP_LOGI(TAG, "Initializing wear levelling");

    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL)
    {
        ESP_LOGE(TAG, "Failed to find FATFS partition. Check the partition table.");
        return ESP_ERR_NOT_FOUND;
    }

    return wl_mount(data_partition, wl_handle);
}
#else

#include "sdmmc_cmd.h" // 确保包含了SDMMC API

static sdmmc_card_t *card = NULL; // 全局变量，便于卸载时访问

static esp_err_t storage_init_sdmmc(sdmmc_card_t **card)
{
    esp_err_t ret = ESP_FAIL;
    bool host_init = false;
    sdmmc_card_t *sd_card;

    ESP_LOGI(TAG, "Initializing SDCard");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    slot_config.clk = GPIO_NUM_36;
    slot_config.cmd = GPIO_NUM_35;
    slot_config.d0 = GPIO_NUM_37;
    slot_config.d1 = GPIO_NUM_38;
    slot_config.d2 = GPIO_NUM_33;
    slot_config.d3 = GPIO_NUM_34;

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sd_card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    ESP_GOTO_ON_FALSE(sd_card, ESP_ERR_NO_MEM, clean, TAG, "could not allocate new sdmmc_card_t");

    ESP_GOTO_ON_ERROR((*host.init)(), clean, TAG, "Host Config Init fail");
    host_init = true;

    ESP_GOTO_ON_ERROR(sdmmc_host_init_slot(host.slot, (const sdmmc_slot_config_t *)&slot_config),
                      clean, TAG, "Host init slot fail");

    if (sdmmc_card_init(&host, sd_card) != ESP_OK)
    {
        ESP_LOGE(TAG, "The detection pin of the slot is disconnected (Insert uSD card)");
        goto clean;
    }

    sdmmc_card_print_info(stdout, sd_card);
    *card = sd_card;

    return ESP_OK;

clean:
    if (host_init)
    {
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)
        {
            host.deinit_p(host.slot);
        }
        else
        {
            (*host.deinit)();
        }
    }

    if (sd_card)
    {
        free(sd_card);
        sd_card = NULL;
    }

    return ret;
}

#endif

// mount the partition and show all the files in path
static bool storage_mounted = false; // 挂载状态标志

bool msc_disk_mount(const char *path) {
    if (storage_mounted) {
        ESP_LOGI(TAG, "Storage already mounted.");
        return true;
    }

#ifdef CONFIG_MSC_STORAGE_MEDIA_SPIFLASH
    static wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_spiflash_config_t config_spi = {.wl_handle = wl_handle};
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));
    ESP_LOGI(TAG, "xx...");
#else
    static sdmmc_card_t *card = NULL;

    if (storage_init_sdmmc(&card) != ESP_OK)
        return false;

    const tinyusb_msc_sdmmc_config_t config_sdmmc = {
        .card = card};

    ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));
#endif
    ESP_LOGI(TAG, "Mount storage...");

    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(path));
    storage_mounted = true;
    return true;
}

// 实现卸载函数
bool msc_disk_unmount(void) {
    if (!storage_mounted) {
        ESP_LOGI(TAG, "Storage not mounted.");
        return true;
    }

#ifdef CONFIG_MSC_STORAGE_MEDIA_SPIFLASH
    // 卸载 TinyUSB 的 MSC 存储
    esp_err_t ret = tinyusb_msc_storage_unmount();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount TinyUSB MSC storage for SPI Flash");
        return false;
    }
    ESP_LOGI(TAG, "TinyUSB MSC storage unmounted successfully for SPI Flash");

    // 反初始化 TinyUSB 的 SPI Flash MSC 存储
    ret = tinyusb_msc_storage_deinit_spiflash();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize TinyUSB MSC storage for SPI Flash");
        return false;
    }
    ESP_LOGI(TAG, "TinyUSB MSC storage deinitialized successfully for SPI Flash");

    // 重置 wear leveling 句柄
    wl_handle = WL_INVALID_HANDLE;
#else
    // 处理 SDMMC 存储的卸载逻辑
#endif

    storage_mounted = false;
    ESP_LOGI(TAG, "MSC storage unmounted successfully");
    return true;
}