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

static wl_handle_t wl_handle;

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle)
{
    // ESP_LOGI(TAG, "Initializing wear levelling");

    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG, "Failed to find FATFS partition. Check the partition table.");
        return ESP_ERR_NOT_FOUND;
    }

    return wl_mount(data_partition, wl_handle);
}

bool msc_disk_mount(const char *path) {

    wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_spiflash_config_t config_spi = {
        .wl_handle = wl_handle,
        // .callback_mount_changed = storage_mount_changed_cb,  /* First way to register the callback. This is while initializing the storage. */
        // .mount_config.max_files = 5,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));
    // ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, storage_mount_changed_cb));
    return true;
}