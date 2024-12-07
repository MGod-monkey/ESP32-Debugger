#include "programmer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/message_buffer.h"
#include "algo_extractor.h"
#include "esp_log.h"
#include "prog_idle.h"
#include "prog_online.h"
#include "prog_offline.h"
#include <sys/stat.h>
#include <cstring>

#define TAG "programmer"

static ProgData s_data;
static Prog *s_prog = nullptr;
static Prog *s_last_prog = nullptr;

prog_err_def programmer_request_handle(char *buf, int len)
{
    prog_request_swap_t swap = {buf, len};

    if (s_data.is_busy())
    {
        ESP_LOGW(TAG, "Programmer is busy");
        return PROG_ERR_BUSY;
    }

    s_data.set_swap(&swap);
    s_data.send_event(PROG_EVT_REQUEST);
    s_data.wait_sync();

    return static_cast<prog_err_def>(reinterpret_cast<int>(s_data.get_swap()));
}

static void programmer_switch_mode(prog_mode_def mode)
{
    static ProgIdle prog_idle;
    static ProgOnline prog_online;
    static ProgOffline prog_offline;

    s_last_prog = s_prog;

    switch (mode)
    {
    case PROG_ONLINE_MODE:
        s_prog = &prog_online;
        break;
    case PROG_OFFLINE_MODE:
        s_prog = &prog_offline;
        break;
    case PROG_IDLE_MODE:
        s_prog = &prog_idle;
        break;
    default:
        break;
    }

    if (s_last_prog && s_prog)
    {
        ESP_LOGI(TAG, "%s--> %s", s_last_prog->name(), s_prog->name());
    }
}

static void programmer_task(void *pvParameters)
{
    prog_evt_def evt = PROG_EVT_NONE;
    ProgData &obj = *(reinterpret_cast<ProgData *>(pvParameters));

    Prog::register_switch_mode_function(programmer_switch_mode);
    Prog::switch_mode(PROG_IDLE_MODE);

    for (;;)
    {
        evt = obj.wait_event();
        ESP_LOGI(TAG, "Received event: %d", evt);

        switch (evt)
        {
        case PROG_EVT_REQUEST:
            ESP_LOGI(TAG, "Handling PROG_EVT_REQUEST");
            s_prog->request_handle(obj);
            break;
        case PROG_EVT_PROGRAM_START:
            ESP_LOGI(TAG, "Handling PROG_EVT_PROGRAM_START");
            s_prog->program_start_handle(obj);
            break;
        case PROG_EVT_PROGRAM_TIMEOUT:
            ESP_LOGI(TAG, "Handling PROG_EVT_PROGRAM_TIMEOUT");
            s_prog->program_timeout_handle(obj);
            break;
        case PROG_EVT_PROGRAM_DATA_RECVED:
            ESP_LOGI(TAG, "Handling PROG_EVT_PROGRAM_DATA_RECVED");
            s_prog->program_data_handle(obj);
            break;
        default:
            ESP_LOGW(TAG, "Unknown event: %d", evt);
            break;
        }
    }
}

TaskHandle_t programmer_task_handle;  // 定义任务句柄

void programmer_init(void)
{
    if (FileProgrammer::is_exist(CONFIG_PROGRAMMER_ALGORITHM_ROOT) != true)
        mkdir(CONFIG_PROGRAMMER_ALGORITHM_ROOT, 0777);

    if (FileProgrammer::is_exist(CONFIG_PROGRAMMER_PROGRAM_ROOT) != true)
        mkdir(CONFIG_PROGRAMMER_PROGRAM_ROOT, 0777);

    s_data.init();
    xTaskCreate(programmer_task, "programmer", 1024 * 4, &s_data, 2, &programmer_task_handle);
    ESP_LOGI(TAG, "Prograprogrammer_taskmmer initialized");
}

void programmer_stop_task(void)
{
    vTaskDelete(programmer_task_handle);
    ESP_LOGI(TAG, "Prograprogrammer_taskmmer close");
}

void programmer_get_status(char *buf, int size, int *encode_len)
{
    *encode_len = snprintf(buf, size, "{\"progress\": %d, \"status\": \"%s\"}", 
                          s_data.get_progress(), 
                          s_data.is_busy() ? "busy" : "idle");
}
prog_err_def programmer_write_data(uint8_t *data, int len)
{
    prog_data_swap_t swap = {data, len};

    s_data.set_swap(&swap);
    s_data.send_event(PROG_EVT_PROGRAM_DATA_RECVED);
    s_data.wait_sync();

    return static_cast<prog_err_def>(reinterpret_cast<int>(s_data.get_swap()));
}