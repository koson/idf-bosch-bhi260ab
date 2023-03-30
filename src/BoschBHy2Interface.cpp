
#include "BoschBHy2Interface.hpp"
#include "BHI260ABSensor.hpp"
#include "freertos/Freertos.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "I2C.hpp"
#include "common.h"
#include "../driver/bhy2.h"
#include "../driver/bhy2_parse.h"

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "bhi260ap/BHI260AP_BMM150-flash.fw.h"
#else
#include "BHI260AP_BMM150.fw.h"
#endif

#define WORK_BUFFER_SIZE  2048

#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_RV

using namespace std;

struct bhy2_dev bhy2Device;
Motion::BHI260ABSensor *_bmi260Sensor;

void boschDelayUs(uint32_t period, void *intf_ptr)
{
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
}

int8_t boschI2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    try
    {
        _bmi260Sensor->getBus()->syncWrite(I2CAddress(dev_addr), {reg_addr});
        vector<uint8_t> data = _bmi260Sensor->getBus()->syncRead(I2CAddress(dev_addr), len);
        memcpy(reg_data, data.data(), len);
        return ESP_OK;
    }
    catch (const I2CException &e)
    {
        ESP_LOGI("TAG", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
        ESP_LOGI("TAG", "Couldn't read sensor!");
        return ESP_FAIL;
    }
}

int8_t boschI2cWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    try
    {
        vector<uint8_t> data;
        data.push_back(reg_addr);
        for (int i = 0; i < len; i++)
        {
            data.push_back(reg_data[i]);
        }
        _bmi260Sensor->getBus()->syncWrite(I2CAddress(dev_addr), data);
        return ESP_OK;
    }
    catch (const I2CException &e)
    {
        ESP_LOGI("TAG", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
        ESP_LOGI("TAG", "Couldn't write sensor!");
        return ESP_FAIL;
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        exit(0);
    }
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void upload_firmware(uint8_t boot_stat)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHY2_OK;

#ifdef UPLOAD_FIRMWARE_TO_FLASH
    if (boot_stat & BHY2_BST_FLASH_DETECTED)
    {
        uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
        uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
        printf("Flash detected. Erasing flash to upload firmware\r\n");

        rslt = bhy2_erase_flash(start_addr, end_addr, &bhy2Device);
        print_api_error(rslt, &bhy2Device);
    }
    else
    {
        printf("Flash not detected\r\n");

        rslt = BHY2_E_IO;
        print_api_error(rslt, &bhy2Device);
    }

    printf("Loading firmware into FLASH.\r\n");
    rslt = bhy2_upload_firmware_to_flash(bhy2_firmware_image, sizeof(bhy2_firmware_image), &bhy2Device);
#else
    printf("Loading firmware into RAM.\r\n");
    rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), &bhy2Device);
#endif
    temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2Device);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, &bhy2Device);
    print_api_error(temp_rslt, &bhy2Device);

#ifdef UPLOAD_FIRMWARE_TO_FLASH
    printf("Booting from FLASH.\r\n");
    rslt = bhy2_boot_from_flash(&bhy2Device);
#else
    printf("Booting from RAM.\r\n");
    rslt = bhy2_boot_from_ram(&bhy2Device);
#endif

    temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2Device);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, &bhy2Device);
    print_api_error(temp_rslt, &bhy2Device);
}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %.2f\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

esp_err_t initBHy2(Motion::BHI260ABSensor *motionSensor)
{

    _bmi260Sensor = motionSensor;
    uint8_t product_id = 0;
    uint16_t version = 0;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    int8_t rslt = bhy2_init(BHY2_I2C_INTERFACE, boschI2cRead, boschI2cWrite, boschDelayUs, BHY2_RD_WR_LEN, NULL, &bhy2Device);
    print_api_error(rslt, &bhy2Device);

    rslt = bhy2_soft_reset(&bhy2Device);
    print_api_error(rslt, &bhy2Device);

    rslt = bhy2_get_product_id(&product_id, &bhy2Device);
    print_api_error(rslt, &bhy2Device);
    ESP_LOGD("BHy2", "BHI260AB found. Product ID read %X", product_id);

    /* Check if the sensor is ready to load firmware */
    rslt = bhy2_get_boot_status(&boot_status, &bhy2Device);
    print_api_error(rslt, &bhy2Device);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy2Device);

        rslt = bhy2_get_kernel_version(&version, &bhy2Device);
        print_api_error(rslt, &bhy2Device);
        if ((rslt == BHY2_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2Device);
        print_api_error(rslt, &bhy2Device);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2Device);
        print_api_error(rslt, &bhy2Device);
        rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, NULL, &bhy2Device);
        print_api_error(rslt, &bhy2Device);

        rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
        print_api_error(rslt, &bhy2Device);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");
        return ESP_FAIL;
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhy2_update_virtual_sensor_list(&bhy2Device);
    print_api_error(rslt, &bhy2Device);

    float sample_rate = 100.0; /* Read out data measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */
    rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &bhy2Device);
    print_api_error(rslt, &bhy2Device);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

    while (rslt == BHY2_OK)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
            print_api_error(rslt, &bhy2Device);
        }
    }

    return ESP_OK;
}