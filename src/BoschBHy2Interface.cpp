
#include "BoschBHy2Interface.hpp"
#include "BHI260APSensor.hpp"
#include "freertos/Freertos.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "I2C.hpp"
#include "driver/gpio.h"
#include "common.hpp"
#include "../driver/bhy2.hpp"
#include "../driver/bhy2_parse.hpp"

// #define UPLOAD_FIRMWARE_TO_FLASH
#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "Bosch_APP30_SHUTTLE_BHI260_aux_BMM150-flash.fw.hpp"
#else
// #include "BHI260AP_BMM150.fw.hpp"
#include "PRO100_BHI260_aux_BMM150.fw.hpp"
#endif

#define WORK_BUFFER_SIZE 2048

#define QUAT_SENSOR_ID BHY2_SENSOR_ID_RV

using namespace std;

namespace Motion
{
    BHI260APSensor *_bmi260Sensor;
    struct bhy2_dev bhy2Device;

    void boschDelayUs(uint32_t period, void *intf_ptr)
    {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }

    int8_t boschI2cRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
    {
        (void)intf_ptr;
        try
        {
            _bmi260Sensor->getBus()->syncWrite(I2CAddress(CONFIG_BHI260AP_ADDRESS), {reg_addr});
            vector<uint8_t> data = _bmi260Sensor->getBus()->syncRead(I2CAddress(CONFIG_BHI260AP_ADDRESS), len);
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
        (void)intf_ptr;
        try
        {
            vector<uint8_t> data;
            data.push_back(reg_addr);
            for (int i = 0; i < len; i++)
            {
                data.push_back(reg_data[i]);
            }
            _bmi260Sensor->getBus()->syncWrite(I2CAddress(CONFIG_BHI260AP_ADDRESS), data);
            return ESP_OK;
        }
        catch (const I2CException &e)
        {
            ESP_LOGI("TAG", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
            ESP_LOGI("TAG", "Couldn't write sensor!");
            return ESP_FAIL;
        }
    }

    static void print_api_error(int8_t rslt)
    {
        if (rslt != BHY2_OK)
        {
            printf("%s\r\n", get_api_error(rslt));
            printf("Exiting...");
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
            print_api_error(rslt);
        }
        else
        {
            printf("Flash not detected\r\n");

            rslt = BHY2_E_IO;
            print_api_error(rslt);
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
            printf("1. %s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt);
        print_api_error(temp_rslt);
        printf("firmware loaded.\r\n");

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

        print_api_error(rslt);
        print_api_error(temp_rslt);
    }

    void configItr()
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_NUM_36);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
    }

    void configReset()
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_NUM_35);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(GPIO_NUM_35, 0);

        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_level(GPIO_NUM_35, 1);
    }

    esp_err_t initBHy2(Motion::BHI260APSensor *motionSensor, bhy2_fifo_parse_callback_t quaternionCallback)
    {

        _bmi260Sensor = motionSensor;
        uint8_t product_id = 0;
        uint16_t version = 0;
        uint8_t work_buffer[WORK_BUFFER_SIZE];
        uint8_t hintr_ctrl, hif_ctrl, boot_status;

        configReset();
        configItr();

        int8_t rslt = bhy2_init(BHY2_I2C_INTERFACE, boschI2cRead, boschI2cWrite, boschDelayUs, BHY2_RD_WR_LEN, NULL, &bhy2Device);
        print_api_error(rslt);

        rslt = bhy2_soft_reset(&bhy2Device);
        print_api_error(rslt);

        rslt = bhy2_get_product_id(&product_id, &bhy2Device);
        print_api_error(rslt);
        ESP_LOGI("BHy2", "BHI260AP found. Product ID read %X", product_id);

        /* Check the interrupt pin and FIFO configurations. Disable status and debug */

        hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

        rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2Device);
        print_api_error(rslt);
        rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2Device);
        print_api_error(rslt);

        printf("Host interrupt control\r\n");
        printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
        printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
        printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
        printf("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
        printf("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
        printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
        printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
        printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

        /* Configure the host interface */
        hif_ctrl = 0;
        rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2Device);
        print_api_error(rslt);

        /* Check if the sensor is ready to load firmware */
        rslt = bhy2_get_boot_status(&boot_status, &bhy2Device);
        print_api_error(rslt);

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
        {
            upload_firmware(boot_status);

            rslt = bhy2_get_kernel_version(&version, &bhy2Device);
            print_api_error(rslt);
            if ((rslt == BHY2_OK) && (version != 0))
            {
                printf("Boot successful. Kernel version %u.\r\n", version);
            }

            rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2Device);
            print_api_error(rslt);
            rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2Device);
            print_api_error(rslt);
            rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, quaternionCallback, NULL, &bhy2Device);
            print_api_error(rslt);

            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
            print_api_error(rslt);
        }
        else
        {
            printf("Host interface not ready. Exiting\r\n");
            return ESP_FAIL;
        }

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhy2_update_virtual_sensor_list(&bhy2Device);
        print_api_error(rslt);

        float sample_rate = 10.0;       /* Read out data measured at 100Hz */
        uint32_t report_latency_ms = 0; /* Report immediately */
        rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &bhy2Device);
        print_api_error(rslt);
        printf("Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

        while (rslt == BHY2_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(GPIO_NUM_36))
            {
                /* Data from the FIFO is read and the relevant callbacks if registered are called */
                rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
                print_api_error(rslt);
            }
        }
        return ESP_OK;
    }
}