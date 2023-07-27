
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
#include "nvs_flash.h"
#ifdef CONFIG_BHI_MOTION_ACTIVE

#define WORK_BUFFER_SIZE 2048

#define QUAT_SENSOR_ID BHY2_SENSOR_ID_RV
#define EULER_SENSOR_ID BHY2_SENSOR_ID_ORI_WU

#define EULER
#ifdef EULER
#define SENSOR_ID EULER_SENSOR_ID
#else
#define SENSOR_ID QUAT_SENSOR_ID
#endif

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
            _bmi260Sensor->getBus()->syncWrite(I2CAddress(CONFIG_BHI_MOTION_ADDRESS), {reg_addr});
            vector<uint8_t> data = _bmi260Sensor->getBus()->syncRead(I2CAddress(CONFIG_BHI_MOTION_ADDRESS), len);
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
            _bmi260Sensor->getBus()->syncWrite(I2CAddress(CONFIG_BHI_MOTION_ADDRESS), data);
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
            ESP_LOGE("BHy2", "%s", get_api_error(rslt));
            ESP_LOGE("BHy2", "Exiting...");
        }
    }

    static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
    {
        (void)callback_ref;
        uint8_t meta_event_type = callback_info->data_ptr[0];
        uint8_t byte1 = callback_info->data_ptr[1];
        uint8_t byte2 = callback_info->data_ptr[2];
#ifdef EULER
        uint8_t *accuracy = (uint8_t *)callback_ref;
#endif
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
            ESP_LOGI("BHy2", "%s Flush complete for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            ESP_LOGI("BHy2", "%s Sample rate changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            ESP_LOGI("BHy2", "%s Power mode changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            ESP_LOGI("BHy2", "%s Algorithm event", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            ESP_LOGI("BHy2", "%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
#ifdef EULER
            if (accuracy)
            {
                *accuracy = byte2;
                if (byte2 >= 1)
                {
                    uint8_t calib_prof[512];
                    uint32_t actual_len = 0;
                    uint8_t r = bhy2_get_calibration_profile(UINT16_C(0x205), calib_prof, sizeof(calib_prof), &actual_len, &bhy2Device);
                    if (r == BHY2_OK)
                    {
                        nvs_handle handle;
                        nvs_open("fusion", NVS_READWRITE, &handle);
                        nvs_set_blob(handle, "mag_cal", calib_prof, actual_len);
                        nvs_set_u32(handle, "mag_len", actual_len);
                        nvs_commit(handle);
                        nvs_close(handle);
                    }
                }
            }
#endif
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            ESP_LOGI("BHy2", "%s BSX event (do steps main)", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            ESP_LOGI("BHy2", "%s BSX event (do steps calib)", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            ESP_LOGI("BHy2", "%s BSX event (get output signal)", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            ESP_LOGI("BHy2", "%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            ESP_LOGI("BHy2", "%s FIFO overflow", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            ESP_LOGI("BHy2", "%s Dynamic range changed for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            ESP_LOGI("BHy2", "%s FIFO watermark reached", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            ESP_LOGI("BHy2", "%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            ESP_LOGI("BHy2", "%s Transfer cause for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            ESP_LOGI("BHy2", "%s Sensor framework event for sensor id %u", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            ESP_LOGI("BHy2", "%s Reset event", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            ESP_LOGI("BHy2", "%s Unknown meta event with id: %u", event_text, meta_event_type);
            break;
        }
    }

    static uint8_t upload_firmware(uint8_t boot_stat, FILE *file, uint32_t len)
    {
        uint16_t version = 0;
        uint8_t sensor_error;
        int8_t temp_rslt;
        int8_t rslt = BHY2_OK;

#ifdef CONFIG_BHI_USE_FLASH
        if (boot_stat & BHY2_BST_FLASH_DETECTED)
        {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
            ESP_LOGI("BHy2", "Flash detected. Erasing flash to upload firmware");

            rslt = bhy2_erase_flash(start_addr, end_addr, &bhy2Device);
            print_api_error(rslt);
        }
        else
        {
            ESP_LOGW("BHy2", "Flash not detected");

            rslt = BHY2_E_IO;
            print_api_error(rslt);
            return rslt;
        }
#endif
        ESP_LOGI("BHy2", "Start upload firmware");

        uint32_t incr = 256; /* Max command packet size */

        if ((incr % 4) != 0) /* Round off to higher 4 bytes */
        {
            incr = ((incr >> 2) + 1) << 2;
        }

        for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
        {
            if (incr > (len - i)) /* If last payload */
            {
                incr = len - i;
                if ((incr % 4) != 0) /* Round off to higher 4 bytes */
                {
                    incr = ((incr >> 2) + 1) << 2;
                }
            }

#ifdef CONFIG_BHI_USE_FLASH
            rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, &bhy2Device);
#else
            uint8_t firmware[incr];
            size_t read = fread(firmware, sizeof(uint8_t), incr, file);
            rslt = bhy2_upload_firmware_to_ram_partly(firmware, len, i, incr, &bhy2Device);
#endif

            printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
        }

        printf("\n");

#ifdef CONFIG_BHI_USE_FLASH
        printf("Booting from Flash.\r\n");
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

        rslt = bhy2_get_kernel_version(&version, &bhy2Device);
        print_api_error(rslt);
        if ((rslt == BHY2_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }
        uint8_t calib_prof[512];
        uint32_t actual_len = 0;
        nvs_handle handle;
        nvs_open("fusion", NVS_READWRITE, &handle);
        nvs_get_u32(handle, "mag_len", &actual_len);
        if (actual_len > 0)
        {
            nvs_get_blob(handle, "mag_cal", calib_prof, (size_t *)&actual_len);
            nvs_close(handle);
            uint8_t r = bhy2_set_calibration_profile(BHY2_PARAM_BSX_CALIB_STATE_MAG, calib_prof, actual_len, &bhy2Device);
            ESP_LOGI("BHy2", "LOAD CALIBRATION RESULT:%u size:%ld", r, actual_len);
        }
        return rslt;
    }

    static void tiltDetectionCallback(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
    {
        (void)callback_ref;
        uint32_t currentTimeS = {0};
        if (callback_info->data_size != 1) /* Check for a valid payload size. Includes sensor ID */
        {
            ESP_LOGE("BHy2", "Unexpected Tilt payload! size: %d", callback_info->data_size);
            return;
        }

        uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */
        timestamp = timestamp * 15625;                   /* Timestamp is now in nanoseconds */
        currentTimeS = (uint32_t)(timestamp / UINT64_C(1000000000));

        ESP_LOGI("BHy2", "Tilt detected! SID: %u; T: %lu", callback_info->sensor_id, currentTimeS);
    }

    void configItr()
    {
        if (CONFIG_PIN_BHI_INTERRUPT > GPIO_NUM_NC)
        {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = (1ULL << CONFIG_PIN_BHI_INTERRUPT);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_config(&io_conf);
        }
    }

    void configReset()
    {
        if (CONFIG_PIN_BHI_RESET > GPIO_NUM_NC)
        {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pin_bit_mask = (1ULL << CONFIG_PIN_BHI_RESET);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_config(&io_conf);
            gpio_set_level((gpio_num_t)CONFIG_PIN_BHI_RESET, 0);

            vTaskDelay(pdMS_TO_TICKS(10));

            gpio_set_level((gpio_num_t)CONFIG_PIN_BHI_RESET, 1);
        }
    }

    esp_err_t initBHy2(Motion::BHI260APSensor *motionSensor, FILE *firmware, uint32_t len)
    {

        _bmi260Sensor = motionSensor;
        uint8_t product_id = 0;
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

        ESP_LOGI("BHy2", "Host interrupt control");
        ESP_LOGI("BHy2", "    Wake up FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
        ESP_LOGI("BHy2", "    Non wake up FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
        ESP_LOGI("BHy2", "    Status FIFO %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
        ESP_LOGI("BHy2", "    Debugging %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
        ESP_LOGI("BHy2", "    Fault %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
        ESP_LOGI("BHy2", "    Interrupt is %s.", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
        ESP_LOGI("BHy2", "    Interrupt is %s triggered.", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
        ESP_LOGI("BHy2", "    Interrupt pin drive is %s.", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

        /* Configure the host interface */
        hif_ctrl = 0;
        rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2Device);
        print_api_error(rslt);

        /* Check if the sensor is ready to load firmware */
        rslt = bhy2_get_boot_status(&boot_status, &bhy2Device);
        print_api_error(rslt);

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
        {
            uint8_t rsl = upload_firmware(boot_status, firmware, len);
            if (rsl == BHY2_OK)
            {
                return ESP_OK;
            }
        }
        ESP_LOGE("BHy2", "Host interface not ready. Exiting");
        return ESP_FAIL;
    }

    esp_err_t startLoop(bhy2_fifo_parse_callback_t dataCallback)
    {
        uint16_t version = 0;
        uint8_t work_buffer[WORK_BUFFER_SIZE];
#ifdef EULER
        uint8_t accuracy;
#endif

        int8_t rslt = bhy2_get_kernel_version(&version, &bhy2Device);
        print_api_error(rslt);
        if (rslt == BHY2_OK)
        {
            ESP_LOGI("BHy2", "Boot successful. Kernel version %u.", version);
        }

#ifdef EULER
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void *)&accuracy, &bhy2Device);
        print_api_error(rslt);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, (void *)&accuracy, &bhy2Device);
        print_api_error(rslt);
        rslt = bhy2_register_fifo_parse_callback(SENSOR_ID, dataCallback, (void *)&accuracy, &bhy2Device);
#else
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2Device);
        print_api_error(rslt);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2Device);
        print_api_error(rslt);
        rslt = bhy2_register_fifo_parse_callback(SENSOR_ID, dataCallback, NULL, &bhy2Device);
#endif
        print_api_error(rslt);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TILT_DETECTOR, tiltDetectionCallback, (void *)&accuracy, &bhy2Device);
        print_api_error(rslt);

        rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
        print_api_error(rslt);
        /* Update the callback table to enable parsing of sensor data */
        rslt = bhy2_update_virtual_sensor_list(&bhy2Device);
        print_api_error(rslt);


        // rslt = bhy2_set_virt_sensor_cfg(SENSOR_ID, 50.0, 0, &bhy2Device);
        // print_api_error(rslt);
        // ESP_LOGI("BHy2", "Enable %s.", get_sensor_name(SENSOR_ID));

        rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_TILT_DETECTOR, 50.0, 0, &bhy2Device);
        print_api_error(rslt);
        ESP_LOGI("BHy2", "Enabled %s.", get_sensor_name(BHY2_SENSOR_ID_TILT_DETECTOR));

        while (rslt == BHY2_OK)
        {

            vTaskDelay(pdMS_TO_TICKS(100));
            if (CONFIG_PIN_BHI_INTERRUPT > GPIO_NUM_NC && gpio_get_level((gpio_num_t)CONFIG_PIN_BHI_INTERRUPT))
            {
                /* Data from the FIFO is read and the relevant callbacks if registered are called */
                rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2Device);
            }
            print_api_error(rslt);
        }

        return ESP_OK;
    }
}
#endif