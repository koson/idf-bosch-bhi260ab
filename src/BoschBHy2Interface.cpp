
#include "BoschBHy2Interface.hpp"
#include "BHI260ABSensor.hpp"
#include "freertos/Freertos.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "I2C.hpp"
#include "bhy2.h"
#include "bhy2_parse.h"

#define NUM_USED_OUTPUTS 9

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
        _bme680Sensor->getBus()->syncWrite(I2CAddress(dev_addr), {reg_addr});
        vector<uint8_t> data = _bme680Sensor->getBus()->syncRead(I2CAddress(dev_addr), len);
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
        _bme680Sensor->getBus()->syncWrite(I2CAddress(dev_addr), data);
        return ESP_OK;
    }
    catch (const I2CException &e)
    {
        ESP_LOGI("TAG", "I2C Exception with error: %s (0x%X)", e.what(), e.error);
        ESP_LOGI("TAG", "Couldn't write sensor!");
        return ESP_FAIL;
    }
}

esp_err_t initBHy2(Motion::BHI260ABSensor *motionSensor)
{

    _bmi260Sensor = motionSensor;
    uint8_t product_id = 0;

    int8_t rslt = bhy2_init(BHY2_I2C_INTERFACE, boschI2cRead, boschI2cWrite, boschDelayUs, BHY2_RD_WR_LEN, NULL, &bhy2Device);
    if (rslt != BHY2_OK)
    {
        ESP_LOGD("BHy2", "%s", get_api_error(rslt));
        return rslt;
    }
    rslt = bhy2_soft_reset(&bhy2Device);
    if (rslt != BHY2_OK)
    {
        ESP_LOGD("BHy2", "%s", get_api_error(rslt));
        return rslt;
    }

    rslt = bhy2_get_product_id(&product_id, &bhy2Device);
    if (rslt != BHY2_OK)
    {
        ESP_LOGD("BHy2", "%s", get_api_error(rslt));
        return rslt;
    }

    ESP_LOGD("BHy2", "BHI260AB found. Product ID read %X", product_id);

    return ESP_OK;
}

void startBHy2Loop()
{
    // TODO
}