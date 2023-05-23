#include "BHI260APSensor.hpp"
#include "BoschBHy2Interface.hpp"

namespace Motion
{
    esp_err_t BHI260APSensor::init(FILE *firmware, uint32_t len)
    {
        return initBHy2(this, firmware, len);
    }
    esp_err_t BHI260APSensor::startSensorLoop(bhy2_fifo_parse_callback_t callback)
    {
        return startLoop(callback);
    }
}
